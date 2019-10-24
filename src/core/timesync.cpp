#include "timesync.h"
#include "system_impl.h"
#include "log.h"
#include <cmath>
#include <functional>
#include <string>
#include <array>

// Source: https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/sys_time.cpp

namespace mavsdk {

Timesync::Timesync(SystemImpl& parent) : _parent(parent)
{
    using namespace std::placeholders; // for `_1`

    _parent.register_mavlink_message_handler(
        MAVLINK_MSG_ID_SYSTEM_TIME,
        std::bind(&Timesync::process_system_time, this, _1),
        this);

    _parent.register_mavlink_message_handler(
        MAVLINK_MSG_ID_TIMESYNC, std::bind(&Timesync::process_timesync, this, _1), this);

    LogDebug() << "Timesync plugin started.";
}

Timesync::~Timesync()
{
    _parent.unregister_all_mavlink_message_handlers(this);
}

void Timesync::do_work()
{
    if (_parent.get_time().elapsed_since_s(_last_time) >= _TIMESYNC_SEND_INTERVAL_S) {
        if (_parent.is_connected()) {
            uint64_t now_ns = _parent.get_time().get_system_time_since_epoch<std::chrono::nanoseconds>().count();
            send_timesync(0, now_ns);
        }
        _last_time = _parent.get_time().steady_time();
    }
}

void Timesync::process_system_time(const mavlink_message_t& message)
{
    mavlink_system_time_t system_time;

    LogDebug() << "FCU System time received.";

    mavlink_msg_system_time_decode(&message, &system_time);

    // date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
    const bool fcu_time_valid = system_time.time_unix_usec > 1234567890ULL * 1000000;

    if (!fcu_time_valid) {
        LogWarn() << "Wrong FCU time.";
        send_sys_time();
        LogInfo() << "FCU time updated.";
    } else {
        LogDebug() << "FCU system time: " << fcu_time_valid;
    }

}

void Timesync::process_timesync(const mavlink_message_t& message)
{
    mavlink_timesync_t timesync;

    mavlink_msg_timesync_decode(&message, &timesync);

    uint64_t now_ns = _parent.get_time().get_system_time_since_epoch<std::chrono::nanoseconds>().count();

    if (timesync.tc1 == 0) {
        send_timesync(now_ns, timesync.ts1);
        return;
    } else if (timesync.tc1 > 0) {
        // Time offset between this system and the remote system is calculated assuming RTT for
        // the timesync packet is roughly equal both ways.

        LogDebug() << "FCU TIMESYNC received";
        LogDebug() << "FCU TIMESYNC: fcutime=" << int64_t(timesync.tc1 / 1e6) << " ms";
        LogDebug() << "FCU TIMESYNC: localtime=" << int64_t(timesync.ts1 / 1e6) << " ms";
        LogDebug() << "FCU TIMESYNC: RTT=" << int64_t(((now_ns - timesync.ts1) / 2.0) / 1e6) << " ms";

        add_timesync_observation(
            (timesync.ts1 + now_ns - timesync.tc1 * 2) / 2, timesync.ts1);
        LogDebug() << "";
    }
}

void Timesync::send_timesync(uint64_t tc1, uint64_t ts1)
{
    mavlink_message_t message;

    mavlink_msg_timesync_pack(
        _parent.get_own_system_id(),
        _parent.get_own_component_id(),
        &message,
        tc1,
        ts1);
    _parent.send_message(message);
}

bool Timesync::send_sys_time()
{
    mavlink_message_t message;

    double now_us = std::chrono::duration_cast<std::chrono::microseconds>(_parent.get_time().get_system_time_since_epoch<std::chrono::microseconds>()).count();

    mavlink_msg_system_time_pack(
        _parent.get_own_system_id(),
        _parent.get_own_component_id(),
        &message,
        now_us,
        0);
    return _parent.send_message(message);
}

void Timesync::add_timesync_observation(int64_t offset_ns, uint64_t local_time_ns)
{
    uint64_t now_ns = _parent.get_time().get_system_time_since_epoch<std::chrono::nanoseconds>().count();

    // Calculate the round trip time (RTT) it took the timesync packet to bounce back to us from remote system
    uint64_t rtt_ns = now_ns - local_time_ns;

    if (rtt_ns < _MAX_RTT_SAMPLE_MS * 1000000ULL) {	// Only use samples with low RTT
        // Filter gain scheduling
        if (!sync_converged()) {
            // Interpolate with a sigmoid function
            double progress = _sequence / _CONVERGENCE_WINDOW;
            double p = 1.0 - exp(0.5 * (1.0 - 1.0 / (1.0 - progress)));
            _filter_alpha = p * _FILTER_ALPHA_FINAL + (1.0 - p) * _filter_alpha_initial;
            _filter_beta = p * _FILTER_BETA_FINAL + (1.0 - p) * _filter_beta_initial;
        } else {
            _filter_alpha = _FILTER_ALPHA_FINAL;
            _filter_beta = _FILTER_BETA_FINAL;
        }

        // Perform filter update
        add_sample(offset_ns);

        // Save time offset for other components to use
        auto new_offset_ns = std::chrono::nanoseconds(static_cast<int64_t>(-1 * _time_offset_ns));
        _parent.get_time().set_fcu_time_offset(new_offset_ns);
        LogDebug() << "FCU TIMESYNC: offset=" << int64_t(std::chrono::duration_cast<std::chrono::milliseconds>(new_offset_ns).count()) << " ms";
        int64_t local_time_in_past_ns = local_time_ns - 10 * 1e9;
        LogDebug() << "FCU TIMESYNC: test: " << local_time_in_past_ns / 1e6 << " ms on sys eq "
                   << int64_t(std::chrono::duration_cast<std::chrono::milliseconds>(_parent.get_time().get_fcu_time(std::chrono::nanoseconds(local_time_in_past_ns))).count()) << " ms on FCU";
        LogDebug() << "FCU TIMESYNC: FCU time now=" << _parent.get_time().get_fcu_time<std::chrono::milliseconds>().count() << " ms";

        // Increment _sequence counter after filter update
        _sequence++;

        // Reset high RTT count after filter update
        _high_rtt_count = 0;
    } else {
        // Increment counter if round trip time is too high for accurate timesync
        _high_rtt_count++;

        if (_high_rtt_count > _MAX_CONS_HIGH_RTT) {
            // Issue a warning to the user if the RTT is constantly high
            LogWarn() << "TM : RTT too high for timesync: " << rtt_ns / 1000000.0 << " ms.";

            // Reset counter
            _high_rtt_count = 0;
        }
    }
}

void Timesync::add_sample(uint64_t offset_ns)
{
    /* Online exponential smoothing filter. The derivative of the estimate is also
     * estimated in order to produce an estimate without steady state lag:
     * https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
     */

    auto time_offset_prev = _time_offset_ns;

    if (_sequence == 0) {			// First offset sample
        _time_offset_ns = offset_ns;
    } else {
        // Update the clock offset estimate
        _time_offset_ns = _filter_alpha * offset_ns + (1.0 - _filter_alpha) * (_time_offset_ns + _time_skew);

        // Update the clock skew estimate
        _time_skew = _filter_beta * (_time_offset_ns - time_offset_prev) + (1.0 - _filter_beta) * _time_skew;
    }
}

} // namespace mavsdk

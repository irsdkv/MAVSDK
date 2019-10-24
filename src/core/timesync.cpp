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
        MAVLINK_MSG_ID_TIMESYNC,
        std::bind(&Timesync::process_timesync, this, _1),
        this);
    LogDebug() << "Timesync plugin started.";
}

Timesync::~Timesync()
{
    _parent.unregister_all_mavlink_message_handlers(this);
}

void Timesync::process_system_time(const mavlink_message_t& message)
{
    mavlink_system_time_t system_time;

    mavlink_msg_system_time_decode(&message, &system_time);

    // date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
    const bool fcu_time_valid = system_time.time_unix_usec > 1234567890ULL * 1000000;

    LogDebug() << "FCU System time received.";

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

    std::chrono::nanoseconds now_ns = _parent.get_time().steady_time().time_since_epoch();

    if (timesync.tc1 == 0) {
        send_timesync(now_ns.count(), timesync.ts1);
        return;
    } else if (timesync.tc1 > 0) {
        // Time offset between this system and the remote system is calculated assuming RTT for
        // the timesync packet is roughly equal both ways.
        add_timesync_observation(
            (timesync.ts1 + now_ns.count() - timesync.tc1 * 2) / 2, timesync.ts1);
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

void Timesync::send_sys_time()
{
    mavlink_message_t message;

    auto now = std::chrono::duration_cast<std::chrono::microseconds>(_parent.get_time().steady_time().time_since_epoch());

    mavlink_msg_system_time_pack(
        _parent.get_own_system_id(),
        _parent.get_own_component_id(),
        &message,
        now.count(),
        0);
    _parent.send_message(message);
}

void Timesync::add_timesync_observation(int64_t offset_ns, uint64_t local_time_ns)
{
    std::chrono::nanoseconds now_ns = _parent.get_time().steady_time().time_since_epoch();

    // Calculate the round trip time (RTT) it took the timesync packet to bounce back to us from remote system
    uint64_t rtt_ns = now_ns.count() - local_time_ns;

    // Calculate the difference of this sample from the current estimate
    uint64_t deviation = llabs(static_cast<int64_t >(_time_offset_ns) - offset_ns);

    if (rtt_ns < MAX_RTT_SAMPLE_MS * 1000000ULL) {	// Only use samples with low RTT
        if (sync_converged() && (deviation > MAX_DEVIATION_SAMPLE_MS * 1000000ULL)) {
            // Increment the counter if we have a good estimate and are getting samples far from the estimate
            _high_deviation_count++;

            // We reset the filter if we received consecutive samples which violate our present estimate.
            // This is most likely due to a time jump on the offboard system.
            if (_high_deviation_count > MAX_CONS_HIGH_DEVIATION) {
                LogErr() << "TM : Time jump detected. Resetting time synchroniser.";

                // Reset the filter
                reset_filter();
            }
        } else {
            // Filter gain scheduling
            if (!sync_converged()) {
                // Interpolate with a sigmoid function
                double progress = _sequence / CONVERGENCE_WINDOW;
                double p = 1.0 - exp(0.5 * (1.0 - 1.0 / (1.0 - progress)));
                _filter_alpha = p * _filter_alpha_final + (1.0 - p) * _filter_alpha_initial;
                _filter_beta = p * _filter_beta_final + (1.0 - p) * _filter_beta_initial;
            } else {
                _filter_alpha = _filter_alpha_final;
                _filter_beta = _filter_beta_final;
            }

            // Perform filter update
            add_sample(offset_ns);

            // Save time offset for other components to use
            _parent.get_time().set_fcu_time_offset(std::chrono::nanoseconds(sync_converged() ? static_cast<int64_t>(_time_offset_ns) : 0));

            // Increment _sequence counter after filter update
            _sequence++;

            // Reset high deviation count after filter update
            _high_deviation_count = 0;

            // Reset high RTT count after filter update
            _high_rtt_count = 0;
        }
    } else {
        // Increment counter if round trip time is too high for accurate timesync
        _high_rtt_count++;

        if (_high_rtt_count > MAX_CONS_HIGH_RTT) {
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

void Timesync::reset_filter()
{
    // Do a full reset of all statistics and parameters
    _sequence = 0;
    _time_offset_ns = 0.0;
    _time_skew = 0.0;
    _filter_alpha = _filter_alpha_initial;
    _filter_beta = _filter_beta_initial;
    _high_deviation_count = 0;
    _high_rtt_count = 0;
}

} // namespace mavsdk

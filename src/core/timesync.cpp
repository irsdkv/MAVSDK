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
        MAVLINK_MSG_ID_TIMESYNC, std::bind(&Timesync::process_timesync, this, _1), this);
}

Timesync::~Timesync()
{
    _parent.unregister_all_mavlink_message_handlers(this);
}

void Timesync::do_work()
{
    if (_parent.get_time().elapsed_since_s(_last_time) >= _TIMESYNC_SEND_INTERVAL_S) {
        if (_parent.is_connected()) {
            uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(_parent.get_time().system_time().time_since_epoch()).count();
            send_timesync(0, now_ns);
        }
        _last_time = _parent.get_time().steady_time();
    }
}

void Timesync::process_timesync(const mavlink_message_t& message)
{
    mavlink_timesync_t timesync;

    mavlink_msg_timesync_decode(&message, &timesync);

    uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(_parent.get_time().system_time().time_since_epoch()).count();

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

        set_timesync_offset((timesync.ts1 + now_ns - timesync.tc1 * 2) / 2, timesync.ts1);
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

void Timesync::set_timesync_offset(int64_t offset_ns, uint64_t local_time_ns)
{
    uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(_parent.get_time().system_time().time_since_epoch()).count();

    // Calculate the round trip time (RTT) it took the timesync packet to bounce back to us from remote system
    uint64_t rtt_ns = now_ns - local_time_ns;

    if (rtt_ns < _MAX_RTT_SAMPLE_MS * 1000000ULL) {	// Only use samples with low RTT

        // Save time offset for other components to use
        auto new_offset = std::chrono::nanoseconds(static_cast<int64_t>(-1 * offset_ns));
        _parent.get_fcu_time().shift_fcu_time_by(new_offset);
        LogDebug() << "FCU TIMESYNC: offset=" << int64_t(std::chrono::duration_cast<std::chrono::milliseconds>(new_offset).count()) << " ms";
        int64_t local_time_in_past_ns = local_time_ns - 10 * 1e9;
        LogDebug() << "FCU TIMESYNC: test: " << local_time_in_past_ns / 1e6 << " ms on sys eq "
                   << std::chrono::duration_cast<std::chrono::milliseconds>(_parent.get_fcu_time().time_in(std::chrono::nanoseconds(local_time_in_past_ns)).time_since_epoch()).count() << " ms on FCU";
        LogDebug() << "FCU TIMESYNC: FCU time now=" << std::chrono::duration_cast<std::chrono::milliseconds>(_parent.get_fcu_time().now().time_since_epoch()).count() << " ms";

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

} // namespace mavsdk

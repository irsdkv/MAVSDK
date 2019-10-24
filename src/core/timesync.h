#pragma once

#include <atomic>
#include <mutex>

#include "mavlink_include.h"

// Since not all vehicles support/require level calibration, this
// is disabled for now.
//#define LEVEL_CALIBRATION

namespace mavsdk {

class SystemImpl;

class Timesync {
public:
    Timesync(SystemImpl& parent);
    ~Timesync();

    Timesync(const Timesync&) = delete;
    Timesync& operator=(const Timesync&) = delete;
private:
    SystemImpl& _parent;

    void process_system_time(const mavlink_message_t& message);
    void process_timesync(const mavlink_message_t& message);
    void send_timesync(uint64_t tc1, uint64_t ts1);
    void send_sys_time();
    void add_timesync_observation(int64_t offset_ns, uint64_t local_time_ns);
    void reset_filter();
    void add_sample(uint64_t offset_ns);

    inline bool sync_converged()
    {
        return _sequence >= CONVERGENCE_WINDOW;
    }

    // Estimated statistics
    double _time_offset_ns{};
    double _time_skew{};

    // Filter parameters
    uint32_t _sequence{};
    double _filter_alpha{};
    double _filter_beta{};

    // Filter settings
    double _filter_alpha_initial{};
    double _filter_beta_initial{};
    double _filter_alpha_final{};
    double _filter_beta_final{};

    // Filter gain scheduling
    //
    // The filter interpolates between the initial and final gains while the number of
    // exhanged timesync packets is less than convergence_window. A lower value will
    // allow the timesync to converge faster, but with potentially less accurate initial
    // offset and skew estimates.
    const uint32_t CONVERGENCE_WINDOW{500};

    // Outlier rejection and filter reset
    //
    // Samples with round-trip time higher than max_rtt_sample are not used to update the filter.
    // More than max_consecutive_high_rtt number of such events in a row will throw a warning
    // but not reset the filter.
    // Samples whose calculated clock offset is more than max_deviation_sample off from the current
    // estimate are not used to update the filter. More than max_consecutive_high_deviation number
    // of such events in a row will reset the filter. This usually happens only due to a time jump
    // on the remote system.
    uint64_t MAX_RTT_SAMPLE_MS{10};
    uint64_t MAX_DEVIATION_SAMPLE_MS{100};
    uint64_t MAX_CONS_HIGH_RTT{5};
    uint64_t MAX_CONS_HIGH_DEVIATION{5};
    uint64_t _high_rtt_count{};
    uint64_t _high_deviation_count{};

};
} // namespace mavsdk

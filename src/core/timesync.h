#pragma once

#include <atomic>
#include <mutex>

#include "global_include.h"
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

    void do_work();

    Timesync(const Timesync&) = delete;
    Timesync& operator=(const Timesync&) = delete;
private:
    SystemImpl& _parent;

    void process_system_time(const mavlink_message_t& message);
    void process_timesync(const mavlink_message_t& message);
    void send_timesync(uint64_t tc1, uint64_t ts1);
    bool send_sys_time();
    void add_timesync_observation(int64_t offset_ns, uint64_t local_time_ns);
    void add_sample(uint64_t offset_ns);

    inline bool sync_converged()
    {
        return _sequence >= _CONVERGENCE_WINDOW;
    }

    static constexpr double _TIMESYNC_SEND_INTERVAL_S = 5.0;
    dl_time_t _last_time{};

    std::atomic_bool work{true};

    // Estimated statistics
    double _time_offset_ns{};
    double _time_skew{};

    // Filter gains
    //
    // Alpha : Used to smooth the overall clock offset estimate. Smaller values will lead
    // to a smoother estimate, but track time drift more slowly, introducing a bias
    // in the estimate. Larger values will cause low-amplitude oscillations.
    //
    // Beta : Used to smooth the clock skew estimate. Smaller values will lead to a
    // tighter estimation of the skew (derivative), but will negatively affect how fast the
    // filter reacts to clock skewing (e.g cause by temperature changes to the oscillator).
    // Larger values will cause large-amplitude oscillations.
    static constexpr uint64_t _CONVERGENCE_WINDOW = 500;
    static constexpr double _FILTER_ALPHA_FINAL = 0.003;
    static constexpr double _FILTER_BETA_FINAL = 0.003;
    uint32_t _sequence{};
    double _filter_alpha{};
    double _filter_beta{};

    // Filter settings
    double _filter_alpha_initial{};
    double _filter_beta_initial{};

    // Outlier rejection and filter reset
    //
    // Samples with round-trip time higher than max_rtt_sample are not used to update the filter.
    // More than max_consecutive_high_rtt number of such events in a row will throw a warning
    // but not reset the filter.
    // Samples whose calculated clock offset is more than max_deviation_sample off from the current
    // estimate are not used to update the filter. More than max_consecutive_high_deviation number
    // of such events in a row will reset the filter. This usually happens only due to a time jump
    // on the remote system.
    static constexpr uint64_t _MAX_RTT_SAMPLE_MS = 10;
    static constexpr uint64_t _MAX_CONS_HIGH_RTT = 5;
    uint64_t _high_rtt_count{};

};
} // namespace mavsdk

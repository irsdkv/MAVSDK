#pragma once

#include "mavlink_include.h"
#include "plugins/tune/tune.h"
#include "plugin_impl_base.h"
#include "system.h"

namespace mavsdk {

class TuneImpl : public PluginImplBase {
public:
    TuneImpl(System& system);
    ~TuneImpl();

    void init() override;
    void deinit() override;

    void enable() override;
    void disable() override;

    void play_tune_async(
        const std::vector<Tune::SongElement>& tune,
        const int tempo,
        const Tune::result_callback_t& callback);

    // Non-copyable
    TuneImpl(const TuneImpl&) = delete;
    const TuneImpl& operator=(const TuneImpl&) = delete;

private:
    void timeout_happened();

    void report_tune_result(const Tune::result_callback_t& callback, Tune::Result result);

    void receive_command_result(
        MAVLinkCommands::Result command_result, const Tune::result_callback_t& callback);

    Tune::result_callback_t _result_callback = nullptr;

    std::vector<std::shared_ptr<mavlink_message_t>> _mavlink_tune_item_messages;
};

} // namespace mavsdk

#pragma once

#include "plugins/logging/logging.h"
#include "mavlink_include.h"
#include "plugin_impl_base.h"
#include "system.h"

namespace mavsdk {

class LoggingImpl : public PluginImplBase {
public:
    LoggingImpl(System& system);
    ~LoggingImpl();

    void init() override;
    void deinit() override;

    void enable() override;
    void disable() override;

	Logging::Result set_rate_logging(double rate_hz);
	Logging::Result set_rate_logging_acked(double rate_hz);
	void set_rate_logging_async(double rate_hz, Logging::result_callback_t callback);
	void set_rate_logging_acked_async(double rate_hz, Logging::result_callback_t callback);

    Logging::Result start_logging() const;
    Logging::Result stop_logging() const;

    void start_logging_async(const Logging::result_callback_t& callback);
    void stop_logging_async(const Logging::result_callback_t& callback);

	Logging::MavlinkMessageLoggingData get_mavlink_message_logging_data() const;

	void mavlink_message_logging_data_async(Logging::mavlink_message_logging_data_callback_t& callback);

	void message_flag_bits_async(Logging::message_flag_bits_callback_t& callback);
	void message_format_async(Logging::message_format_callback_t& callback);
	void message_info_async(Logging::message_info_callback_t& callback);
	void message_info_multiple_async(Logging::message_info_multiple_callback_t& callback);
	void message_data_async(Logging::message_data_callback_t& callback);
	void message_dropout_async(Logging::message_dropout_callback_t& callback);
	void message_parameter_async(Logging::message_parameter_callback_t& callback);
	void message_logging_async(Logging::message_logging_callback_t& callback);
	void message_add_logged_async(Logging::message_add_logged_callback_t& callback);
	void message_remove_logged_async(Logging::message_remove_logged_callback_t& callback);

private:

	void set_mavlink_message_logging_data(Logging::MavlinkMessageLoggingData mavlink_message_logging_data);

    void process_logging_data(const mavlink_message_t& message);
    void process_logging_data_acked(const mavlink_message_t& message);
	void process_mavlink_message_logging_data(const Logging::MavlinkMessageLoggingData &message_logging_data);

    static Logging::Result logging_result_from_command_result(MAVLinkCommands::Result result);

    static void command_result_callback(
        MAVLinkCommands::Result command_result, const Logging::result_callback_t& callback);

	Logging::mavlink_message_logging_data_callback_t _mavlink_message_logging_data_subscription{nullptr};

    Logging::message_flag_bits_callback_t _message_flag_bits_subscription{nullptr};
    Logging::message_format_callback_t _message_format_subscription{nullptr};
    Logging::message_data_callback_t _message_data_subscription{nullptr};
    Logging::message_dropout_callback_t _message_dropout_subscription{nullptr};
    Logging::message_info_callback_t _message_info_subscription{nullptr};
    Logging::message_info_multiple_callback_t _message_info_multiple_subscription{nullptr};
    Logging::message_parameter_callback_t _message_parameter_subscription{nullptr};
    Logging::message_logging_callback_t _message_logging_subscription{nullptr};
    Logging::message_add_logged_callback_t _message_add_logged_subscription{nullptr};
	Logging::message_remove_logged_callback_t _message_remove_logged_subscription{nullptr};

	mutable std::mutex _mavlink_message_logging_data_mutex{};
	Logging::MavlinkMessageLoggingData _mavlink_message_logging_data{};
};

} // namespace mavsdk

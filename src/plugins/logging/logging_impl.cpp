#include <algorithm>
#include "global_include.h"
#include "logging_impl.h"
#include "mavsdk_impl.h"
#include "px4_custom_mode.h"

namespace mavsdk {

LoggingImpl::LoggingImpl(System& system) : PluginImplBase(system)
{
    _parent->register_plugin(this);
}

LoggingImpl::~LoggingImpl()
{
    _parent->unregister_plugin(this);
}

void LoggingImpl::init()
{
    using namespace std::placeholders; // for `_1`

    _parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_LOGGING_DATA, std::bind(&LoggingImpl::process_logging_data, this, _1), this);

    _parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_LOGGING_DATA_ACKED,
        std::bind(&LoggingImpl::process_logging_data_acked, this, _1),
        this);
}

void LoggingImpl::deinit()
{
    _parent->unregister_all_mavlink_message_handlers(this);
}

void LoggingImpl::enable() {}

void LoggingImpl::disable() {}

Logging::Result LoggingImpl::start_logging() const
{
    MAVLinkCommands::CommandLong command{};

    command.command = MAV_CMD_LOGGING_START;
    MAVLinkCommands::CommandLong::set_as_reserved(command.params, 0.f);
    command.target_component_id = _parent->get_autopilot_id();

    return logging_result_from_command_result(_parent->send_command(command));
}

Logging::Result LoggingImpl::stop_logging() const
{
    MAVLinkCommands::CommandLong command{};

    command.command = MAV_CMD_LOGGING_STOP;
    MAVLinkCommands::CommandLong::set_as_reserved(command.params, 0.f);
    command.target_component_id = _parent->get_autopilot_id();

    return logging_result_from_command_result(_parent->send_command(command));
}

void LoggingImpl::start_logging_async(const Logging::result_callback_t& callback)
{
    MAVLinkCommands::CommandLong command{};

    command.command = MAV_CMD_LOGGING_START;
    MAVLinkCommands::CommandLong::set_as_reserved(command.params, 0.f);
    command.target_component_id = _parent->get_autopilot_id();

    _parent->send_command_async(
        command, std::bind(&LoggingImpl::command_result_callback, std::placeholders::_1, callback));
}

void LoggingImpl::stop_logging_async(const Logging::result_callback_t& callback)
{
    MAVLinkCommands::CommandLong command{};

    command.command = MAV_CMD_LOGGING_STOP;
    MAVLinkCommands::CommandLong::set_as_reserved(command.params, 0.f);
    command.target_component_id = _parent->get_autopilot_id();

    _parent->send_command_async(
        command, std::bind(&LoggingImpl::command_result_callback, std::placeholders::_1, callback));
}

void LoggingImpl::mavlink_message_logging_data_async(Logging::mavlink_message_logging_data_callback_t& callback)
{
	_mavlink_message_logging_data_subscription = callback;
}

void LoggingImpl::message_flag_bits_async(Logging::message_flag_bits_callback_t& callback)
{
    _message_flag_bits_subscription = callback;
}
void LoggingImpl::message_format_async(Logging::message_format_callback_t& callback)
{
    _message_format_subscription = callback;
}
void LoggingImpl::message_info_async(Logging::message_info_callback_t& callback)
{
    _message_info_subscription = callback;
}
void LoggingImpl::message_info_multiple_async(Logging::message_info_multiple_callback_t& callback)
{
    _message_info_multiple_subscription = callback;
}
void LoggingImpl::message_data_async(Logging::message_data_callback_t& callback)
{
    _message_data_subscription = callback;
}
void LoggingImpl::message_dropout_async(Logging::message_dropout_callback_t& callback)
{
    _message_dropout_subscription = callback;
}
void LoggingImpl::message_parameter_async(Logging::message_parameter_callback_t& callback)
{
    _message_parameter_subscription = callback;
}
void LoggingImpl::message_logging_async(Logging::message_logging_callback_t& callback)
{
    _message_logging_subscription = callback;
}
void LoggingImpl::message_add_logged_async(Logging::message_add_logged_callback_t& callback)
{
    _message_add_logged_subscription = callback;
}
void LoggingImpl::message_remove_logged_async(Logging::message_remove_logged_callback_t& callback)
{
	_message_remove_logged_subscription = callback;
}

Logging::MavlinkMessageLoggingData LoggingImpl::get_mavlink_message_logging_data() const
{
	std::lock_guard<std::mutex> lock(_mavlink_message_logging_data_mutex);
	return _mavlink_message_logging_data;
}

void LoggingImpl::set_mavlink_message_logging_data(Logging::MavlinkMessageLoggingData mavlink_message_logging_data)
{
	std::lock_guard<std::mutex> lock(_mavlink_message_logging_data_mutex);
	_mavlink_message_logging_data = mavlink_message_logging_data;
}

void LoggingImpl::process_mavlink_message_logging_data(const Logging::MavlinkMessageLoggingData &message_logging_data)
{
	set_mavlink_message_logging_data(message_logging_data);

	if (_mavlink_message_logging_data_subscription) {
		auto callback = _mavlink_message_logging_data_subscription;
		auto arg = get_mavlink_message_logging_data();
		_parent->call_user_callback([callback, arg]() { callback(arg); });
	}
}

void LoggingImpl::process_logging_data(const mavlink_message_t& message)
{
	size_t length;
	Logging::MavlinkMessageLoggingData message_logging_data{};

	message_logging_data.first_message_offset = mavlink_msg_logging_data_get_first_message_offset(&message);

	length = static_cast<size_t>(mavlink_msg_logging_data_get_length(&message));
	message_logging_data.data.reserve(MAVLINK_MSG_LOGGING_DATA_FIELD_DATA_LEN);
	mavlink_msg_logging_data_get_data(&message, message_logging_data.data.data());
	message_logging_data.data.erase(message_logging_data.data.begin() + length, message_logging_data.data.end());


	process_mavlink_message_logging_data(message_logging_data);
}


	Logging::Result LoggingImpl::set_rate_logging(double rate_hz)
	{
		return logging_result_from_command_result(
				_parent->set_msg_rate(MAVLINK_MSG_ID_LOGGING_DATA, rate_hz));
	}



	Logging::Result LoggingImpl::set_rate_logging_acked(double rate_hz)
	{
		return logging_result_from_command_result(
				_parent->set_msg_rate(MAVLINK_MSG_ID_LOGGING_DATA_ACKED, rate_hz));

	}

	void LoggingImpl::set_rate_logging_async(double rate_hz, Logging::result_callback_t callback)
	{
		_parent->set_msg_rate_async(
				MAVLINK_MSG_ID_LOGGING_DATA,
				rate_hz,
				std::bind(&LoggingImpl::command_result_callback, std::placeholders::_1, callback));
	}

	void LoggingImpl::set_rate_logging_acked_async(double rate_hz, Logging::result_callback_t callback)
	{
		_parent->set_msg_rate_async(
				MAVLINK_MSG_ID_LOGGING_DATA_ACKED,
				rate_hz,
				std::bind(&LoggingImpl::command_result_callback, std::placeholders::_1, callback));
	}

void LoggingImpl::process_logging_data_acked(const mavlink_message_t& message)
{
	uint16_t sequence;
	size_t length;
	Logging::MavlinkMessageLoggingData message_logging_data{};

    sequence = mavlink_msg_logging_data_acked_get_sequence(&message);

	message_logging_data.first_message_offset = mavlink_msg_logging_data_acked_get_first_message_offset(&message);

	length = static_cast<size_t>(mavlink_msg_logging_data_acked_get_length(&message));
	message_logging_data.data.reserve(MAVLINK_MSG_LOGGING_DATA_ACKED_FIELD_DATA_LEN);
	mavlink_msg_logging_data_acked_get_data(&message, message_logging_data.data.data());
	message_logging_data.data.erase(message_logging_data.data.begin() + length, message_logging_data.data.end());

	process_mavlink_message_logging_data(message_logging_data);

    mavlink_message_t answer;
    mavlink_msg_logging_ack_pack(
        _parent->get_own_system_id(),
        _parent->get_own_component_id(),
        &answer,
        _parent->get_system_id(),
        _parent->get_autopilot_id(),
        sequence);

    _parent->send_message(answer);
}

Logging::Result LoggingImpl::logging_result_from_command_result(MAVLinkCommands::Result result)
{
    switch (result) {
        case MAVLinkCommands::Result::SUCCESS:
            return Logging::Result::SUCCESS;
        case MAVLinkCommands::Result::NO_SYSTEM:
            return Logging::Result::NO_SYSTEM;
        case MAVLinkCommands::Result::CONNECTION_ERROR:
            return Logging::Result::CONNECTION_ERROR;
        case MAVLinkCommands::Result::BUSY:
            return Logging::Result::BUSY;
        case MAVLinkCommands::Result::COMMAND_DENIED:
            return Logging::Result::COMMAND_DENIED;
        case MAVLinkCommands::Result::TIMEOUT:
            return Logging::Result::TIMEOUT;
        default:
            return Logging::Result::UNKNOWN;
    }
}

void LoggingImpl::command_result_callback(
    MAVLinkCommands::Result command_result, const Logging::result_callback_t& callback)
{
    Logging::Result action_result = logging_result_from_command_result(command_result);

    callback(action_result);
}

} // namespace mavsdk

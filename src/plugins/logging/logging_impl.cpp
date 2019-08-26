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

void LoggingImpl::read_message(
    const Logging::MessageHeader& header,
    std::vector<uint8_t>::const_iterator iter_begin,
    std::vector<uint8_t>::const_iterator iter_end)
{
    switch (header.msg_type) {
        case Logging::MessageType::FLAG_BITS: {
            if (_message_flag_bits_subscription) {
                Logging::MessageFlagBits message_flag_bits{};

                std::copy_n(iter_begin, 8, &message_flag_bits.compat_flags[0]);
                std::copy_n(iter_begin + 8, 8, &message_flag_bits.incompat_flags[0]);
                for (int i = 0; i < 3; i++) {
                    message_flag_bits.appended_offsets[i] =
                        uint64_t(iter_begin[16 + 8 * i + 7]) << 56 |
                        uint64_t(iter_begin[16 + 8 * i + 6]) << 48 |
                        uint64_t(iter_begin[16 + 8 * i + 5]) << 40 |
                        uint64_t(iter_begin[16 + 8 * i + 4]) << 32 |
                        uint64_t(iter_begin[16 + 8 * i + 3]) << 24 |
                        uint64_t(iter_begin[16 + 8 * i + 2]) << 16 |
                        uint64_t(iter_begin[16 + 8 * i + 1]) << 8 |
                        uint64_t(iter_begin[16 + 8 * i + 0]) << 0;
                }

                auto callback = _message_flag_bits_subscription;
                auto arg = message_flag_bits;
                _parent->call_user_callback([callback, arg]() { callback(arg); });
            }
            break;
        }
        case Logging::MessageType::FORMAT: {
            if (_message_format_subscription) {
                Logging::MessageFormat message_format{};

                message_format.format.assign(iter_begin, iter_end);

                auto callback = _message_format_subscription;
                auto arg = message_format;
                _parent->call_user_callback([callback, arg]() { callback(arg); });
            }
            break;
        }
        case Logging::MessageType::DATA: {
            if (_message_data_subscription) {
                Logging::MessageData message_data{};

                message_data.msg_id = iter_begin[1] << 8 | iter_begin[0];
                message_data.data.assign(iter_begin + 3, iter_end);

                auto callback = _message_data_subscription;
                auto arg = message_data;
                _parent->call_user_callback([callback, arg]() { callback(arg); });
            }
            break;
        }
        case Logging::MessageType::INFO: {
            if (_message_info_subscription) {
                Logging::MessageInfo message_info{};

                uint8_t key_length = iter_begin[0];
                message_info.key.assign(iter_begin + 1, iter_begin + 1 + key_length);
                message_info.value.assign(iter_begin + 1 + key_length, iter_end);

                auto callback = _message_info_subscription;
                auto arg = message_info;
                _parent->call_user_callback([callback, arg]() { callback(arg); });
            }
            break;
        }
        case Logging::MessageType::INFO_MULTIPLE: {
            if (_message_info_multiple_subscription) {
                Logging::MessageInfoMultiple message_info_multiple{};

                message_info_multiple.is_continued = iter_begin[0];
                uint8_t key_len = iter_begin[1];
                message_info_multiple.key.assign(iter_begin + 1, iter_begin + 1 + key_len);
                message_info_multiple.value.assign(iter_begin + 1 + key_len, iter_end);

                auto callback = _message_info_multiple_subscription;
                auto arg = message_info_multiple;
                _parent->call_user_callback([callback, arg]() { callback(arg); });
            }
            break;
        }
        case Logging::MessageType::PARAMETER: {
            if (_message_parameter_subscription) {
                Logging::MessageParameter message_parameter{};

                uint8_t key_length = iter_begin[0];
                message_parameter.key.assign(iter_begin + 1, iter_begin + 1 + key_length);
                message_parameter.value.assign(iter_begin + 1 + key_length, iter_end);

                auto callback = _message_parameter_subscription;
                auto arg = message_parameter;
                _parent->call_user_callback([callback, arg]() { callback(arg); });
            }
            break;
        }
        case Logging::MessageType::ADD_LOGGED_MSG: {
            if (_message_add_logged_subscription) {
                Logging::MessageAddLogged message_add_logged{};

                message_add_logged.multi_id = iter_begin[0];
                message_add_logged.msg_id = iter_begin[2] << 8 | iter_begin[1];
                message_add_logged.message_name.assign(iter_begin + 3, iter_end);

                auto callback = _message_add_logged_subscription;
                auto arg = message_add_logged;
                _parent->call_user_callback([callback, arg]() { callback(arg); });
            }
            break;
        }
        case Logging::MessageType::DROPOUT: {
            if (_message_dropout_subscription) {
                Logging::MessageDropout message_dropout{};

                message_dropout.duration = iter_begin[1] << 8 | iter_begin[0];

                auto callback = _message_dropout_subscription;
                auto arg = message_dropout;
                _parent->call_user_callback([callback, arg]() { callback(arg); });
            }
            break;
        }
        case Logging::MessageType::LOGGING: {
            if (_message_logging_subscription) {
                Logging::MessageLogging message_logging{};

                message_logging.log_level = Logging::MessageLogging::LogLevel(iter_begin[0]);

                message_logging.timestamp =
                    uint64_t(iter_begin[8]) << 56 | uint64_t(iter_begin[7]) << 48 |
                    uint64_t(iter_begin[6]) << 40 | uint64_t(iter_begin[5]) << 32 |
                    uint64_t(iter_begin[4]) << 24 | uint64_t(iter_begin[3]) << 16 |
                    uint64_t(iter_begin[2]) << 8 | uint64_t(iter_begin[1]) << 0;
                message_logging.data.assign(iter_begin + 9, iter_end);

                auto callback = _message_logging_subscription;
                auto arg = message_logging;
                _parent->call_user_callback([callback, arg]() { callback(arg); });
            }
            break;
        }
        case Logging::MessageType::REMOVE_LOGGED_MSG: {
            // Not implemented
            break;
        }
        case Logging::MessageType::SYNC: {
            // Not implemented
            break;
        }
    }
}

LoggingImpl::ProcessMessageResult LoggingImpl::process_message(
    ProcessMessageResult curr_state,
    const std::vector<uint8_t>& message,
    std::vector<uint8_t>::const_iterator& next_first,
    std::vector<uint8_t>::const_iterator expected_first)
{
    switch (curr_state) {
        case ProcessMessageResult::COMPLETE: {
            _current_header.msg_size = (message[1] << 8 | message[0]);
            _current_header.msg_type = static_cast<Logging::MessageType>(message[2]);
            return ProcessMessageResult::IN_PROGRESS;
        }
        case ProcessMessageResult::IN_PROGRESS: {
            if (message.size() < static_cast<size_t>(_current_header.msg_size + 3)) {
	            next_first = expected_first;

                if (expected_first == message.end()) {
                	return ProcessMessageResult::IN_PROGRESS;
                } else {
	                return ProcessMessageResult::ERROR;
                }
            }else {

	            auto message_data_begin = message.begin() + 3;
	            next_first = message_data_begin + _current_header.msg_size;

	            //read_message(_current_header, message_data_begin, next_first);

	            return ProcessMessageResult::COMPLETE;
            }
        }
        case ProcessMessageResult::ERROR:
        default: {
            return ProcessMessageResult::ERROR;
        }
    }
}

void LoggingImpl::process_message_data_chunk(
    const std::vector<uint8_t>& data_chunk, uint8_t first_message_offset)
{

	if (_message_logging_subscription) {
		Logging::MessageLogging message_logging{};

		message_logging.log_level = Logging::MessageLogging::LogLevel::DEBUG;
		message_logging.timestamp = first_message_offset;

		std::ostringstream os;
		os << "NEW MESSAGE" << '\0';
		std::string s = os.str();
		message_logging.data.assign(s.begin(), s.end());

		auto callback = _message_logging_subscription;
		auto arg = message_logging;
		_parent->call_user_callback([callback, arg]() { callback(arg); });
	}

	int offs;

    if (first_message_offset == 255) {
	    offs = 0;
    } else {
	    offs = data_chunk.size() - first_message_offset;
    }

	if (!_header_received) {
		_current_logging_data.assign(data_chunk.begin() + 16, data_chunk.end());
		offs-= 16;
		_header_received = true;
	} else {
		_current_logging_data.assign(data_chunk.begin(), data_chunk.end());
	}

    do {
	    switch (_current_state) {
		    case ProcessMessageResult::COMPLETE: {

			    if (_message_logging_subscription) {
				    Logging::MessageLogging message_logging{};

				    message_logging.log_level = Logging::MessageLogging::LogLevel::DEBUG;
				    message_logging.timestamp = first_message_offset;

				    std::ostringstream os;
				    os << "COMPLETE: " << "all.size = " << _current_logging_data.size() << ", fmo = " << unsigned(first_message_offset) << '\0';
				    std::string s = os.str();
				    message_logging.data.assign(s.begin(), s.end());

				    auto callback = _message_logging_subscription;
				    auto arg = message_logging;
				    _parent->call_user_callback([callback, arg]() { callback(arg); });
			    }
			    if (_current_logging_data.size() < 3){
				    _curr_offs = 0;
			    	break;
			    }
			    _current_header.msg_size = (_current_logging_data[1] << 8 | _current_logging_data[0]);
			    _current_header.msg_type = static_cast<Logging::MessageType>(_current_logging_data[2]);
			    _curr_offs = _current_logging_data.size() - 3;
			    _current_state = ProcessMessageResult::IN_PROGRESS;
			    offs = 0;
			    break;
		    }
		    case ProcessMessageResult::IN_PROGRESS: {

			    if (_message_logging_subscription) {
				    Logging::MessageLogging message_logging{};

				    message_logging.log_level = Logging::MessageLogging::LogLevel::DEBUG;

				    message_logging.timestamp = first_message_offset;
				    std::ostringstream os;
				    os << "IN_PROGRESS: " << "curr_msg_size = " << unsigned(_current_header.msg_size) << ", all.size = " << _current_logging_data.size() << ", fmo = " << unsigned(first_message_offset) << '\0';
				    std::string s = os.str();
				    message_logging.data.assign(s.begin(), s.end());

				    auto callback = _message_logging_subscription;
				    auto arg = message_logging;
				    _parent->call_user_callback([callback, arg]() { callback(arg); });
			    }

			    if (_current_logging_data.size() < static_cast<size_t>(_current_header.msg_size + 3)) {
				    if (offs != 0) {
					    _current_state = ProcessMessageResult::ERROR;
					    break;
				    }
				    _curr_offs = 0;
				    break;
			    }

			    auto message_data_begin = _current_logging_data.begin() + 3;
			    auto message_data_end = message_data_begin + _current_header.msg_size;

			    read_message(_current_header, message_data_begin, message_data_end);

			    _current_logging_data.erase(_current_logging_data.begin(), message_data_end);
			    _curr_offs = _current_logging_data.size();
			    _current_state = ProcessMessageResult::COMPLETE;
			    break;
		    }
		    case ProcessMessageResult::ERROR:
		    default: {

			    if (_message_logging_subscription) {
				    Logging::MessageLogging message_logging{};

				    message_logging.log_level = Logging::MessageLogging::LogLevel::DEBUG;

				    message_logging.timestamp = first_message_offset;
				    std::ostringstream os;
				    os << "ERROR: " << "all.size = " << _current_logging_data.size() << ", fmo = " << unsigned(first_message_offset) << '\0';
				    std::string s = os.str();
				    message_logging.data.assign(s.begin(), s.end());

				    auto callback = _message_logging_subscription;
				    auto arg = message_logging;
				    _parent->call_user_callback([callback, arg]() { callback(arg); });
			    }

			    _current_logging_data.erase(_current_logging_data.begin(), _current_logging_data.end() - offs);
			    _curr_offs = 0;
			    if (offs != 0) {
				    _current_state = ProcessMessageResult::COMPLETE;
			    }
			    offs = 0;
			    break;
		    }
	    }
    } while (_curr_offs != 0);
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

void LoggingImpl::process_logging_data(const mavlink_message_t& message)
{
    const size_t length = static_cast<size_t>(mavlink_msg_logging_data_get_length(&message));
    uint8_t first_message_offset;
    std::array<uint8_t, MAVLINK_MSG_LOGGING_DATA_FIELD_DATA_LEN> data{};

    first_message_offset = mavlink_msg_logging_data_get_first_message_offset(&message);
    mavlink_msg_logging_data_get_data(&message, data.data());
    std::vector<uint8_t> chunk(data.begin(), data.begin() + length);
    process_message_data_chunk(chunk, first_message_offset);
}

void LoggingImpl::process_logging_data_acked(const mavlink_message_t& message)
{
    uint16_t sequence;
    uint8_t length;
    uint8_t first_message_offset;
    std::array<uint8_t, MAVLINK_MSG_LOGGING_DATA_ACKED_FIELD_DATA_LEN> data{};

    sequence = mavlink_msg_logging_data_acked_get_sequence(&message);
    length = mavlink_msg_logging_data_acked_get_length(&message);
    first_message_offset = mavlink_msg_logging_data_acked_get_first_message_offset(&message);
    mavlink_msg_logging_data_acked_get_data(&message, data.data());

    mavlink_message_t answer;
    mavlink_msg_logging_ack_pack(
        _parent->get_own_system_id(),
        _parent->get_own_component_id(),
        &answer,
        _parent->get_system_id(),
        _parent->get_autopilot_id(),
        sequence);

    _parent->send_message(answer);

    std::vector<uint8_t> chunk(data.begin(), data.begin() + length);
    process_message_data_chunk(chunk, first_message_offset);
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

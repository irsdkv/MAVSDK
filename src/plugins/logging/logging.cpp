#include "plugins/logging/logging.h"
#include "logging_impl.h"

namespace mavsdk {

Logging::Logging(System& system) : PluginBase(), _impl{new LoggingImpl(system)} {}

Logging::~Logging() {}

Logging::Result Logging::start_logging() const
{
    return _impl->start_logging();
}

Logging::Result Logging::stop_logging() const
{
    return _impl->stop_logging();
}

void Logging::start_logging_async(result_callback_t callback)
{
    _impl->start_logging_async(callback);
}

void Logging::stop_logging_async(result_callback_t callback)
{
    _impl->stop_logging_async(callback);
}

Logging::Result Logging::set_rate_logging(double rate_hz)
{
	return _impl->set_rate_logging(rate_hz);
}
Logging::Result Logging::set_rate_logging_acked(double rate_hz)
{
	return _impl->set_rate_logging_acked(rate_hz);
}
	void Logging::set_rate_logging_async(double rate_hz, Logging::result_callback_t callback)
{
	_impl->set_rate_logging_async(rate_hz, callback);
}
	void Logging::set_rate_logging_acked_async(double rate_hz, Logging::result_callback_t callback)
{
	_impl->set_rate_logging_acked_async(rate_hz, callback);
}

const char* Logging::result_str(Result result)
{
    switch (result) {
        case Result::SUCCESS:
            return "Success";
        case Result::NO_SYSTEM:
            return "No system";
        case Result::CONNECTION_ERROR:
            return "Connection error";
        case Result::BUSY:
            return "Busy";
        case Result::COMMAND_DENIED:
            return "Command denied";
        case Result::TIMEOUT:
            return "Timeout";
        case Result::UNKNOWN:
        default:
            return "Unknown";
    }
}

void Logging::mavlink_message_logging_data_async(mavlink_message_logging_data_callback_t callback)
{
	return _impl->mavlink_message_logging_data_async(callback);
}

void Logging::message_flag_bits_async(message_flag_bits_callback_t callback)
{
    return _impl->message_flag_bits_async(callback);
}
void Logging::message_format_async(message_format_callback_t callback)
{
    return _impl->message_format_async(callback);
}
void Logging::message_info_async(message_info_callback_t callback)
{
    return _impl->message_info_async(callback);
}
void Logging::message_info_multiple_async(message_info_multiple_callback_t callback)
{
    return _impl->message_info_multiple_async(callback);
}
void Logging::message_data_async(message_data_callback_t callback)
{
    return _impl->message_data_async(callback);
}
void Logging::message_dropout_async(message_dropout_callback_t callback)
{
    return _impl->message_dropout_async(callback);
}
void Logging::message_parameter_async(message_parameter_callback_t callback)
{
    return _impl->message_parameter_async(callback);
}
void Logging::message_logging_async(message_logging_callback_t callback)
{
    return _impl->message_logging_async(callback);
}
void Logging::message_add_logged_async(message_add_logged_callback_t callback)
{
    return _impl->message_add_logged_async(callback);
}
void Logging::message_remove_logged_async(message_remove_logged_callback_t callback)
{
    return _impl->message_remove_logged_async(callback);
}

} // namespace mavsdk

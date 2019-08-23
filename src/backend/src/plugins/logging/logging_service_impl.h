#include <future>

#include "plugins/logging/logging.h"
#include "logging/logging.grpc.pb.h"

namespace mavsdk {
	namespace backend {


		template<typename Logging = Logging>
		class LoggingServiceImpl final : public mavsdk::rpc::logging::LoggingService::Service {
		public:
			LoggingServiceImpl(Logging& logging) :
					_logging(logging),
					_stop_promise(std::promise<void>()),
					_stop_future(_stop_promise.get_future())
			{}

			template<typename ResponseType>
			void fillResponseWithResult(ResponseType* response, mavsdk::Logging::Result& logging_result) const
			{
				auto rpc_result = static_cast<rpc::logging::LoggingResult::Result>(logging_result);

				auto* rpc_logging_result = new rpc::logging::LoggingResult();
				rpc_logging_result->set_result(rpc_result);
				rpc_logging_result->set_result_str(mavsdk::Logging::result_str(logging_result));

				response->set_allocated_logging_result(rpc_logging_result);
			}

			grpc::Status StartLogging(
					grpc::ServerContext* /* context */,
					const rpc::logging::StartLoggingRequest* /* request */,
					rpc::logging::StartLoggingResponse* response) override
			{
				auto logging_result = _logging.start_logging();

				if (response != nullptr) {
					fillResponseWithResult(response, logging_result);
				}

				return grpc::Status::OK;
			}

			grpc::Status StopLogging(
					grpc::ServerContext* /* context */,
					const rpc::logging::StopLoggingRequest* /* request */,
					rpc::logging::StopLoggingResponse* response) override
			{
				auto logging_result = _logging.stop_logging();

				if (response != nullptr) {
					fillResponseWithResult(response, logging_result);
				}

				return grpc::Status::OK;
			}

			grpc::Status SubscribeMessageFlagBits(
					grpc::ServerContext* /* context */,
					const mavsdk::rpc::logging::SubscribeMessageFlagBitsRequest* /* request */,
					grpc::ServerWriter<rpc::logging::MessageFlagBitsResponse>* writer) override
			{
				std::mutex message_flag_bits_mutex{};

				_logging.message_flag_bits_async([&writer, &message_flag_bits_mutex](mavsdk::Logging::MessageFlagBits message_flag_bits) {
					auto rpc_message_flag_bits = new mavsdk::rpc::logging::MessageFlagBits();
					for (int i = 0; i < 8; i++) {
						rpc_message_flag_bits->add_compat_flags(message_flag_bits.compat_flags[i]);
					}
					for (int i = 0; i < 8; i++) {
						rpc_message_flag_bits->add_incompat_flags(message_flag_bits.incompat_flags[i]);
					}
					for (int i = 0; i < 3; i++) {
						rpc_message_flag_bits->add_appended_offsets(message_flag_bits.appended_offsets[i]);
					}


					mavsdk::rpc::logging::MessageFlagBitsResponse rpc_message_flag_bits_response;
					rpc_message_flag_bits_response.set_allocated_message_flag_bits(rpc_message_flag_bits);

					std::lock_guard<std::mutex> lock(message_flag_bits_mutex);
					writer->Write(rpc_message_flag_bits_response);
				});

				_stop_future.wait();
				return grpc::Status::OK;
			}

			grpc::Status SubscribeMessageFormat(
					grpc::ServerContext* /* context */,
					const mavsdk::rpc::logging::SubscribeMessageFormatRequest* /* request */,
					grpc::ServerWriter<rpc::logging::MessageFormatResponse>* writer) override
			{
				std::mutex message_format_mutex{};

				_logging.message_format_async([&writer, &message_format_mutex](mavsdk::Logging::MessageFormat message_format) {
					auto rpc_message_format = new mavsdk::rpc::logging::MessageFormat();
					rpc_message_format->set_format(message_format.format.data());

					mavsdk::rpc::logging::MessageFormatResponse rpc_message_format_response;
					rpc_message_format_response.set_allocated_message_format(rpc_message_format);

					std::lock_guard<std::mutex> lock(message_format_mutex);
					writer->Write(rpc_message_format_response);
				});

				_stop_future.wait();
				return grpc::Status::OK;
			}

			grpc::Status SubscribeMessageInfo(
					grpc::ServerContext* /* context */,
					const mavsdk::rpc::logging::SubscribeMessageInfoRequest* /* request */,
					grpc::ServerWriter<rpc::logging::MessageInfoResponse>* writer) override
			{
				std::mutex message_info_mutex{};

				_logging.message_info_async([&writer, &message_info_mutex](mavsdk::Logging::MessageInfo message_info) {
					auto rpc_message_info = new mavsdk::rpc::logging::MessageInfo();
					rpc_message_info->set_key(message_info.key.data());
					rpc_message_info->set_value(message_info.value.data());

					mavsdk::rpc::logging::MessageInfoResponse rpc_message_info_response;
					rpc_message_info_response.set_allocated_message_info(rpc_message_info);

					std::lock_guard<std::mutex> lock(message_info_mutex);
					writer->Write(rpc_message_info_response);
				});

				_stop_future.wait();
				return grpc::Status::OK;
			}

			grpc::Status SubscribeMessageInfoMultiple(
					grpc::ServerContext* /* context */,
					const mavsdk::rpc::logging::SubscribeMessageInfoMultipleRequest* /* request */,
					grpc::ServerWriter<rpc::logging::MessageInfoMultipleResponse>* writer) override
			{
				std::mutex message_info_multiple_mutex{};

				_logging.message_info_multiple_async([&writer, &message_info_multiple_mutex](mavsdk::Logging::MessageInfoMultiple message_info_multiple) {
					auto rpc_message_info_multiple = new mavsdk::rpc::logging::MessageInfoMultiple();
					rpc_message_info_multiple->set_is_continued(message_info_multiple.is_continued);
					rpc_message_info_multiple->set_key(message_info_multiple.key.data());
					rpc_message_info_multiple->set_value(message_info_multiple.value.data());

					mavsdk::rpc::logging::MessageInfoMultipleResponse rpc_message_info_multiple_response;
					rpc_message_info_multiple_response.set_allocated_message_info_multiple(rpc_message_info_multiple);

					std::lock_guard<std::mutex> lock(message_info_multiple_mutex);
					writer->Write(rpc_message_info_multiple_response);
				});

				_stop_future.wait();
				return grpc::Status::OK;
			}

			grpc::Status SubscribeMessageParameter(
					grpc::ServerContext* /* context */,
					const mavsdk::rpc::logging::SubscribeMessageParameterRequest* /* request */,
					grpc::ServerWriter<rpc::logging::MessageParameterResponse>* writer) override
			{
				std::mutex message_parameter_mutex{};

				_logging.message_parameter_async([&writer, &message_parameter_mutex](mavsdk::Logging::MessageParameter message_parameter) {
					auto rpc_message_parameter = new mavsdk::rpc::logging::MessageParameter();
					rpc_message_parameter->set_key(message_parameter.key.data());
					rpc_message_parameter->set_value(message_parameter.value.data());

					mavsdk::rpc::logging::MessageParameterResponse rpc_message_parameter_response;
					rpc_message_parameter_response.set_allocated_message_parameter(rpc_message_parameter);

					std::lock_guard<std::mutex> lock(message_parameter_mutex);
					writer->Write(rpc_message_parameter_response);
				});

				_stop_future.wait();
				return grpc::Status::OK;
			}

			grpc::Status SubscribeMessageAddLogged(
					grpc::ServerContext* /* context */,
					const mavsdk::rpc::logging::SubscribeMessageAddLoggedRequest* /* request */,
					grpc::ServerWriter<rpc::logging::MessageAddLoggedResponse>* writer) override
			{
				std::mutex message_add_logged_mutex{};

				_logging.message_add_logged_async([&writer, &message_add_logged_mutex](mavsdk::Logging::MessageAddLogged message_add_logged) {
					auto rpc_message_add_logged = new mavsdk::rpc::logging::MessageAddLogged();
					rpc_message_add_logged->set_multi_id(message_add_logged.multi_id);
					rpc_message_add_logged->set_msg_id(message_add_logged.msg_id);
					rpc_message_add_logged->set_message_name(message_add_logged.message_name.data());

					mavsdk::rpc::logging::MessageAddLoggedResponse rpc_message_add_logged_response;
					rpc_message_add_logged_response.set_allocated_message_add_logged(rpc_message_add_logged);

					std::lock_guard<std::mutex> lock(message_add_logged_mutex);
					writer->Write(rpc_message_add_logged_response);
				});

				_stop_future.wait();
				return grpc::Status::OK;
			}

			grpc::Status SubscribeMessageRemoveLogged(
					grpc::ServerContext* /* context */,
					const mavsdk::rpc::logging::SubscribeMessageRemoveLoggedRequest* /* request */,
					grpc::ServerWriter<rpc::logging::MessageRemoveLoggedResponse>* writer) override
			{
				std::mutex message_remove_logged_mutex{};

				_logging.message_remove_logged_async([&writer, &message_remove_logged_mutex](mavsdk::Logging::MessageRemoveLogged message_remove_logged) {
					auto rpc_message_remove_logged = new mavsdk::rpc::logging::MessageRemoveLogged();
					rpc_message_remove_logged->set_msg_id(message_remove_logged.msg_id);

					mavsdk::rpc::logging::MessageRemoveLoggedResponse rpc_message_remove_logged_response;
					rpc_message_remove_logged_response.set_allocated_message_remove_logged(rpc_message_remove_logged);

					std::lock_guard<std::mutex> lock(message_remove_logged_mutex);
					writer->Write(rpc_message_remove_logged_response);
				});

				_stop_future.wait();
				return grpc::Status::OK;
			}

			grpc::Status SubscribeMessageData(
					grpc::ServerContext* /* context */,
					const mavsdk::rpc::logging::SubscribeMessageDataRequest* /* request */,
					grpc::ServerWriter<rpc::logging::MessageDataResponse>* writer) override
			{
				std::mutex message_data_mutex{};

				_logging.message_data_async([&writer, &message_data_mutex](mavsdk::Logging::MessageData message_data) {
					auto rpc_message_data = new mavsdk::rpc::logging::MessageData();
					rpc_message_data->set_msg_id(message_data.msg_id);
					rpc_message_data->set_data(message_data.data.data(), message_data.data.size());

					mavsdk::rpc::logging::MessageDataResponse rpc_message_data_response;
					rpc_message_data_response.set_allocated_message_data(rpc_message_data);

					std::lock_guard<std::mutex> lock(message_data_mutex);
					writer->Write(rpc_message_data_response);
				});

				_stop_future.wait();
				return grpc::Status::OK;
			}


			mavsdk::rpc::logging::MessageLogging::LogLevel
			translateLogLevel(const mavsdk::Logging::MessageLogging::LogLevel level) const
			{
				switch (level) {
					default:
					case mavsdk::Logging::MessageLogging::LogLevel::UNDEF:
						return mavsdk::rpc::logging::MessageLogging::LogLevel::MessageLogging_LogLevel_UNDEF;
					case mavsdk::Logging::MessageLogging::LogLevel::EMERG:
						return mavsdk::rpc::logging::MessageLogging::LogLevel::MessageLogging_LogLevel_EMERG;
					case mavsdk::Logging::MessageLogging::LogLevel::ALERT:
						return mavsdk::rpc::logging::MessageLogging::LogLevel::MessageLogging_LogLevel_ALERT;
					case mavsdk::Logging::MessageLogging::LogLevel::CRIT:
						return mavsdk::rpc::logging::MessageLogging::LogLevel::MessageLogging_LogLevel_CRIT;
					case mavsdk::Logging::MessageLogging::LogLevel::ERR:
						return mavsdk::rpc::logging::MessageLogging::LogLevel::MessageLogging_LogLevel_ERR;
					case mavsdk::Logging::MessageLogging::LogLevel::WARNING:
						return mavsdk::rpc::logging::MessageLogging::LogLevel::MessageLogging_LogLevel_WARNING;
					case mavsdk::Logging::MessageLogging::LogLevel::NOTICE:
						return mavsdk::rpc::logging::MessageLogging::LogLevel::MessageLogging_LogLevel_NOTICE;
					case mavsdk::Logging::MessageLogging::LogLevel::INFO:
						return mavsdk::rpc::logging::MessageLogging::LogLevel::MessageLogging_LogLevel_INFO;
					case mavsdk::Logging::MessageLogging::LogLevel::DEBUG:
						return mavsdk::rpc::logging::MessageLogging::LogLevel::MessageLogging_LogLevel_DEBUG;
				}
			}

			grpc::Status SubscribeMessageLogging(
					grpc::ServerContext* /* context */,
					const mavsdk::rpc::logging::SubscribeMessageLoggingRequest* /* request */,
					grpc::ServerWriter<rpc::logging::MessageLoggingResponse>* writer) override
			{
				std::mutex message_logging_mutex{};

				_logging.message_logging_async([this, &writer, &message_logging_mutex](mavsdk::Logging::MessageLogging message_logging) {
					auto rpc_message_logging = new mavsdk::rpc::logging::MessageLogging();
					rpc_message_logging->set_log_level(translateLogLevel(message_logging.log_level));
					rpc_message_logging->set_timestamp(message_logging.timestamp);
					rpc_message_logging->set_data(message_logging.data.data());

					mavsdk::rpc::logging::MessageLoggingResponse rpc_message_logging_response;
					rpc_message_logging_response.set_allocated_message_logging(rpc_message_logging);

					std::lock_guard<std::mutex> lock(message_logging_mutex);
					writer->Write(rpc_message_logging_response);
				});

				_stop_future.wait();
				return grpc::Status::OK;
			}

			grpc::Status SubscribeMessageDropout(
					grpc::ServerContext* /* context */,
					const mavsdk::rpc::logging::SubscribeMessageDropoutRequest* /* request */,
					grpc::ServerWriter<rpc::logging::MessageDropoutResponse>* writer) override
			{
				std::mutex message_dropout_mutex{};

				_logging.message_dropout_async([&writer, &message_dropout_mutex](mavsdk::Logging::MessageDropout message_dropout) {
					auto rpc_message_dropout = new mavsdk::rpc::logging::MessageDropout();
					rpc_message_dropout->set_duration(message_dropout.duration);

					mavsdk::rpc::logging::MessageDropoutResponse rpc_message_dropout_response;
					rpc_message_dropout_response.set_allocated_message_dropout(rpc_message_dropout);

					std::lock_guard<std::mutex> lock(message_dropout_mutex);
					writer->Write(rpc_message_dropout_response);
				});

				_stop_future.wait();
				return grpc::Status::OK;
			}

			void stop() { _stop_promise.set_value(); }

		private:
			Logging& _logging;
			std::promise<void> _stop_promise;
			std::future<void> _stop_future;
		};

	} // namespace backend
} // namespace mavsdk

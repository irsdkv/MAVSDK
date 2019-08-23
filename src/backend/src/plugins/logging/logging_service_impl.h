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

			grpc::Status SubscribeLoggingRaw(
					grpc::ServerContext* /* context */,
					const mavsdk::rpc::logging::SubscribeLoggingRawRequest* /* request */,
					grpc::ServerWriter<rpc::logging::LoggingRawResponse>* writer) override
			{
				std::mutex mavlink_message_logging_mutex{};

				_logging.mavlink_message_logging_data_async([&writer, &mavlink_message_logging_mutex](mavsdk::Logging::MavlinkMessageLoggingData message_logging_data) {
					auto rpc_logging_raw = new mavsdk::rpc::logging::LoggingRaw();
					rpc_logging_raw->set_first_message_offset(message_logging_data.first_message_offset);
					rpc_logging_raw->set_data(message_logging_data.data.data(), message_logging_data.data.size());


					mavsdk::rpc::logging::LoggingRawResponse rpc_logging_raw_response;
					rpc_logging_raw_response.set_allocated_logging_raw(rpc_logging_raw);

					std::lock_guard<std::mutex> lock(mavlink_message_logging_mutex);
					writer->Write(rpc_logging_raw_response);
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

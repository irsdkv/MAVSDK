#pragma once

#include <functional>
#include <memory>
#include <vector>

#include <plugin_base.h>

namespace mavsdk {

class LoggingImpl;
class System;

/**
 * @brief The Logging class allows log data using logger and log streaming from the vehicle.
 */
class Logging : public PluginBase {
public:
    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto logging = std::make_shared<Logging>(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit Logging(System& system);

    /**
     * @brief Destructor (internal use only).
     */
    ~Logging();

    /**
     * @brief Results for logging requests.
     */
    enum class Result {
        SUCCESS = 0, /**< @brief %Request succeeded. */
        NO_SYSTEM, /**< @brief No system connected. */
        CONNECTION_ERROR, /**< @brief %Connection error. */
        BUSY, /**< @brief %System busy. */
        COMMAND_DENIED, /**< @brief Command denied. */
        TIMEOUT, /**< @brief Timeout. */
        UNKNOWN /**< @brief Unknown error. */
    };

	/**
	 * @brief MAVLink Logging Data Message type.
	 *
	 * A message containing logged data.
	 */
    struct MavlinkMessageLoggingData {
    	uint8_t first_message_offset; /**< @brief Offset into data where first message starts (255 if no start exists). */
    	std::vector<uint8_t> data; /**< @brief Logged data. */
    };

	/**
	 * @brief Callback type for MAVLink Logging Data messages.
	 *
	 * @param mavlink_message_logging_data MAVLink Logging Data message.
	 */
	typedef std::function<void(MavlinkMessageLoggingData mavlink_message_logging_data)> mavlink_message_logging_data_callback_t;

	/**
	 * @brief Subscribe to MAVLink Logging Data messages (asynchronous).
	 *
	 * @param callback Function to call with updates.
	 */
	void mavlink_message_logging_data_async(mavlink_message_logging_data_callback_t callback);


	/**
	 * @brief Log file Header type.
	 *
	 * The header with which log file begin.
	 */
	struct LogFileHeader {
		std::array<uint8_t, 7> file_magic; /**< @brief File magic. */
		uint8_t version; /**< @brief Version of protocol. */
		uint64_t timestamp; /**< @brief Timestamp. */
	};

    enum class MessageType : uint8_t {
        FORMAT = 'F',
        DATA = 'D',
        INFO = 'I',
        INFO_MULTIPLE = 'M',
        PARAMETER = 'P',
        ADD_LOGGED_MSG = 'A',
        REMOVE_LOGGED_MSG = 'R',
        SYNC = 'S',
        DROPOUT = 'O',
        LOGGING = 'L',
        FLAG_BITS = 'B'
    };

    /**
     * @brief Message header type.
     *
     * The header with which each message begins.
     */
    struct MessageHeader {
        uint16_t msg_size; /**< @brief Size of the message in bytes without the header (header size
                              is 3 bytes). */
        MessageType msg_type; /**< @brief Defines the content (see below). */
    };

    /**
     * @brief Flag bitset message type. Definitions Section.
     */
    struct MessageFlagBits {
        std::array<uint8_t, 8> compat_flags; /**< @brief Compatible flag bits. */
	    std::array<uint8_t, 8> incompat_flags; /**< @brief Incompatible flag bits. */
	    std::array<uint64_t, 3> appended_offsets; /**< @brief File offsets (0-based) for appended data. If no
                                         data is appended, all offsets must be zero. */
    };

    /**
     * @brief Format definition message type.
     *
     * Format definition for a single (composite) type that can be logged or used in another
     * definition as a nested type.
     */
    struct MessageFormat {
	    MessageHeader header; /**< @brief Header of the message */
        std::vector<char>
            format; /**< @brief  plain-text string with the following format:
                     * 'message_name:field0;field1;'.
                     * There can be an arbitrary amount of fields (at least 1), separated by ';' */
    };

    /**
     * @brief Information message type. Definitions Section.
     */
    struct MessageInfo {
	    MessageHeader header; /**< @brief Header of the message */
        std::vector<char> key; /**< @brief Plain string key name */
        std::vector<char> value; /**< @brief the data as described by key */
    };

    /**
     * @brief Information multiple message type. Definitions Section.
     */
    struct MessageInfoMultiple {
	    MessageHeader header; /**< @brief Header of the message */
        bool is_continued; /**< @brief Is this is continued message */
        std::vector<char> key; /**< @brief Plain string key name */
        std::vector<char> value; /**< @brief the data as described by key */
    };

	/**
	 * @brief Parameter change message type. Definitions Section.
	 *
	 * If a parameter dynamically changes during runtime, this message can also be used in the Data
	 * section.
	 */
	struct MessageParameter {
		MessageHeader header; /**< @brief Header of the message */
		std::vector<char> key; /**< @brief Plain string key name */
		std::vector<char> value; /**< @brief the data as described by key */
	};

	/**
	 * @brief TODO
	 */
	struct MessageAddLogged {
		MessageHeader header; /**< @brief Header of the message */
		uint8_t multi_id; /**< @brief TODO */
		uint16_t msg_id; /**< @brief Unique id to match MessageData data. */
		std::vector<char> message_name; /**< @brief Message name to subscribe to. */
	};

	/**
	 * @brief TODO
	 */
	struct MessageRemoveLogged {
		MessageHeader header; /**< @brief Header of the message */
		uint16_t msg_id; /**< @brief TODO */
	};

	/**
	 * @brief TODO
	 */
	struct MessageData {
		MessageHeader header; /**< @brief Header of the message */
		uint16_t msg_id; /**< @brief TODO */
		std::vector<uint8_t> data; /**< @brief The logged binary message as defined by TODO */
	};

	/**
	 * @brief TODO
	 */
	struct MessageLogging {
		enum class LogLevel {
			EMERG = '0', /**< @brief System is unusable. */
			ALERT = '1', /**< @brief Action must be taken immediately. */
			CRIT = '2', /**< @brief Critical conditions. */
			ERR = '3', /**< @brief Error conditions. */
			WARNING = '4', /**< @brief Warning conditions. */
			NOTICE = '5', /**< @brief Normal but significant condition. */
			INFO = '6', /**< @brief Informational. */
			DEBUG = '7' /**< @brief Debug-level messages. */
		};

		MessageHeader header; /**< @brief Header of the message */
		LogLevel log_level; /**< @brief TODO */
		uint64_t timestamp; /**< @brief TODO */
		std::vector<char> data; /**< @brief TODO */
	};

	/**
	 * @brief Synchronization message so that a reader can recover from a corrupt message by searching for the
	 * next sync message (not used currently).
	 */
	struct MessageSync {
		MessageHeader header; /**< @brief Header of the message */
		std::array<uint8_t, 8> sync_magic; /**< @brief To be defined */
	};

	/**
	 * @brief TODO
	 */
	struct MessageDropout {
		MessageHeader header; /**< @brief Header of the message */
		uint16_t duration; /**< @brief Duration in ms. */
	};

    /**
     * @brief Returns human-readable English string for Logging::Result.
     *
     * @param result Enum for which string is required.
     * @return result Human-readable string for Logging::Result.
     */
    static const char* result_str(Result result);

    /**
     * @brief Callback type for logging requests.
     */
    typedef std::function<void(Result)> result_callback_t;

    /**
     * @brief Start logging (synchronous).
     *
     * @return Result of request.
     */
    Result start_logging() const;

    /**
     * @brief Stop logging (synchronous).
     *
     * @return Result of request.
     */
    Result stop_logging() const;

	Result set_rate_logging(double rate_hz);
	Result set_rate_logging_acked(double rate_hz);
	void set_rate_logging_async(double rate_hz, result_callback_t callback);
	void set_rate_logging_acked_async(double rate_hz, result_callback_t callback);

    /**
     * @brief Start logging (asynchronous).
     *
     * @param callback Callback to get result of request.
     */
    void start_logging_async(result_callback_t callback);

    /**
     * @brief Stop logging (asynchronous).
     *
     * @param callback Callback to get result of request.
     */
    void stop_logging_async(result_callback_t callback);

	/**
	 * @brief Callback type for Flag Bits messages.
	 *
	 * @param message_flag_bits Flag bits message.
	 */
	typedef std::function<void(MessageFlagBits message_flag_bits)> message_flag_bits_callback_t;

	/**
	 * @brief Callback type for Format messages.
	 *
	 * @param message_format Format message.
	 */
	typedef std::function<void(MessageFormat message_format)> message_format_callback_t;

	/**
	 * @brief Callback type for Info messages.
	 *
	 * @param message_info Info message.
	 */
	typedef std::function<void(MessageInfo message_info)> message_info_callback_t;

	/**
	 * @brief Callback type for Info Multiple messages.
	 *
	 * @param message_info_multiple Info Multiple message.
	 */
	typedef std::function<void(MessageInfoMultiple message_info_multiple)> message_info_multiple_callback_t;

	/**
	 * @brief Callback type for Parameter messages.
	 *
	 * @param message_parameter Parameter message.
	 */
	typedef std::function<void(MessageParameter message_parameter)> message_parameter_callback_t;

	/**
	 * @brief Callback type for Data messages.
	 *
	 * @param message_data Data message.
	 */
	typedef std::function<void(MessageData message_data)> message_data_callback_t;

	/**
	 * @brief Callback type for Logging messages.
	 *
	 * @param message_logging Logging message.
	 */
	typedef std::function<void(MessageLogging message_logging)> message_logging_callback_t;

	/**
	 * @brief Callback type for Add Logged messages.
	 *
	 * @param message_add_logged Add Logged message.
	 */
	typedef std::function<void(MessageAddLogged message_add_logged)> message_add_logged_callback_t;

	/**
	 * @brief Callback type for Remove Logged messages.
	 *
	 * @param message_remove_logged Remove Logged message.
	 */
	typedef std::function<void(MessageRemoveLogged message_remove_logged)> message_remove_logged_callback_t;

	/**
	 * @brief Callback type for Dropout messages.
	 *
	 * @param message_dropout Dropout message.
	 */
	typedef std::function<void(MessageDropout message_dropout)> message_dropout_callback_t;

	/**
	 * @brief Subscribe to Flag Bits messages (asynchronous).
	 *
	 * **This feature is not yet implemented**.
	 *
	 * @param callback Function to call with updates.
	 */
	void message_flag_bits_async(message_flag_bits_callback_t callback);

	/**
	 * @brief Subscribe to Format messages (asynchronous).
	 *
	 * **This feature is not yet implemented**.
	 *
	 * @param callback Function to call with updates.
	 */
	void message_format_async(message_format_callback_t callback);

	/**
	 * @brief Subscribe to Info messages (asynchronous).
	 *
	 * **This feature is not yet implemented**.
	 *
	 * @param callback Function to call with updates.
	 */
	void message_info_async(message_info_callback_t callback);

	/**
	 * @brief Subscribe to Info Multiple messages (asynchronous).
	 *
	 * **This feature is not yet implemented**.
	 *
	 * @param callback Function to call with updates.
	 */
	void message_info_multiple_async(message_info_multiple_callback_t callback);

	/**
	 * @brief Subscribe to Data messages (asynchronous).
	 *
	 * **This feature is not yet implemented**.
	 *
	 * @param callback Function to call with updates.
	 */
	void message_data_async(message_data_callback_t callback);

	/**
	 * @brief Subscribe to Dropout messages (asynchronous).
	 *
	 * **This feature is not yet implemented**.
	 *
	 * @param callback Function to call with updates.
	 */
	void message_dropout_async(message_dropout_callback_t callback);

	/**
	 * @brief Subscribe to Parameter messages (asynchronous).
	 *
	 * **This feature is not yet implemented**.
	 *
	 * @param callback Function to call with updates.
	 */
	void message_parameter_async(message_parameter_callback_t callback);

	/**
	 * @brief Subscribe to Logging messages (asynchronous).
	 *
	 * **This feature is not yet implemented**.
	 *
	 * @param callback Function to call with updates.
	 */
	void message_logging_async(message_logging_callback_t callback);

	/**
	 * @brief Subscribe to Add Logged messages (asynchronous).
	 *
	 * **This feature is not yet implemented**.
	 *
	 * @param callback Function to call with updates.
	 */
	void message_add_logged_async(message_add_logged_callback_t callback);

	/**
	 * @brief Subscribe to Remove Logged messages (asynchronous).
	 *
	 * **This feature is not yet implemented**.
	 *
	 * @param callback Function to call with updates.
	 */
	void message_remove_logged_async(message_remove_logged_callback_t callback);

    // Non-copyable
    /**
     * @brief Copy constructor (object is not copyable).
     */
    Logging(const Logging&) = delete;
    /**
     * @brief Equality operator (object is not copyable).
     */
    const Logging& operator=(const Logging&) = delete;

private:

    /** @private Underlying implementation, set at instantiation */
    std::unique_ptr<LoggingImpl> _impl;
};

} // namespace mavsdk

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
 *
 * **This feature is not yet implemented**.
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
        uint8_t compat_flags[8]; /**< @brief Compatible flag bits. */
        uint8_t incompat_flags[8]; /**< @brief Incompatible flag bits. */
        uint64_t appended_offsets[3]; /**< @brief File offsets (0-based) for appended data. If no
                                         data is appended, all offsets must be zero. */
    };

    /**
     * @brief Format definition message type.
     *
     * Format definition for a single (composite) type that can be logged or used in another
     * definition as a nested type.
     */
    struct MessageFormat {
        std::vector<char>
            format; /**< @brief  plain-text string with the following format:
                     * 'message_name:field0;field1;'.
                     * There can be an arbitrary amount of fields (at least 1), separated by ';' */
    };

    /**
     * @brief Information message type. Definitions Section.
     */
    struct MessageInfo {
        std::vector<char> key; /**< @brief Plain string key name */
        std::vector<char> value; /**< @brief the data as described by key */
    };

    /**
     * @brief Information multiple message type. Definitions Section.
     */
    struct MessageInfoMultiple {
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
        std::vector<char> key; /**< @brief Plain string key name */
        std::vector<char> value; /**< @brief the data as described by key */
    };

    /**
     * @brief TODO
     */
    struct MessageAddLogged {
        uint8_t multi_id; /**< @brief TODO */
        uint16_t msg_id; /**< @brief Unique id to match MessageData data. */
        std::vector<char> message_name; /**< @brief Message name to subscribe to. */
    };

    /**
     * @brief TODO
     */
    struct MessageRemoveLogged {
        uint16_t msg_id; /**< @brief TODO */
    };

    /**
     * @brief TODO
     */
    struct MessageData {
        uint16_t msg_id; /**< @brief TODO */
        std::vector<uint8_t> data; /**< @brief The logged binary message as defined by TODO */
    };

    /**
     * @brief TODO
     */
    struct MessageLogging {
        enum class LogLevel {
            UNDEF = 0,
            EMERG = '0', /**< @brief System is unusable. */
            ALERT = '1', /**< @brief Action must be taken immediately. */
            CRIT = '2', /**< @brief Critical conditions. */
            ERR = '3', /**< @brief Error conditions. */
            WARNING = '4', /**< @brief Warning conditions. */
            NOTICE = '5', /**< @brief Normal but significant condition. */
            INFO = '6', /**< @brief Informational. */
            DEBUG = '7' /**< @brief Debug-level messages. */
        };

        LogLevel log_level; /**< @brief TODO */
        uint64_t timestamp; /**< @brief TODO */
        std::vector<char> data; /**< @brief TODO */
    };

    /**
     * @brief TODO
     */
    struct MessageDropout {
        uint16_t duration; /**< @brief Duration in ms. */
    };

    /**
     * @brief Callback type for flag bits messages.
     *
     * @param message_flag_bits Flag bits message.
     */
    typedef std::function<void(MessageFlagBits)> message_flag_bits_callback_t;

    /**
     * @brief Callback type for format messages.
     *
     * @param message_format Format message.
     */
    typedef std::function<void(MessageFormat)> message_format_callback_t;

    /**
     * @brief Callback type for info messages.
     *
     * @param message_info Info message.
     */
    typedef std::function<void(MessageInfo)> message_info_callback_t;

    /**
     * @brief Callback type for multiple info messages.
     *
     * @param message_info_multiple Multiple info message.
     */
    typedef std::function<void(MessageInfoMultiple)> message_info_multiple_callback_t;

    /**
     * @brief Callback type for parameter messages.
     *
     * @param message_parameter Parameter message.
     */
    typedef std::function<void(MessageParameter)> message_parameter_callback_t;

    /**
     * @brief Callback type for data messages.
     *
     * @param message_data Data message.
     */
    typedef std::function<void(MessageData)> message_data_callback_t;

    /**
     * @brief Callback type for logging messages.
     *
     * @param message_logging Logging message.
     */
    typedef std::function<void(MessageLogging)> message_logging_callback_t;

    /**
     * @brief Callback type for add logging messages.
     *
     * @param message_logging Logging message.
     */
    typedef std::function<void(MessageAddLogged)> message_add_logged_callback_t;

    /**
     * @brief Callback type for add logging messages.
     *
     * @param message_logging Logging message.
     */
    typedef std::function<void(MessageRemoveLogged)> message_remove_logged_callback_t;

    /**
     * @brief Callback type for dropout messages.
     *
     * @param message_logging Dropout message.
     */
    typedef std::function<void(MessageDropout)> message_dropout_callback_t;

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
     * **This feature is not yet implemented**.
     *
     * @return Result of request.
     */
    Result start_logging() const;

    /**
     * @brief Stop logging (synchronous).
     *
     * **This feature is not yet implemented**.
     *
     * @return Result of request.
     */
    Result stop_logging() const;

    /**
     * @brief Start logging (asynchronous).
     *
     * **This feature is not yet implemented**.
     *
     * @param callback Callback to get result of request.
     */
    void start_logging_async(result_callback_t callback);

    /**
     * @brief Stop logging (asynchronous).
     *
     * **This feature is not yet implemented**.
     *
     * @param callback Callback to get result of request.
     */
    void stop_logging_async(result_callback_t callback);

    // Non-copyable
    /**
     * @brief Copy constructor (object is not copyable).
     */
    Logging(const Logging&) = delete;
    /**
     * @brief Equality operator (object is not copyable).
     */
    const Logging& operator=(const Logging&) = delete;

    void message_flag_bits_async(message_flag_bits_callback_t callback);
    void message_format_async(message_format_callback_t callback);
    void message_info_async(message_info_callback_t callback);
    void message_info_multiple_async(message_info_multiple_callback_t callback);
    void message_data_async(message_data_callback_t callback);
    void message_dropout_async(message_dropout_callback_t callback);
    void message_parameter_async(message_parameter_callback_t callback);
    void message_logging_async(message_logging_callback_t callback);
    void message_add_logged_async(message_add_logged_callback_t callback);
    void message_remove_logged_async(message_remove_logged_callback_t callback);

private:
    /** @private Underlying implementation, set at instantiation */
    std::unique_ptr<LoggingImpl> _impl;
};

} // namespace mavsdk

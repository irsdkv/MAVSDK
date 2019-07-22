#pragma once

#include <mutex>

#include "mavlink_include.h"
#include "plugins/info/info.h"
#include "plugin_impl_base.h"

namespace mavsdk {

class InfoImpl : public PluginImplBase {
public:
    InfoImpl(System& system);
    ~InfoImpl();

    void init() override;
    void deinit() override;

    void enable() override;
    void disable() override;

    std::pair<Info::Result, Info::Identification> get_identification() const;
    std::pair<Info::Result, Info::Version> get_version() const;
    std::pair<Info::Result, Info::Product> get_product() const;

    InfoImpl(const InfoImpl&) = delete;
    InfoImpl& operator=(const InfoImpl&) = delete;

private:
    void request_version_again();
    void process_heartbeat(const mavlink_message_t& message);
    void process_autopilot_version(const mavlink_message_t& message);

    mutable std::mutex _mutex{};

    Info::Version _version{};
    Info::Product _product{};
    Info::Identification _identification{};
    bool _information_received{false};

    void* _call_every_cookie{nullptr};

    static const char* vendor_id_str(uint16_t vendor_id);
    static const char* product_id_str(uint16_t product_id);

    static void
    translate_binary_to_str(uint8_t* binary, unsigned binary_len, char* str, unsigned str_len);
};

} // namespace mavsdk

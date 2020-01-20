#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifdef WINDOWS
#define DLLExport __declspec(dllexport)
#else
#define DLLExport __attribute__((visibility("default")))
#endif

DLLExport void
runBackend(const char* system_address, const int mavsdk_server_port, void (*onServerStarted)(void*), void* context, bool enable_timesync);

#ifdef __cplusplus
}
#endif

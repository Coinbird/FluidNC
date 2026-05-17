// Copyright (c) 2024 FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#if defined(ESP32) && __has_include(<esp_now.h>)

#include "../src/Configuration/Configurable.h"

#include <esp_now.h>
#include <cstdint>

class ESPNowClient;
class ESPNowBroadcastChannel;

// ESPNowServer manages the ESP-NOW radio and implements the [FluidNC: ...] protocol.
//
// Two modes of operation:
//
//   Display mode  — any device sends "[FluidNC: $report/interval=N]" (broadcast).
//                   Server starts broadcasting status reports as "[FluidNC: <Idle|...>]".
//                   No connection needed; fire-and-forget for any number of displays.
//
//   Control mode  — device sends "[FluidNC: Connect]" (broadcast).
//                   Server creates a unicast ESPNowClient for that MAC, replies
//                   "[FluidNC: Connected]".  Subsequent GCode is raw bytes over unicast.
//                   Up to kMaxClients simultaneous remotes supported.
//                   All slots full → "[FluidNC: Busy]".
//
// Configure in YAML (key kept as espnow_channel for backward compatibility):
//   espnow_channel:
//     report_interval_ms: 500   # default broadcast interval; overridden by $report/interval=N

class ESPNowServer : public Configuration::Configurable {
    static constexpr int kMaxClients = 4;

    static ESPNowServer* _instance;

    static void onReceive(const uint8_t* mac, const uint8_t* data, int len);
    static void onSend(const uint8_t* mac, esp_now_send_status_t status);

    int32_t                _report_interval_ms = 500;
    int                    _next_id            = 0;  // increments with each new client
    ESPNowBroadcastChannel* _broadcastChannel  = nullptr;
    ESPNowClient*           _clients[kMaxClients] = {};

    bool initESPNow();

    void          handleFrame(const uint8_t* mac, const char* cmd);
    ESPNowClient* findClient(const uint8_t* mac) const;
    ESPNowClient* createClient(const uint8_t* mac);

    static void sendFrame(const uint8_t* mac, const char* msg);

public:
    ESPNowServer()  = default;
    ~ESPNowServer() = default;

    void init();  // called after WiFi is up (from Main.cpp)

    void group(Configuration::HandlerBase& handler) override {
        handler.item("report_interval_ms", _report_interval_ms);
    }
};

#endif  // defined(ESP32) && __has_include(<esp_now.h>)

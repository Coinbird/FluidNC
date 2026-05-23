// Copyright (c) 2026 FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "ESPNowServer.h"
#include "ESPNowClient.h"
#include "../src/Channel.h"
#include "../src/Serial.h"   // allChannels
#include "../src/Report.h"   // log_info, log_error, log_warn

#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>  // xTaskGetTickCount
#include <cstring>
#include <cstdio>
#include <cstdlib>

static const uint8_t kBroadcastMac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ── ESPNowBroadcastChannel ────────────────────────────────────────────────────
// Registered with allChannels so FluidNC's report system sends status strings
// here.  Only lines that start with '<' (status reports) are forwarded;
// everything else is discarded.  Accepted lines are wrapped as
// "[FluidNC: <Idle|Run|...>]" and sent as a broadcast ESP-NOW frame.

class ESPNowBroadcastChannel : public Channel {
    static constexpr size_t kLineBuf = 256;

    char   _buf[kLineBuf];
    size_t _len = 0;

    static void doBroadcast(const char* msg) {
        char   frame[300];
        size_t flen = snprintf(frame, sizeof(frame), "[FluidNC: %s]", msg);
        esp_now_send(kBroadcastMac, reinterpret_cast<const uint8_t*>(frame), flen);
    }

public:
    ESPNowBroadcastChannel() : Channel("espnow_broadcast") {}

    size_t write(uint8_t c) override {
        if (_len < kLineBuf - 1) {
            _buf[_len++] = c;
        }
        if (c == '\n' || _len >= kLineBuf - 1) {
            // Trim trailing CR/LF.
            while (_len > 0 && (_buf[_len - 1] == '\n' || _buf[_len - 1] == '\r')) {
                --_len;
            }
            _buf[_len] = '\0';
            if (_len > 0 && _buf[0] == '<') {
                doBroadcast(_buf);
            }
            _len = 0;
        }
        return 1;
    }

    size_t write(const uint8_t* buf, size_t len) override {
        for (size_t i = 0; i < len; i++) {
            write(buf[i]);
        }
        return len;
    }

    // Emit a status report on a fixed interval regardless of machine state.
    // The base Channel::autoReport() only fires periodically while the machine
    // is moving (Cycle/Homing/Jog); a passive display needs updates even when
    // the machine is idle, so this override drops the motion-state gate.
    void autoReport() override {
        if (_reportInterval == 0) {
            return;
        }
        if ((int32_t(xTaskGetTickCount()) - _nextReportTime) >= 0) {
            _nextReportTime = xTaskGetTickCount() + _reportInterval;
            report_realtime_status(*this);
        }
    }

    // Broadcast channel is write-only — no inbound data.
    int  available() override { return 0; }
    int  read() override { return -1; }
    int  peek() override { return -1; }
    void flushRx() override {}
    int  rx_buffer_available() override { return 0; }
    bool realtimeOkay(char /*c*/) override { return false; }
    bool lineComplete(char* /*line*/, char /*c*/) override { return false; }
};

// ── ESPNowServer ──────────────────────────────────────────────────────────────

ESPNowServer* ESPNowServer::_instance = nullptr;

// ── Static callbacks (WiFi task context) ──────────────────────────────────────

void ESPNowServer::onReceive(const uint8_t* mac, const uint8_t* data, int len) {
    if (!_instance || len <= 0) {
        return;
    }

    // Detect "[FluidNC: cmd]" control frames.
    static const char kPrefix[]  = "[FluidNC: ";
    static const int  kPrefixLen = sizeof(kPrefix) - 1;

    if (len > kPrefixLen && data[0] == '[' && data[len - 1] == ']' &&
        memcmp(data, kPrefix, kPrefixLen) == 0) {
        int  cmdLen = len - kPrefixLen - 1;  // strip prefix and trailing ']'
        char cmd[128] = {};
        if (cmdLen > 0 && cmdLen < (int)sizeof(cmd)) {
            memcpy(cmd, data + kPrefixLen, cmdLen);
            cmd[cmdLen] = '\0';
            _instance->handleFrame(mac, cmd);
        }
        return;
    }

    // Raw bytes — GCode from a connected remote, or a bare '?' channel probe.
    //
    // A '?' from an ACTIVE client is a normal status request → route to it.
    // A '?' from an unconnected OR *stale* client is a channel-alive probe →
    // reply "[FluidNC: ?]". The stale case matters: a remote that dropped (e.g.
    // walked out of range) still owns its client slot here, so without the
    // stale check its re-scan probes would be swallowed by pushBytes() and it
    // could never re-find our channel. Any non-probe bytes still route to the
    // client (stale or not) — it's clearly alive again.
    ESPNowClient* client = _instance->findClient(mac);
    bool          probe  = (len == 1 && data[0] == '?');
    if (client && !(probe && client->isStale())) {
        client->pushBytes(data, len);
    } else if (probe) {
        sendFrame(kBroadcastMac, "?");
    }
}

void ESPNowServer::onSend(const uint8_t* mac, esp_now_send_status_t /*status*/) {
    if (!_instance) {
        return;
    }
    // Broadcast sends don't count against any client's inflight budget.
    if (memcmp(mac, kBroadcastMac, 6) == 0) {
        return;
    }
    // Notify the matching client that a TX slot freed up.
    ESPNowClient* client = _instance->findClient(mac);
    if (client) {
        client->onSendComplete();
    }
}

// ── Frame handling ────────────────────────────────────────────────────────────

void ESPNowServer::handleFrame(const uint8_t* mac, const char* cmd) {
    // ── Control mode ──────────────────────────────────────────────────────────
    if (strcmp(cmd, "Connect") == 0) {
        ESPNowClient* existing = findClient(mac);
        if (existing) {
            // Reuse the existing client object — NEVER delete a Channel from
            // this WiFi-task callback; that races the main task's channel
            // poll and corrupts the heap. reactivate() resets the idle-timeout
            // state in place. The pendant keeps its original ID.
            existing->reactivate();
            char reply[32];
            snprintf(reply, sizeof(reply), "Connected id=%d", existing->id());
            sendFrame(mac, reply);
            log_info("espnow: remote (id=" << existing->id() << ") reconnected");
            return;
        }
        ESPNowClient* client = createClient(mac);
        if (client) {
            char reply[32];
            snprintf(reply, sizeof(reply), "Connected id=%d", client->id());
            sendFrame(mac, reply);
            log_info("espnow: remote (id=" << client->id() << ") connected");
        } else {
            sendFrame(mac, "Busy");
            log_warn("espnow: rejected remote (slots full)");
        }
        return;
    }

    if (strcmp(cmd, "Disconnect") == 0) {
        // Mark stale rather than delete — removeClient() from this WiFi-task
        // callback would race the main task's channel poll. The client object
        // is reused in place if the same pendant reconnects.
        ESPNowClient* client = findClient(mac);
        if (client && !client->isStale()) {
            client->markStale();
            log_info("espnow: remote (id=" << client->id() << ") disconnected");
        }
        return;
    }

    // Display mode is fully config-driven (espnow_channel: broadcast_interval_ms);
    // display devices are passive listeners and send no control frames.
}

// ── Client management ─────────────────────────────────────────────────────────

ESPNowClient* ESPNowServer::findClient(const uint8_t* mac) const {
    for (int i = 0; i < kMaxClients; i++) {
        if (_clients[i] && memcmp(_clients[i]->peerMac(), mac, 6) == 0) {
            return _clients[i];
        }
    }
    return nullptr;
}

ESPNowClient* ESPNowServer::createClient(const uint8_t* mac) {
    for (int i = 0; i < kMaxClients; i++) {
        if (_clients[i] == nullptr) {
            _clients[i] = new ESPNowClient(mac, _report_interval_ms, ++_next_id);
            allChannels.registration(_clients[i]);
            return _clients[i];
        }
    }
    return nullptr;  // all slots occupied
}

// ── Radio helpers ─────────────────────────────────────────────────────────────

void ESPNowServer::sendFrame(const uint8_t* mac, const char* msg) {
    char   frame[128];
    size_t flen = snprintf(frame, sizeof(frame), "[FluidNC: %s]", msg);
    esp_now_send(mac, reinterpret_cast<const uint8_t*>(frame), flen);
}

bool ESPNowServer::initESPNow() {
    if (esp_now_init() != ESP_OK) {
        log_error("espnow: esp_now_init failed");
        return false;
    }
    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb(onSend);

    // Pre-register the broadcast peer so the broadcast channel can TX
    // status reports before any remote has connected.
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, kBroadcastMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        log_error("espnow: failed to add broadcast peer");
        return false;
    }
    return true;
}

// ── init ──────────────────────────────────────────────────────────────────────

void ESPNowServer::init() {
    _instance = this;

    if (!initESPNow()) {
        return;
    }

    _broadcastChannel = new ESPNowBroadcastChannel();
    _broadcastChannel->setReportInterval(_broadcast_interval_ms);
    allChannels.registration(_broadcastChannel);

    if (_broadcast_interval_ms > 0) {
        log_info("espnow: server ready (broadcasting status every " << _broadcast_interval_ms << " ms)");
    } else {
        log_info("espnow: server ready (broadcast disabled; waiting for remotes)");
    }
}

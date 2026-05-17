// Copyright (c) 2024 FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#if defined(ESP32) && __has_include(<esp_now.h>)

#include "../src/Channel.h"
#include "../src/lineedit.h"

#include <esp_now.h>
#include <atomic>
#include <cstdint>

// ESPNowClient is a Channel bound to a single remote MAC address.
// Created by ESPNowServer::createClient() when a "[FluidNC: Connect]" frame
// arrives; destroyed on "[FluidNC: Disconnect]" or server teardown.
// TX is always unicast to the latched peer MAC.

class ESPNowClient : public Channel {
    static constexpr uint32_t kRxBufSize     = 512;
    static constexpr uint32_t kRxBufMask     = kRxBufSize - 1;
    static constexpr int      kMaxInflight   = 4;    // ESP-NOW internal queue is ~7
    static constexpr uint32_t kJogWatchdogMs = 500;  // inject JogCancel if pendant goes silent
    static constexpr uint32_t kIdleTimeoutMs = 4000; // log disconnect after 4 s of silence

    Lineedit* _lineedit = nullptr;

    // Lock-free SPSC ring. Producer = recv callback (WiFi task).
    // Consumer = main task via read()/peek()/available().
    uint8_t               _rx_ring[kRxBufSize];
    std::atomic<uint32_t> _rx_head { 0 };
    std::atomic<uint32_t> _rx_tail { 0 };

    // TX buffering + back-pressure tracking
    uint8_t          _tx_buf[250];
    size_t           _tx_len   = 0;
    std::atomic<int> _inflight { 0 };
    uint32_t         _tx_drops = 0;

    // Peer MAC and session ID: set at construction; immutable thereafter.
    uint8_t _peer_mac[6];
    int     _id = 0;

    // Last time a real packet was received from the pendant. Updated only by
    // pushBytes(); drives the disconnect timeout in autoReport().
    std::atomic<uint32_t> _last_rx_ms { 0 };

    // Jog-watchdog rate limiter (main task only — no atomic needed).
    uint32_t _last_jog_inject_ms = 0;

    // Idle-timeout disconnect detection. Doubles as the "already logged" guard.
    std::atomic<bool> _stale { false };

    void ringPush(uint8_t c);
    void flushTx();

public:
    explicit ESPNowClient(const uint8_t* mac, int32_t report_interval_ms, int id);

    // MAC and ID accessors for routing in ESPNowServer.
    const uint8_t* peerMac() const { return _peer_mac; }
    int            id()      const { return _id; }
    bool           isStale() const { return _stale.load(std::memory_order_relaxed); }

    // Called by ESPNowServer::onReceive (WiFi task context).
    // MAC already validated by the server before calling here.
    void pushBytes(const uint8_t* data, int len);

    // Called by ESPNowServer::onSend (WiFi task context).
    void onSendComplete();

    // Overrides Channel::autoReport() to detect idle timeout and set _stale.
    void autoReport() override;

    // Reset idle-timeout state when a previously-seen pendant reconnects.
    // Safe to call from the WiFi task — touches only atomics.
    void reactivate();

    // Mark the client stale (e.g. on an explicit Disconnect frame) without
    // deleting it. Safe to call from the WiFi task. The slot is reclaimed in
    // place if the same pendant reconnects later.
    void markStale() { _stale.store(true, std::memory_order_relaxed); }

    // Channel interface
    size_t write(uint8_t c) override;
    size_t write(const uint8_t* buf, size_t len) override;
    Error  pollLine(char* line) override;
    int    available() override;
    int    read() override;
    int    peek() override;
    void   flushRx() override;
    int    rx_buffer_available() override;
    bool   realtimeOkay(char c) override;
    bool   lineComplete(char* line, char c) override;
};

#endif  // defined(ESP32) && __has_include(<esp_now.h>)

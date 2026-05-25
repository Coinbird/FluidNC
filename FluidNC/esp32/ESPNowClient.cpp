// Copyright (c) 2026 FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "ESPNowClient.h"
#include "../src/Report.h"       // log_warn
#include "../src/State.h"        // state_is(), State::Jog
#include "../src/RealtimeCmd.h"  // execute_realtime_command(), Cmd::JogCancel

#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include <cstring>

ESPNowClient::ESPNowClient(const uint8_t* mac, int32_t report_interval_ms, int id) : Channel("espnow_client") {
    _lineedit = new Lineedit(this, _line, Channel::maxLine - 1);
    memcpy(_peer_mac, mac, 6);
    _id = id;
    setReportInterval(static_cast<uint32_t>(report_interval_ms));

    // Register the peer immediately so we can TX right away.
    if (!esp_now_is_peer_exist(mac)) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, mac, 6);
        peer.channel = 0;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }
}

// ── RX ───────────────────────────────────────────────────────────────────────

void ESPNowClient::ringPush(uint8_t c) {
    uint32_t head = _rx_head.load(std::memory_order_relaxed);
    uint32_t next = (head + 1) & kRxBufMask;
    if (next == _rx_tail.load(std::memory_order_acquire)) {
        return;  // ring full — drop
    }
    _rx_ring[head] = c;
    _rx_head.store(next, std::memory_order_release);
}

void ESPNowClient::pushBytes(const uint8_t* data, int len) {
    _last_rx_ms.store(millis(), std::memory_order_relaxed);
    for (int i = 0; i < len; i++) {
        ringPush(data[i]);
    }
}

// ── TX ───────────────────────────────────────────────────────────────────────

void ESPNowClient::onSendComplete() {
    _inflight.fetch_sub(1, std::memory_order_relaxed);
}

void ESPNowClient::autoReport() {
    Channel::autoReport();  // send status report to pendant as usual
    // Detect pendant timeout. _last_rx_ms is 0 until first packet so we
    // don't fire on a brand-new client that hasn't heard from its pendant yet.
    // _stale also guards against logging the disconnect more than once.
    if (!_stale.load(std::memory_order_relaxed)) {
        uint32_t last = _last_rx_ms.load(std::memory_order_relaxed);
        if (last != 0 && (millis() - last) > kIdleTimeoutMs) {
            _stale.store(true, std::memory_order_relaxed);
            log_info("espnow: remote (id=" << _id << ") disconnected");
        }
    }
}

void ESPNowClient::reactivate() {
    // Called from the WiFi task on [FluidNC: Connect] for an existing client.
    // Refresh the rx timestamp first so autoReport() can't re-flag stale in
    // the window before clearing _stale.
    _last_rx_ms.store(millis(), std::memory_order_relaxed);
    _stale.store(false, std::memory_order_relaxed);
}

void ESPNowClient::flushTx() {
    if (_tx_len == 0) {
        return;
    }
    for (int attempt = 0; attempt < 3; attempt++) {
        if (_inflight.load(std::memory_order_relaxed) < kMaxInflight) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (_inflight.load(std::memory_order_relaxed) >= kMaxInflight) {
        _tx_drops++;
        if ((_tx_drops & 0x1F) == 1) {
            log_warn("espnow_client: tx queue full, drops=" << _tx_drops);
        }
        _tx_len = 0;
        return;
    }
    esp_err_t err = esp_now_send(_peer_mac, _tx_buf, _tx_len);
    if (err == ESP_OK) {
        _inflight.fetch_add(1, std::memory_order_relaxed);
    } else {
        _tx_drops++;
        if ((_tx_drops & 0x1F) == 1) {
            log_warn("espnow_client: send err=" << (int)err << " drops=" << _tx_drops);
        }
    }
    _tx_len = 0;
}

size_t ESPNowClient::write(uint8_t c) {
    if (_tx_len < sizeof(_tx_buf)) {
        _tx_buf[_tx_len++] = c;
    }
    if (c == '\n' || _tx_len >= sizeof(_tx_buf)) {
        flushTx();
    }
    return 1;
}

size_t ESPNowClient::write(const uint8_t* buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        write(buf[i]);
    }
    return len;
}

// ── Channel interface ────────────────────────────────────────────────────────

int ESPNowClient::available() {
    uint32_t head = _rx_head.load(std::memory_order_acquire);
    uint32_t tail = _rx_tail.load(std::memory_order_relaxed);
    return static_cast<int>((head - tail) & kRxBufMask);
}

Error ESPNowClient::pollLine(char* line) {
    // Jog watchdog: while a jog is running, the pendant sends a keepalive
    // every ~250 ms (see MultiJogScene). If that keepalive stops for longer
    // than kJogWatchdogMs the pendant has dropped — cancel the jog so it can't
    // run away. Armed ONLY during State::Jog.
    //
    // Guard: only fire if the pendant sent something AFTER this jog started.
    // Without this, a WebUI jog would be cancelled after 500 ms because
    // _last_rx_ms predates the jog and the pendant stays silent.
    if (state_is(State::Jog)) {
        if (!_in_jog) {
            _in_jog       = true;
            _jog_start_ms = millis();
        }
        uint32_t last = _last_rx_ms.load(std::memory_order_relaxed);
        uint32_t now  = millis();
        // last > _jog_start_ms: pendant sent something during this jog
        if (last != 0 && last > _jog_start_ms &&
            (now - last) > kJogWatchdogMs &&
            (now - _last_jog_inject_ms) > kJogWatchdogMs) {
            // Pendant went silent mid-jog — cancel it. Execute the realtime
            // command directly: pollLine() runs in the main task, the same
            // context that normally dispatches realtime commands. Do NOT push
            // 0x85 into the RX ring — that ring is single-producer (the WiFi-task
            // recv callback via pushBytes()), so writing it from this task too
            // would race _rx_head and corrupt the byte stream. execute_realtime_
            // command() self-gates on State::Jog and posts motionCancelEvent,
            // exactly as a received 0x85 would.
            execute_realtime_command(Cmd::JogCancel, *this);
            _last_jog_inject_ms = now;
        }
    } else {
        _in_jog = false;
    }
    return Channel::pollLine(line);
}

int ESPNowClient::read() {
    uint32_t tail = _rx_tail.load(std::memory_order_relaxed);
    if (tail == _rx_head.load(std::memory_order_acquire)) {
        return -1;
    }
    uint8_t c = _rx_ring[tail];
    _rx_tail.store((tail + 1) & kRxBufMask, std::memory_order_release);
    return static_cast<int>(c);
}

int ESPNowClient::peek() {
    uint32_t tail = _rx_tail.load(std::memory_order_relaxed);
    if (tail == _rx_head.load(std::memory_order_acquire)) {
        return -1;
    }
    return static_cast<int>(_rx_ring[tail]);
}

void ESPNowClient::flushRx() {
    _rx_tail.store(_rx_head.load(std::memory_order_relaxed), std::memory_order_relaxed);
    Channel::flushRx();
}

int ESPNowClient::rx_buffer_available() {
    return std::max(0, int(kRxBufSize) - 1 - available());
}

bool ESPNowClient::realtimeOkay(char c) {
    return _lineedit->realtime(c);
}

bool ESPNowClient::lineComplete(char* line, char c) {
    if (_lineedit->step(c)) {
        _linelen        = _lineedit->finish();
        _line[_linelen] = '\0';
        strcpy(line, _line);
        _linelen = 0;
        return true;
    }
    return false;
}

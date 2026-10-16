#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace indicator {
enum State : uint8_t { STARTUP, READY, AUTOMATIC, MANUAL, PAUSED, COMPLETED, FAULT, EMERGENCY, COLLISION_PAUSED };
constexpr uint32_t TIMEOUT_MS = 3000;
constexpr uint8_t MAX_BRIGHTNESS = 102;  // 40% of 255.

inline uint16_t crc16(const char* text, size_t count) {
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < count; ++i) {
        crc ^= static_cast<uint16_t>(static_cast<uint8_t>(text[i])) << 8;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

inline bool number(const char* text, uint32_t maximum, uint32_t& out) {
    if (!*text) return false;
    out = 0;
    for (; *text; ++text) {
        if (*text < '0' || *text > '9') return false;
        const uint32_t digit = *text - '0';
        if (digit > maximum || out > (maximum - digit) / 10) return false;
        out = out * 10 + digit;
    }
    return true;
}

struct Frame {
    char session[17];
    uint32_t sequence;
    State state;
    uint8_t brightness;
    bool clear;
};

inline bool parse(const char* line, Frame& frame) {
    const size_t length = strlen(line);
    if (length < 7 || length >= 128 || line[length - 1] != '\n') return false;
    const char* star = strrchr(line, '*');
    if (!star || line + length - star != 6) return false;
    uint16_t checksum = 0;
    for (int i = 1; i <= 4; ++i) {
        const char c = star[i];
        const int digit = c >= '0' && c <= '9' ? c - '0' : (c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1);
        if (digit < 0) return false;
        checksum = static_cast<uint16_t>((checksum << 4) | digit);
    }
    if (checksum != crc16(line, star - line)) return false;
    char payload[128];
    memcpy(payload, line, star - line);
    payload[star - line] = 0;
    char* fields[6];
    fields[0] = payload;
    int count = 1;
    for (char* cursor = payload; *cursor; ++cursor) {
        if (*cursor == ',') {
            if (count == 6) return false;
            *cursor = 0;
            fields[count++] = cursor + 1;
        }
    }
    if (count != 6 || strcmp(fields[0], "S1") || strlen(fields[1]) != 16) return false;
    for (const char* c = fields[1]; *c; ++c)
        if (!(*c >= '0' && *c <= '9') && !(*c >= 'a' && *c <= 'f')) return false;
    uint32_t sequence, state, brightness, clear;
    if (!number(fields[2], UINT32_MAX, sequence) || sequence == 0 ||
        !number(fields[3], COLLISION_PAUSED, state) || !number(fields[4], MAX_BRIGHTNESS, brightness) ||
        !number(fields[5], 1, clear) || (clear && state == EMERGENCY)) return false;
    strcpy(frame.session, fields[1]);
    frame.sequence = sequence;
    frame.state = static_cast<State>(state);
    frame.brightness = static_cast<uint8_t>(brightness);
    frame.clear = clear != 0;
    return true;
}

enum Result { REJECTED, ACCEPTED, DUPLICATE };

class Receiver {
public:
    Result receive(const Frame& frame, uint32_t now) {
        if (strcmp(frame.session, session_) != 0) {
            for (const auto& retired : retired_)
                if (!strcmp(frame.session, retired)) return REJECTED;
            if (session_[0]) {
                strcpy(retired_[retired_index_], session_);
                retired_index_ = (retired_index_ + 1) % 4;
            }
            strcpy(session_, frame.session);
            sequence_ = 0;
        }
        if (frame.sequence < sequence_) return REJECTED;
        if (frame.sequence == sequence_) return DUPLICATE;  // ACK, but don't extend heartbeat
        sequence_ = frame.sequence;
        received_ = true;
        last_received_ = now;
        desired_ = frame.state;
        brightness_ = frame.brightness;
        if (frame.state == EMERGENCY) emergency_ = true;
        else if (frame.clear) emergency_ = false;
        return ACCEPTED;
    }

    State state(uint32_t now) const {
        if (emergency_) return EMERGENCY;
        if (!received_) return STARTUP;
        if (static_cast<uint32_t>(now - last_received_) > TIMEOUT_MS) return FAULT;
        return desired_;
    }

    uint8_t brightness() const { return brightness_; }

private:
    char session_[17] = {};
    char retired_[4][17] = {};
    uint8_t retired_index_ = 0;
    uint32_t sequence_ = 0;
    uint32_t last_received_ = 0;
    uint8_t brightness_ = MAX_BRIGHTNESS;
    State desired_ = STARTUP;
    bool received_ = false;
    bool emergency_ = false;
};

inline void ack(const Frame& frame, char* output, size_t capacity) {
    char payload[64];
    const int length = snprintf(payload, sizeof(payload), "A1,%s,%lu", frame.session,
                                static_cast<unsigned long>(frame.sequence));
    snprintf(output, capacity, "%s*%04X\n", payload, static_cast<unsigned>(crc16(payload, length)));
}
}  // namespace indicator

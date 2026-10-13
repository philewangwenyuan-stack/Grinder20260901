#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <esp_task_wdt.h>
#include "indicator_protocol.h"

// No Wi-Fi/Bluetooth initialization. GPIO21 is the board's onboard LED;
// GPIO4 exclusively drives the external strip via a 5V AHCT buffer.
constexpr uint8_t PIXEL_PIN = 4;
constexpr uint8_t PIXEL_COUNT = 3;
Adafruit_NeoPixel pixels(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
indicator::Receiver receiver;
char line[128];
size_t line_size = 0;
bool discard_line = false;
indicator::State displayed_state = indicator::STARTUP;
uint32_t state_started = 0;
uint32_t last_render = 0;

void readUSB(uint32_t now) {
    // Bound processing so a noisy input cannot starve rendering/watchdog.
    for (int budget = 0; budget < 256 && Serial.available(); ++budget) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\n') {
            if (!discard_line) {
                line[line_size++] = '\n';
                line[line_size] = 0;
                indicator::Frame frame;
                if (indicator::parse(line, frame) && receiver.receive(frame, now) != indicator::REJECTED) {
                    char response[80];
                    indicator::ack(frame, response, sizeof(response));
                    if (Serial && Serial.availableForWrite() >= static_cast<int>(strlen(response)))
                        Serial.write(reinterpret_cast<const uint8_t*>(response), strlen(response));
                }
            }
            line_size = 0;
            discard_line = false;
        } else if (!discard_line) {
            if (c == 0 || line_size >= sizeof(line) - 2) discard_line = true;
            else line[line_size++] = c;
        }
    }
}

void render(uint32_t now) {
    const auto state = receiver.state(now);
    if (state != displayed_state) {
        displayed_state = state;
        state_started = now;
    }
    const uint32_t age = now - state_started;
    uint8_t red = 0, green = 0, blue = 0;
    const uint8_t level = receiver.brightness();
    switch (state) {
        case indicator::STARTUP: red = green = level; break;
        case indicator::READY: green = level; break;
        case indicator::AUTOMATIC: {
            const float phase = (age % 2000) / 2000.0f;
            blue = static_cast<uint8_t>(level * (0.1f + 0.9f * (1.0f - cosf(phase * 2.0f * PI)) / 2.0f));
            break;
        }
        case indicator::MANUAL: green = blue = level; break;
        case indicator::PAUSED: blue = age % 1000 < 500 ? level : 0; break;
        case indicator::COLLISION_PAUSED: red = green = age % 1000 < 500 ? level : 0; break;
        case indicator::COMPLETED: green = age >= 3000 || age % 1000 < 500 ? level : 0; break;
        case indicator::FAULT: red = age % 1000 < 500 ? level : 0; break;
        case indicator::EMERGENCY: red = age % 250 < 125 ? level : 0; break;
    }
    const uint32_t color = pixels.Color(red, green, blue);
    for (int i = 0; i < PIXEL_COUNT; ++i) pixels.setPixelColor(i, color);
    pixels.show();
}

void setup() {
    pixels.begin();
    render(millis());  // Yellow before USB enumeration and before ROS.
    Serial.begin(115200);
    // Never wait for Serial/host connection: boot indication is autonomous.
    esp_task_wdt_init(5, true);
    esp_task_wdt_add(nullptr);
}

void loop() {
    const uint32_t now = millis();
    readUSB(now);
    if (now - last_render >= 20) {
        render(now);
        last_render = now;
    }
    esp_task_wdt_reset();
    delay(1);
}

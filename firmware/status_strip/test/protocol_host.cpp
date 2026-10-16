// Compile on a host with: c++ -std=c++11 -Iinclude test/protocol_host.cpp -o protocol_host
#include <assert.h>
#include "indicator_protocol.h"

using namespace indicator;

Frame makeFrame(const char* session, uint32_t sequence, State state, bool clear = false) {
    Frame frame;
    strcpy(frame.session, session);
    frame.sequence = sequence;
    frame.state = state;
    frame.brightness = MAX_BRIGHTNESS;
    frame.clear = clear;
    return frame;
}

int main(int argc, char**) {
    setvbuf(stdout, nullptr, _IONBF, 0);
    assert(crc16("123456789", 9) == 0x29b1);
    Receiver receiver;
    assert(receiver.brightness() == DEFAULT_BRIGHTNESS);
    const char* first = "0123456789abcdef";
    const char* second = "abcdef0123456789";
    assert(receiver.state(100000) == STARTUP);
    assert(receiver.receive(makeFrame(first, 1, AUTOMATIC), 100) == ACCEPTED);
    assert(receiver.brightness() == MAX_BRIGHTNESS);
    assert(receiver.state(3100) == AUTOMATIC);
    assert(receiver.receive(makeFrame(first, 1, AUTOMATIC), 3101) == DUPLICATE);
    assert(receiver.state(3101) == FAULT);  // duplicate cannot feed watchdog
    assert(receiver.receive(makeFrame(first, 2, EMERGENCY), 3200) == ACCEPTED);
    assert(receiver.receive(makeFrame(first, 3, AUTOMATIC), 3300) == ACCEPTED);
    assert(receiver.state(100000) == EMERGENCY);  // old RUNNING cannot erase emergency
    assert(receiver.receive(makeFrame(second, 1, READY), 3400) == ACCEPTED);
    assert(receiver.state(3400) == EMERGENCY);  // reconnect also cannot erase it
    assert(receiver.receive(makeFrame(first, 4, READY, true), 3500) == REJECTED);
    assert(receiver.receive(makeFrame(second, 2, READY, true), 3600) == ACCEPTED);
    assert(receiver.state(3600) == READY);
    assert(receiver.receive(makeFrame(second, 1, EMERGENCY), 3700) == REJECTED);
    Receiver wrapped;
    assert(wrapped.receive(makeFrame(first, 1, PAUSED), UINT32_MAX - 100) == ACCEPTED);
    assert(wrapped.state(100) == PAUSED);
    assert(wrapped.state(4000) == FAULT);
    Receiver collision_pause;
    assert(collision_pause.receive(makeFrame(first, 1, COLLISION_PAUSED), 100) == ACCEPTED);
    assert(collision_pause.state(100) == COLLISION_PAUSED);
    assert(collision_pause.receive(makeFrame(first, 2, EMERGENCY), 200) == ACCEPTED);
    assert(collision_pause.receive(makeFrame(first, 3, COLLISION_PAUSED), 300) == ACCEPTED);
    assert(collision_pause.state(300) == EMERGENCY);
    puts("receiver self-tests passed");
    // Optional stdin mode: independently validate Python-generated wire frames.
    if (argc > 1) {
        char line[512];
        while (fgets(line, sizeof(line), stdin)) {
            Frame frame;
            if (!parse(line, frame)) { puts("REJECTED"); continue; }
            char response[80];
            ack(frame, response, sizeof(response));
            fputs(response, stdout);
        }
    }
}

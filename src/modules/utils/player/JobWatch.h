#pragma once

#include "FreeRTOS.h"

#include <cstdint>

class Player;
class StreamOutput;

class JobWatch {
public:
    explicit JobWatch(Player &player) : player(player) {}

    bool add(StreamOutput *stream);
    void remove(StreamOutput *stream);
    void tick();

    static const int WATCHERS = 5;

private:
    static const unsigned PUSH_MS = 40;
    static const unsigned AGAIN_MS = 1000;

    Player &player;
    StreamOutput *watchers[WATCHERS] = {};
    uint16_t pushed_crc = 0;
    TickType_t checked_at = 0, pushed_at = 0;
};

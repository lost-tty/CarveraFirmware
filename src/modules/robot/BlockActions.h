#pragma once

#include "GcodeLine.h"
#include "McodeRegistry.h"

#include <cstdint>

class Gcode;

// An ACTION M code runs where it was written in the path, without stopping the machine for it.
// The code waits here until the block it was written after has finished, then runs on the main
// task. Several codes may wait on one block, and a code written with nothing queued runs at once.
class BlockActions
{
public:
    static const uint8_t k_max_pending = 8;
    static const uint8_t k_max_words = 4;   // an action carries its command and a value or two

    // false: it could not be held, so the caller runs it now rather than losing it
    bool hold(const McodeRegistry::Mcode *code, const Gcode &gcode, uint32_t after_block);
    void run_upto(uint32_t finished_block);
    void clear() { count= 0; }
    bool empty() const { return count == 0; }

private:
    // the words are copied out of the line: the line is gone by the time this runs
    struct Pending {
        const McodeRegistry::Mcode *code;
        uint32_t after_block;
        gcode::Word words[k_max_words];
        uint8_t n_words;
        uint16_t number;
        uint8_t subcode;
    };

    void run(const Pending &p);

    Pending pending[k_max_pending];
    uint8_t count= 0;
};

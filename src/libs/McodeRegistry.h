#pragma once

#include <cstdint>
#include <type_traits>

class Gcode;

// Every M code has one owner. The owner keeps the slot, the registry only threads them.
class McodeRegistry
{
public:
    // When an M code runs, relative to the motion around it. Pick IMMEDIATE if the handler does
    // not care where the machine is, ACTION if it must happen at this point in the path, and
    // BARRIER if the next move must not start until it is finished.
    enum When : uint8_t {
        IMMEDIATE,  // runs as the line is read, while queued motion carries on
        ACTION,     // runs at the block it was written before, without stopping; waits like BARRIER until a block can carry one
        BARRIER,    // everything queued before it runs first, and the handler may take as long as it likes

        MID_JOB= 0x80,                      // the operator may type it while a job runs
        BESIDE_JOB= IMMEDIATE | MID_JOB,
    };

    static const uint8_t ANY_SUBCODE= 0xff;

    using McodeFn = void (*)(void *owner, Gcode *);
    struct Mcode {
        uint16_t number;
        uint8_t subcode;
        uint8_t when;
        void *owner;
        McodeFn handler;
        Mcode *next;
    };

    // an owner need not be a Module: a leveling strategy claims its own codes.
    // false: the slot was left untouched because someone else already claimed the code
    static bool add(Mcode &slot, uint16_t number, uint8_t subcode, uint8_t when, void *owner, McodeFn handler);

    template<class T, void (T::*M)(Gcode *)> static bool add(Mcode &slot, uint16_t number, uint8_t subcode, uint8_t when, T *owner)
    {
        return add(slot, number, subcode, when, owner, [](void *self, Gcode *gcode) { (((T *)self)->*M)(gcode); });
    }

    static const Mcode *find(uint16_t number, uint8_t subcode);

private:
    static Mcode *mcodes;
};

// the code with every subcode it may carry; ADD_SUBCODE claims one exact spelling
#define ADD_MCODE(slot, number, when, method) \
    McodeRegistry::add<std::remove_reference<decltype(*this)>::type, &method>(slot, number, McodeRegistry::ANY_SUBCODE, McodeRegistry::when, this)
#define ADD_SUBCODE(slot, number, subcode, when, method) \
    McodeRegistry::add<std::remove_reference<decltype(*this)>::type, &method>(slot, number, subcode, McodeRegistry::when, this)

#ifndef SERIALMESSAGE_H
#define SERIALMESSAGE_H

#include <cstdint>
#include <string>

class StreamOutput;
namespace gcode { class ParamStore; }

struct SerialMessage {
        StreamOutput* stream;
        std::string message;
        uint32_t mark;                     // call << 24 | line, see Program; 0 for none
        const gcode::ParamStore *params;   // to evaluate with; null: the machine's
};
#endif

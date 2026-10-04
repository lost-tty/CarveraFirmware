#ifndef SERIALMESSAGE_H
#define SERIALMESSAGE_H

#include <string>

class StreamOutput;
namespace gcode { class ParamStore; }

struct SerialMessage {
        StreamOutput* stream;
        std::string message;
        unsigned int line;
        const gcode::ParamStore *params;   // to evaluate with; null: the machine's
};
#endif

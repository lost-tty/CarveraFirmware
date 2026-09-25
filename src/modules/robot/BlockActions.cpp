#include "BlockActions.h"

#include "Gcode.h"
#include "Logging.h"

#include <cstring>

int8_t BlockActions::take_text(const std::string &from)
{
    for (uint8_t i = 0; i < k_max_pending; i++) {
        if(taken & (1 << i)) {
            continue;
        }
        size_t n = from.size() < k_max_text - 1 ? from.size() : k_max_text - 1;
        memcpy(texts[i], from.data(), n);
        texts[i][n] = '\0';
        taken |= 1 << i;
        return (int8_t)i;
    }
    return -1;
}

bool BlockActions::hold(const McodeRegistry::Mcode *code, const Gcode &gcode, uint32_t after_block)
{
    if(count >= k_max_pending) return false;

    Pending &p = pending[count];
    p.code = code;
    p.after_block = after_block;
    p.number = gcode.m;
    p.subcode = gcode.subcode;
    p.n_words = 0;
    p.text = -1;

    if(!gcode.text.empty()) {
        p.text = take_text(gcode.text);
        if(p.text < 0) {
            return false;
        }
    }

    for (const gcode::Word &w : gcode.get_words()) {
        if(w.letter == 'G' || w.letter == 'M') continue;   // held apart, so a second command word cannot displace it
        if(p.n_words >= k_max_words) {                     // rather run it late than run it wrong
            if(p.text >= 0) {
                taken &= ~(1 << p.text);
            }
            return false;
        }
        p.words[p.n_words++] = w;
    }

    count++;
    return true;
}

void BlockActions::run(const Pending &p)
{
    gcode::Words words;
    words.push_back(gcode::Word{.letter= 'M', .subcode= p.subcode,
                                .has_value= true, .value= (float)p.number});
    for (uint8_t i = 0; i < p.n_words; i++) words.push_back(p.words[i]);

    Gcode gcode(words, 0, nullptr, 0);
    if(p.text >= 0) {
        gcode.text = texts[p.text];
    }
    p.code->handler(p.code->owner, &gcode);
}

// the queue retires in order, so everything that comes due is at the front
void BlockActions::run_upto(uint32_t finished_block)
{
    // each one leaves the list before it runs: a handler that queues a move re-enters through collect
    while (count > 0 && pending[0].after_block <= finished_block) {
        Pending due = pending[0];
        for (uint8_t i = 1; i < count; i++) pending[i - 1] = pending[i];
        count--;
        if(due.text >= 0) {
            taken &= ~(1 << due.text);
        }
        run(due);
    }
}

#include "BlockActions.h"

#include "Gcode.h"
#include "Logging.h"

bool BlockActions::hold(const McodeRegistry::Mcode *code, const Gcode &gcode, uint32_t after_block)
{
    if(count >= k_max_pending) return false;

    Pending &p = pending[count];
    p.code = code;
    p.after_block = after_block;
    p.number = gcode.m;
    p.subcode = gcode.subcode;
    p.n_words = 0;

    for (const gcode::Word &w : gcode.get_words()) {
        if(w.letter == 'G' || w.letter == 'M') continue;   // held apart, so a second command word cannot displace it
        if(p.n_words >= k_max_words) return false;         // rather run it late than run it wrong
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
        run(due);
    }
}

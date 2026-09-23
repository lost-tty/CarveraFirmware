#include "McodeRegistry.h"
#include "Logging.h"

McodeRegistry::Mcode *McodeRegistry::mcodes= nullptr;

bool McodeRegistry::add(Mcode &slot, uint16_t number, uint8_t subcode, uint8_t when, void *owner, McodeFn handler)
{
    for (const Mcode *m = mcodes; m != nullptr; m = m->next) {
        if(m == &slot) return false;   // re-registering a slot would make the list point at itself

        if(m->number != number || m->subcode != subcode) continue;
        // the second owner would never be called, so say which code went nowhere
        if(subcode == ANY_SUBCODE)
            printk("ERROR: M%u is already claimed\n", number);
        else
            printk("ERROR: M%u.%u is already claimed\n", number, subcode);
        return false;
    }

    slot = Mcode{number, subcode, when, owner, handler, mcodes};
    mcodes = &slot;
    return true;
}

// an owner that named this subcode shuts out the one that took every subcode
const McodeRegistry::Mcode *McodeRegistry::find(uint16_t number, uint8_t subcode)
{
    const Mcode *any= nullptr;
    for (const Mcode *m = mcodes; m != nullptr; m = m->next) {
        if(m->number != number) continue;
        if(m->subcode == subcode) return m;
        if(m->subcode == ANY_SUBCODE) any= m;
    }
    return any;
}

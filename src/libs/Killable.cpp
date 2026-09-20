#include "Killable.h"

#ifdef __ARM_EABI__
#include "cmsis.h"
#else
static inline void __disable_irq() {}   // the host test builds this file too
static inline void __enable_irq() {}
#endif

Killable *Killable::head = nullptr;

// kill_all walks this list from an interrupt, so unlinking must not be visible half done
Killable::~Killable()
{
    __disable_irq();
    for (Killable **p = &head; *p != nullptr; p = &(*p)->next) {
        if(*p == this) {
            *p = this->next;
            break;
        }
    }
    __enable_irq();
}

#pragma once

// An output that must stop when the machine halts.
//
// kill()    may run in an interrupt: pin writes only, nothing that allocates, prints or waits.
// cleanup() runs on the main loop afterwards, where blocking is allowed.
// restore() undoes kill() and nothing else: a supply rail comes back, a spindle does not spin up.
class Killable {
public:
    Killable() { next = head; head = this; }
    virtual ~Killable();

    virtual void kill() = 0;
    virtual void cleanup() {}
    virtual void restore() {}

    static void kill_all() { for (Killable *k = head; k != nullptr; k = k->next) k->kill(); }
    static void cleanup_all() { for (Killable *k = head; k != nullptr; k = k->next) k->cleanup(); }
    static void restore_all() { for (Killable *k = head; k != nullptr; k = k->next) k->restore(); }

private:
    static Killable *head;
    Killable *next;
};

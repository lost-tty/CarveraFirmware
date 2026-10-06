#ifndef SHAREDBUFFER_H
#define SHAREDBUFFER_H

#include <cstddef>
#include <cstdint>

// Memory several users take turns with, from any task: one holds it at a time.
class SharedBuffer {
public:
    SharedBuffer(uint8_t* data, size_t size) : data(data), size(size) {}

    // true: who holds it, now or already
    bool take(const void* who);
    void give(const void* who);

    uint8_t* const data;
    const size_t size;

private:
    const void* volatile holder = nullptr;
};

#endif

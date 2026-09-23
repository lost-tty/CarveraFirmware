#include "Persist.h"

#include "I2C.h"
#include "wait_api.h"
#include "Logging.h"

#include <cstring>

#define EEP_MAX_PAGE_SIZE     32
#define EEPROM_DATA_STARTPAGE 1

Persist persist;

void Persist::init(mbed::I2C *bus)
{
    i2c = bus;

    size_t size = sizeof(Record);
    char buf[size];

    short address = EEPROM_DATA_STARTPAGE * EEP_MAX_PAGE_SIZE;
    i2c->start();
    i2c->write(0xA0);
    i2c->write((unsigned char)(address >> 8));
    i2c->write((unsigned char)(address & 0xff));
    i2c->start();
    i2c->write(0xA1);
    for (size_t i = 0; i < size; i++) buf[i] = i2c->read(1);
    i2c->stop();
    i2c->stop();
    wait(0.05);

    memcpy(&record, buf, size);
    written = record;
}

void Persist::set_work_offset(float x, float y, float z)
{
    record.work_offset[0]= x;
    record.work_offset[1]= y;
    record.work_offset[2]= z;
    save();
}

// the chip is only written when a value actually changed: a write blocks for a few hundred ms
void Persist::save()
{
    if(memcmp(&record, &written, sizeof(Record)) == 0) return;
    if(store(&record)) written = record;
}

void Persist::erase()
{
    Record zero{};
    if(store(&zero)) {
        record = zero;
        written = zero;
        printk("EEPROM data erase finished.\n");
    }
}

bool Persist::store(const void *from)
{
    size_t size = sizeof(Record);
    char buf[size];
    memcpy(buf, from, size);

    const uint8_t *p = (const uint8_t *)buf;
    for (size_t done = 0, page = 0; done < size; page++) {
        uint8_t len = size - done >= EEP_MAX_PAGE_SIZE ? EEP_MAX_PAGE_SIZE : size - done;
        if(!page_write(EEPROM_DATA_STARTPAGE + page, len, p)) {
            printk("ALARM: EEPROM write failed at page %u\n", (unsigned)page);
            return false;
        }
        wait(0.1);
        done += len;
        p += len;
    }
    return true;
}

// true when the chip acknowledged every byte
bool Persist::page_write(uint8_t page, uint8_t len, const uint8_t *from)
{
    unsigned int address = (unsigned int)page << 5;

    i2c->start();
    bool ok= i2c->write(0xA0) == 1;
    ok= i2c->write((unsigned char)(address >> 8)) == 1 && ok;
    ok= i2c->write((unsigned char)address) == 1 && ok;
    for (uint8_t i = 0; i < len; i++) ok= i2c->write(from[i]) == 1 && ok;
    i2c->stop();
    i2c->stop();

    return ok;
}

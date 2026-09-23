#include "Persist.h"

#include "I2C.h"
#include "wait_api.h"
#include "Logging.h"

#include <cstring>
#include <cmath>

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

// a chip written before this field existed reads as NaN there, which is no offset at all
float Persist::work_offset(uint8_t wcs, uint8_t axis) const
{
    if(wcs >= k_work_offsets || axis > 2) return 0;
    float v= wcs == 0 ? record.work_offset[axis] : record.more_work_offsets[wcs - 1][axis];
    return std::isnan(v) ? 0 : v;
}

void Persist::set_work_offset(uint8_t wcs, float x, float y, float z)
{
    if(wcs >= k_work_offsets) return;
    float *o= wcs == 0 ? record.work_offset : record.more_work_offsets[wcs - 1];
    o[0]= x;
    o[1]= y;
    o[2]= z;
    save();
}

void Persist::save()
{
    store(&record);
}

bool Persist::erase()
{
    Record zero{};
    record = zero;
    return store(&record);
}

// each page costs a tenth of a second, so only the ones that differ from the chip are written
bool Persist::store(const void *from)
{
    size_t size = sizeof(Record);
    char buf[size];
    memcpy(buf, from, size);

    const uint8_t *p = (const uint8_t *)buf;
    uint8_t *cached = (uint8_t *)&written;

    for (size_t done = 0, page = 0; done < size; page++) {
        uint8_t len = size - done >= EEP_MAX_PAGE_SIZE ? EEP_MAX_PAGE_SIZE : size - done;
        if(memcmp(p, cached + done, len) != 0) {
            if(!page_write(EEPROM_DATA_STARTPAGE + page, len, p)) {
                printk("ALARM: EEPROM write failed at page %u\n", (unsigned)page);
                return false;
            }
            wait(0.1);
            memcpy(cached + done, p, len);
        }
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

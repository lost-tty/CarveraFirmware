// PinGroup::any() must agree with Pin::get() for every combination of inversion and level.
#include <cstdio>
#include <cstdint>
#include <initializer_list>
static int fails = 0;
#define CHECK(c) do { if(!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while(0)

struct FakePort { uint32_t FIOPIN; };
struct FakePin {
    FakePort *port; unsigned char pin; bool inverting, valid;
    bool connected() const { return valid; }
    bool is_inverting() const { return inverting; }
    bool get() const { return inverting ^ ((port->FIOPIN >> pin) & 1); }
};

// PinGroup, transcribed
class Group {
public:
    void clear() { n= 0; }
    void add(const FakePin &p, bool inverted = false) {
        if(!p.connected()) return;
        for (uint8_t i=0;i<n;++i) if(ports[i]==p.port) { claim(i,p,inverted); return; }
        ports[n]=p.port; mask[n]=0; expected[n]=0; claim(n,p,inverted); n++;
    }
    bool any() const {
        for (uint8_t i=0;i<n;++i) if((ports[i]->FIOPIN ^ expected[i]) & mask[i]) return true;
        return false;
    }
private:
    void claim(uint8_t i, const FakePin &p, bool inverted) {
        mask[i] |= 1UL << p.pin;
        if(p.is_inverting() != inverted) expected[i] |= 1UL << p.pin;
    }
    FakePort *ports[5]{}; uint32_t mask[5]{}, expected[5]{}; uint8_t n{0};
};

int main()
{
    FakePort p0{0}, p1{0};

    // one pin, both polarities, both levels: any() must equal get()
    for (int inv = 0; inv < 2; inv++) {
        for (int level = 0; level < 2; level++) {
            p0.FIOPIN = level ? (1u << 24) : 0;
            FakePin pin{&p0, 24, (bool)inv, true};
            Group g; g.add(pin);
            CHECK(g.any() == pin.get());
        }
    }

    // the real config: three on port 0, three on port 1, all pull-up (not inverting)
    FakePin a{&p0,24,false,true}, b{&p0,25,false,true}, c{&p0,26,false,true};
    FakePin d{&p1, 1,false,true}, e{&p1, 4,false,true}, f{&p1, 8,false,true};
    Group g; g.add(a); g.add(b); g.add(c); g.add(d); g.add(e); g.add(f);

    // every pin high = nothing asserted
    p0.FIOPIN = 0;   // non-inverting pins idle low
    p1.FIOPIN = 0;
    CHECK(!g.any());

    // any single pin low is an assertion, on either port
    for (int bit : {24,25,26}) {
        p0.FIOPIN = (1u<<bit);
        CHECK(g.any());
    }
    p0.FIOPIN = 0;
    for (int bit : {1,4,8}) {
        p1.FIOPIN = (1u<<bit);
        CHECK(g.any());
    }
    p1.FIOPIN = 0;
    CHECK(!g.any());

    // a pin outside the group must not register
    p0.FIOPIN |= (1u<<3);  CHECK(!g.any());
    p0.FIOPIN &= ~(1u<<3); CHECK(!g.any());

    // mixed polarity on one port
    FakePort p2{0};
    FakePin norm{&p2,5,false,true}, inv{&p2,6,true,true};
    Group m; m.add(norm); m.add(inv);
    p2.FIOPIN = (1u<<6);            // norm low (idle), inv high (idle)
    CHECK(!m.any());
    p2.FIOPIN = (1u<<5)|(1u<<6);    // norm high = asserted
    CHECK(m.any());
    p2.FIOPIN = 0;                  // inv low = asserted
    CHECK(m.any());

    // an unconnected pin is ignored
    Group u; FakePin nc{&p0,7,false,false}; u.add(nc);
    CHECK(!u.any());

    // inverted: asserted means the pin reads its idle level, for probing away from a switch
    {
        FakePort p3{0};
        FakePin  pin{&p3, 9, false, true};
        Group away; away.add(pin, true);
        p3.FIOPIN = (1u<<9);   // normally asserted
        CHECK(!away.any());
        p3.FIOPIN = 0;         // released, which is what we are waiting for
        CHECK(away.any());
    }

    // clear() makes a group reusable for the next move
    {
        FakePort p4{0};
        FakePin  pin{&p4, 2, false, true};
        Group g2; g2.add(pin);
        p4.FIOPIN = (1u<<2);
        CHECK(g2.any());
        g2.clear();
        CHECK(!g2.any());
    }

    printf(fails == 0 ? "pingroup: all passed\n" : "pingroup: %d FAILED\n", fails);
    return fails != 0;
}

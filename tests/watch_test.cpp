// The step ticker's watch: how many triggered ticks it takes, which motors stop, and that the
// latched count is the one at the trigger rather than where the axis came to rest.
#include <cstdio>
#include <cstdint>
#include <initializer_list>

static int fails = 0;
#define CHECK(c) do { if(!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while(0)

static const int MOTORS = 3;

struct Watch {
    bool     asserted{false};     // stands in for inputs.any()
    uint8_t  motors{0};
    uint16_t hysteresis{1};
    uint16_t count{0};
    bool     hit{false};
    int32_t  at_steps[MOTORS]{};
    void arm() { count = 0; hit = false; }
};

static int32_t position[MOTORS];
static bool    moving[MOTORS];

// StepTicker::step_tick's watch block, transcribed
static void tick(Watch *w)
{
    if(w != nullptr && !w->hit) {
        if(!w->asserted) {
            w->count = 0;
        } else {
            if(w->count == 0) for (int m = 0; m < MOTORS; m++) w->at_steps[m] = position[m];
            if(++w->count >= w->hysteresis) {
                for (int m = 0; m < MOTORS; m++) if(w->motors & (1 << m)) moving[m] = false;
                w->hit = true;
            }
        }
    }
    for (int m = 0; m < MOTORS; m++) if(moving[m]) position[m]++;   // one step per tick
}

static void reset() { for (int m = 0; m < MOTORS; m++) { position[m] = 0; moving[m] = true; } }

int main()
{
    // hysteresis 1: the first asserted tick stops it
    {
        reset();
        Watch w; w.motors = 1; w.hysteresis = 1; w.arm();
        tick(&w); CHECK(!w.hit);
        w.asserted = true;
        tick(&w); CHECK(w.hit);
        CHECK(!moving[0]);
    }

    // hysteresis 20: nineteen asserted ticks are not enough
    {
        reset();
        Watch w; w.motors = 1; w.hysteresis = 20; w.arm();
        w.asserted = true;
        for (int i = 0; i < 19; i++) tick(&w);
        CHECK(!w.hit);
        CHECK(moving[0]);
        tick(&w);
        CHECK(w.hit);
    }

    // a release before the count is reached starts it over
    {
        reset();
        Watch w; w.motors = 1; w.hysteresis = 20; w.arm();
        w.asserted = true;  for (int i = 0; i < 19; i++) tick(&w);
        w.asserted = false; tick(&w);
        CHECK(w.count == 0);
        w.asserted = true;  for (int i = 0; i < 19; i++) tick(&w);
        CHECK(!w.hit);
    }

    // only the named motors stop; the others keep stepping
    {
        reset();
        Watch w; w.motors = (1 << 0); w.hysteresis = 1; w.arm();
        w.asserted = true;
        tick(&w);
        CHECK(!moving[0]);
        CHECK(moving[1] && moving[2]);
        int32_t y_before = position[1];
        tick(&w);
        CHECK(position[1] == y_before + 1);   // Y is still going
        CHECK(position[0] == w.at_steps[0]);  // X has not moved since
    }

    // the position is the first edge, not where the hysteresis ran out: the confirming steps
    // must not show up in the measurement
    {
        reset();
        Watch w; w.motors = (1 << 0); w.hysteresis = 5; w.arm();
        for (int i = 0; i < 10; i++) tick(&w);      // 10 steps with nothing asserted
        w.asserted = true;
        for (int i = 0; i < 5; i++) tick(&w);       // confirms on the fifth
        CHECK(w.hit);
        CHECK(w.at_steps[0] == 10);                 // where it was when the input first changed
        for (int i = 0; i < 20; i++) tick(&w);
        CHECK(w.at_steps[0] == 10);
    }

    // a glitch that fails to confirm does not leave its position behind
    {
        reset();
        Watch w; w.motors = (1 << 0); w.hysteresis = 5; w.arm();
        for (int i = 0; i < 10; i++) tick(&w);
        w.asserted = true;  for (int i = 0; i < 2; i++) tick(&w);   // a bounce, not enough
        w.asserted = false; for (int i = 0; i < 10; i++) tick(&w);
        CHECK(!w.hit);
        w.asserted = true;  for (int i = 0; i < 5; i++) tick(&w);   // the real edge
        CHECK(w.hit);
        CHECK(w.at_steps[0] == 22);                 // 10 + 2 + 10, not the glitch at 10
    }

    // once hit, it stays hit: a release does not re-arm within the same move
    {
        reset();
        Watch w; w.motors = 1; w.hysteresis = 1; w.arm();
        w.asserted = true;  tick(&w); CHECK(w.hit);
        w.asserted = false; tick(&w); CHECK(w.hit);
        CHECK(!moving[0]);
    }

    // arm() clears it for the next move
    {
        Watch w; w.hit = true; w.count = 7;
        w.arm();
        CHECK(!w.hit && w.count == 0);
    }

    // no watch at all is a plain move
    {
        reset();
        for (int i = 0; i < 10; i++) tick(nullptr);
        CHECK(position[0] == 10 && moving[0]);
    }

    printf(fails == 0 ? "watch: all passed\n" : "watch: %d FAILED\n", fails);
    return fails != 0;
}

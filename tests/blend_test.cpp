// The step rule across a blend window, driven the way the interrupt drives it.
//
// What has to hold, or the machine loses position or the blend is not the curve it claims:
//   - with no blend the pulses are the single-block rule's, to the path step
//   - every block makes exactly its own steps per motor, and the motors end where the polygon ends
//   - the path through a window is the parabola between the two chords, within a step and a half
//   - a motor's pin is set a path step before a pulse the other way
//   - the axis both blocks step every path step goes on stepping every path step
//   - the heading turns evenly through a window, where the polygon turns it at the vertex
//   - windows that touch, one ending where the next starts, stay exact
//
// c++ -std=c++11 -I ../src/libs -I ../src/modules/robot blend_test.cpp
#include "StepMix.h"

#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>

static int fails = 0;
#define CHECK(c) do { if(!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while(0)

Block::Block() {}
uint8_t Block::n_actuators = 3;

static Block chord(int dx, int dy, int dz = 0)
{
    Block b;
    b.steps.fill(0);
    b.steps[0] = abs(dx); b.steps[1] = abs(dy); b.steps[2] = abs(dz);
    b.direction_bits = (dx < 0 ? 1 : 0) | (dy < 0 ? 2 : 0) | (dz < 0 ? 4 : 0);
    b.blend_in = b.blend_out = 0;
    return b;
}

static uint32_t longest(const Block &b)
{
    uint32_t n = 0;
    for (int m = 0; m < 3; m++) if (b.steps[m] > n) n = b.steps[m];
    return n;
}

struct Trace {
    std::vector<int> pos[3];        // motor position after each tick
    std::vector<size_t> mark;       // tick index where each block's path steps end
    int at[3] = {0, 0, 0};
    bool ok = true;
};

// the interrupt's use of the mix: a tick per path step, the next block opened where the lead
// asks for it, the swap at the mark
static Trace play(const std::vector<Block> &blocks)
{
    Trace t;
    StepMix mix;
    mix.n = 3;
    mix.reset();
    auto pulse = [&](StepMix::Player *p) {
        for (int m = 0; m < 3; m++) {
            if (mix.motor(p, m) == StepMix::FIRE) t.at[m] += ((mix.pin_dirs >> m) & 1) ? -1 : 1;
            if (mix.owed[m] > 2 || mix.owed[m] < -2) t.ok = false;   // falling behind
            t.pos[m].push_back(t.at[m]);
        }
    };
    for (size_t i = 0; i < blocks.size(); i++) {
        if (mix.two && mix.first_half) mix.swap_at_mark();
        else {
            mix.start(blocks[i]);
            mix.pin_dirs = blocks[i].direction_bits;   // set with its lead at a block start
        }
        uint32_t n = longest(blocks[i]);
        for (uint32_t k = 0; k < n; k++) {
            StepMix::Player *p = mix.tick();
            if (mix.wants_next()) {
                if (i + 1 >= blocks.size()) { t.ok = false; break; }
                mix.open(blocks[i + 1]);
            }
            pulse(p);
        }
        t.mark.push_back(t.pos[0].size());
    }
    for (int guard = 0; guard < 16 && mix.owing(); guard++) pulse(nullptr);
    if (!mix.idle()) t.ok = false;
    return t;
}

// the single-block rule, as owed_test has it
static std::vector<int> plain(uint32_t steps, uint32_t n)
{
    uint32_t share = Block::share_of(steps, n), acc = 0x80000000UL, count = 0;
    std::vector<int> at;
    for (uint32_t k = 1; k <= n; k++) {
        uint32_t was = acc;
        acc += share;
        if (count < steps && (share == 0 || acc < was)) count++;
        at.push_back((int)count);
    }
    return at;
}

// a window of w path steps on each side of the corner between a and b
static void blend(Block &a, Block &b, uint32_t w)
{
    a.blend_out = b.blend_in = (uint16_t)w;
}

int main()
{
    // 1. no blend: block by block the single-block rule
    {
        std::vector<Block> b = {chord(1000, 371), chord(250, -900), chord(-3, 7), chord(640, 640)};
        Trace t = play(b);
        CHECK(t.ok);
        size_t from = 0;
        int base[3] = {0, 0, 0};
        for (size_t i = 0; i < b.size(); i++) {
            uint32_t n = longest(b[i]);
            for (int m = 0; m < 2; m++) {
                std::vector<int> want = plain(b[i].steps[m], n);
                int sign = ((b[i].direction_bits >> m) & 1) ? -1 : 1;
                for (uint32_t k = 0; k < n; k++) CHECK(t.pos[m][from + k] == base[m] + sign * want[k]);
                base[m] += sign * (int)b[i].steps[m];
            }
            from += n;
        }
        printf("no blend: ok\n");
    }

    // 2. a right-angle corner: exact ends, and the path is the parabola
    {
        const int n = 200;
        std::vector<Block> b = {chord(1000, 0), chord(0, 1000)};
        blend(b[0], b[1], n);
        Trace t = play(b);
        CHECK(t.ok);
        CHECK(t.at[0] == 1000 && t.at[1] == 1000);
        // from A = (1000 - n, 0) over C = (1000, 0) to B = (1000, n): the nearest point of
        // the parabola to every traced point inside the window
        double worst = 0, inside = 0;
        for (size_t k = 1000 - n; k < 1000 + (size_t)n; k++) {
            double x = t.pos[0][k], y = t.pos[1][k], best = 1e9;
            for (int j = 0; j <= 2000; j++) {
                double u = j / 2000.0;
                double px = (1000 - n) * (1 - u) * (1 - u) + 2 * 1000 * u * (1 - u) + 1000 * u * u;
                double py = n * u * u;
                double d = hypot(px - x, py - y);
                if (d < best) best = d;
            }
            if (best > worst) worst = best;
            double corner = hypot(1000 - x, 0 - y);
            if (k == 1000) inside = corner;
        }
        printf("corner: path within %.2f steps of the parabola, "
               "%.1f steps inside the vertex (n/2 sin 45 = %.1f)\n",
               worst, inside, n / 2.0 * sin(M_PI / 4));
        CHECK(worst <= 1.5);
        CHECK(fabs(inside - n / 2.0 * sin(M_PI / 4)) <= 2.0);
    }

    // 3. a motor that reverses through the corner
    {
        std::vector<Block> b = {chord(1000, 60), chord(1000, -45)};
        blend(b[0], b[1], 300);
        Trace t = play(b);
        CHECK(t.ok);
        CHECK(t.at[0] == 2000 && t.at[1] == 15);
        int peak = 0;
        for (int y : t.pos[1]) if (y > peak) peak = y;
        CHECK(peak <= 60 && peak >= 52);   // the corner is cut: the vertex at 60 is not reached
        printf("reversal: ends at %d, turns at %d of 60\n", t.at[1], peak);
    }

    // 4. an arc of chords, every chord blended over half its length on each side: the direction
    //    of travel turns evenly where the polygon turns it once a chord
    {
        std::vector<Block> poly, arc;
        const int chords = 24;
        const double len = 4000;
        double x = 0, y = 0;
        int ix = 0, iy = 0;
        for (int i = 0; i < chords; i++) {
            double a = (i + 0.5) * (M_PI / 2) / chords;
            x += len * cos(a); y += len * sin(a);
            int nx = (int)lround(x), ny = (int)lround(y);
            poly.push_back(chord(nx - ix, ny - iy));
            ix = nx; iy = ny;
        }
        arc = poly;
        for (int i = 0; i + 1 < chords; i++) {
            blend(arc[i], arc[i + 1], std::min(longest(arc[i]), longest(arc[i + 1])) / 2);
        }
        Trace p = play(poly), s = play(arc);
        CHECK(p.ok && s.ok);
        CHECK(s.at[0] == ix && s.at[1] == iy && p.at[0] == ix && p.at[1] == iy);
        // the heading over windows of 400 path steps: the largest turn between neighbours
        auto turn = [](const Trace &t) {
            double worst = 0, last = -9;
            for (size_t k = 400; k + 400 < t.pos[1].size(); k += 400) {
                double h = atan2(t.pos[1][k + 400] - t.pos[1][k], t.pos[0][k + 400] - t.pos[0][k]);
                if (last > -9 && fabs(h - last) > worst) worst = fabs(h - last);
                last = h;
            }
            return worst * 180 / M_PI;
        };
        double tp = turn(p), ts = turn(s);
        printf("arc: largest turn between 400-step windows, polygon %.2f deg, blended %.2f deg "
               "(a chord turns %.2f)\n",
               tp, ts, 90.0 / chords);
        CHECK(ts < 0.35 * tp);
        // inside the polygon by no more than a chord's sagitta
        double worst = 0;
        for (size_t k = 0; k < s.pos[0].size() && k < p.pos[0].size(); k++) {
            double d = hypot(s.pos[0][k] - p.pos[0][k], s.pos[1][k] - p.pos[1][k]);
            if (d > worst) worst = d;
        }
        // X is the longest axis of every chord in the first third: there it pulses every path step
        size_t third = s.mark[chords / 3 - 1], odd = 0;
        for (size_t k = 1; k < third; k++) if (s.pos[0][k] - s.pos[0][k - 1] != 1) odd++;
        printf("arc: %zu of %zu path steps without exactly one X pulse\n", odd, third);
        CHECK(odd == 0);
        double sagitta = len / 8 * (M_PI / 2 / chords);
        printf("arc: blended path within %.1f steps of the polygon's (a chord's sagitta is %.1f)\n",
               worst, sagitta);
        CHECK(worst <= sagitta * 1.5 + 2);
    }

    // 5. an axis move into a diagonal: the path step grows by root two, and the distance made
    //    per path step passes from the one to the other across the window without a jump
    {
        std::vector<Block> b = {chord(2000, 0), chord(1500, 1500)};
        blend(b[0], b[1], 300);
        Trace t = play(b);
        CHECK(t.ok);
        CHECK(t.at[0] == 3500 && t.at[1] == 1500);
        auto dist = [&](size_t from, size_t w) {
            return hypot(t.pos[0][from + w] - t.pos[0][from], t.pos[1][from + w] - t.pos[1][from]) / w;
        };
        size_t m = t.mark[0];
        double early = dist(m - 290, 40), before = dist(m - 45, 40);
        double after = dist(m + 5, 40), late = dist(m + 250, 40);
        printf("unequal steps: %.3f, %.3f | %.3f, %.3f per path step across the window (1 to %.3f)\n",
               early, before, after, late, sqrt(2.0));
        CHECK(fabs(early - 1.0) < 0.06 && fabs(late - sqrt(2.0)) < 0.08);
        CHECK(before > early && after > before - 0.05 && late > after);
        CHECK(fabs(after - before) < 0.12);
    }

    // 6. windows that touch: three short chords, each blended over exactly half
    {
        std::vector<Block> b = {chord(40, 3), chord(40, 9), chord(40, 15), chord(40, 21)};
        for (size_t i = 0; i + 1 < b.size(); i++) { b[i].blend_out = 20; b[i + 1].blend_in = 20; }
        Trace t = play(b);
        CHECK(t.ok);
        CHECK(t.at[0] == 160 && t.at[1] == 48);
        printf("touching windows: ok\n");
    }

    if (fails == 0) printf("all ok\n");
    return fails == 0 ? 0 : 1;
}

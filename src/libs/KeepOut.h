#pragma once

#include <cmath>

// A box in machine coordinates a move may not enter. A NaN bound is open.
struct KeepOut {
    float min[3]{NAN, NAN, NAN};
    float max[3]{NAN, NAN, NAN};

    bool unbounded() const
    {
        for (int i = 0; i < 3; i++) if(!std::isnan(min[i]) || !std::isnan(max[i])) return false;
        return true;
    }

    bool contains(const float p[3]) const
    {
        for (int i = 0; i < 3; i++) {
            if(!std::isnan(min[i]) && p[i] < min[i]) return false;
            if(!std::isnan(max[i]) && p[i] > max[i]) return false;
        }
        return true;
    }

    // the segment from a to b passes through the box
    bool crossed(const float a[3], const float b[3]) const
    {
        float t0 = 0, t1 = 1;
        for (int i = 0; i < 3; i++) {
            float d = b[i] - a[i];
            if(d == 0) {
                if((!std::isnan(min[i]) && a[i] < min[i]) || (!std::isnan(max[i]) && a[i] > max[i])) return false;
                continue;
            }
            float tlo = ((std::isnan(min[i]) ? -INFINITY : min[i]) - a[i]) / d;
            float thi = ((std::isnan(max[i]) ? INFINITY : max[i]) - a[i]) / d;
            t0 = std::fmax(t0, std::fmin(tlo, thi));
            t1 = std::fmin(t1, std::fmax(tlo, thi));
            if(t0 > t1) return false;
        }
        return true;
    }
};

/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "StepCompress.h"
#include "StepStream.h"

#include <math.h>

float StepCompress::timer_hz_= 25000000.0F;
float StepCompress::tolerance_= 0.002F;

void StepCompress::configure(float timer_hz, float tolerance)
{
    timer_hz_= timer_hz;
    tolerance_= tolerance;
}

float StepCompress::interval_at(float from, float to, uint32_t steps, uint32_t k)
{
    if(steps == 0) {
        return timer_hz_;
    }

    float at= (k + 0.5F) / (float)steps;
    float v2= from * from + (to * to - from * from) * at;
    float v= v2 > 1.0F ? sqrtf(v2) : 1.0F;
    return timer_hz_ / v;
}

uint32_t StepCompress::ramp(StepStream &out, float from, float to, uint32_t steps, uint32_t done)
{
    uint32_t i= done;
    while(i < steps) {
        if(out.full()) {
            break;
        }

        float first= interval_at(from, to, steps, i);
        if(i + 1 >= steps) {
            out.push((uint32_t)(first + 0.5F), 1, 0);
            i++;
            break;
        }

        float add= interval_at(from, to, steps, i + 1) - first;
        uint32_t k= 2;
        float want= first + interval_at(from, to, steps, i + 1);

        while(i + k < steps && k < 0xFFFFU) {
            float exact= interval_at(from, to, steps, i + k);
            float allow= exact * tolerance_;
            if(allow < 1.0F) allow= 1.0F;
            if(fabsf(first + k * add - exact) > allow) {
                break;
            }
            want+= exact;
            k++;
        }

        float fit= (want - k * first) / (k * (k - 1) * 0.5F);
        float last= interval_at(from, to, steps, i + k - 1);
        float allow= last * tolerance_;
        if(allow < 1.0F) allow= 1.0F;
        if(fabsf(first + (k - 1) * fit - last) > allow) {
            fit= add;
        }

        float scaled= fit * (1 << StepStream::k_add_shift);
        out.push((uint32_t)(first + 0.5F), k, (int32_t)(scaled < 0 ? scaled - 0.5F : scaled + 0.5F));
        i+= k;
    }
    return i;
}

uint32_t StepCompress::plateau(StepStream &out, float rate, uint32_t steps)
{
    if(rate < 1.0F) {
        return 0;
    }

    uint32_t interval= (uint32_t)(timer_hz_ / rate + 0.5F);
    uint32_t done= 0;
    while(done < steps) {
        if(out.full()) {
            break;
        }
        uint32_t n= steps - done;
        out.push(interval, n, 0);
        done+= n;
    }
    return done;
}

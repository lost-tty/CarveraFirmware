/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

using namespace std;

#include "mri.h"
#include "nuts_bolts.h"
#include "RingBuffer.h"
#include "Gcode.h"
#include "Module.h"
#include "Kernel.h"
#include "Block.h"
#include "Planner.h"
#include "libs/Profile.h"
#include "Conveyor.h"
#include "libs/StepCompress.h"
#include "StepperMotor.h"
#include "checksumm.h"
#include "Robot.h"
#include "ConfigTable.h"

#include <math.h>
#include <algorithm>

#define PLANNER_CONFIG(X) \
    X(float, junction_deviation,   "junction_deviation",   0.01f) \
    X(float, z_junction_deviation, "z_junction_deviation", NAN) \
    X(float, minimum_planner_speed,"minimum_planner_speed",0.0f) \
    X(float, path_tolerance,       "path_tolerance",       0.0f)
CONFIG_STRUCT(PlannerConfig, PLANNER_CONFIG);
CONFIG_KEYS(planner_config_keys, PlannerConfig, PLANNER_CONFIG);
static void planner_config_changed(const ConfigTable::Group *, const void *c)
{
    THEKERNEL->planner.config_load(c);
}
CONFIG_GROUPS(planner_config_groups,
    CFG_GROUP("", planner_config_keys, PlannerConfig, planner_config_changed));



void Planner::init()
{
    memset(this->previous_unit_vec, 0, sizeof this->previous_unit_vec);
    config_load(&ConfigTable::config<PlannerConfig>(planner_config_groups));
}

void Planner::config_load(const void *cfg)
{
    const PlannerConfig &c = *(const PlannerConfig *)cfg;
    this->junction_deviation = c.junction_deviation;
    this->z_junction_deviation = c.z_junction_deviation; // NAN disables it.
    this->minimum_planner_speed = c.minimum_planner_speed;
    this->default_tolerance = c.path_tolerance;
    this->tolerance = this->default_tolerance;
}

// Append a block to the queue, compute it's speed factors
// 2024
bool Planner::append_block( ActuatorCoordinates &actuator_pos, uint8_t n_motors, float rate_mm_s,
                            float distance, float *unit_vec, float acceleration,
                            const float share[3], float s_value, bool cutting, uint32_t mark)
// bool Planner::append_block( ActuatorCoordinates &actuator_pos, uint8_t n_motors, float rate_mm_s, float distance, float *unit_vec, float acceleration, float *s_values, int s_count, bool cutting, unsigned int _line)
{
    PROFILE("append_block");
    acceleration/= StepCompress::k_peak_over_mean;

    // Create ( recycle ) a new block
    Block* block = THECONVEYOR.queue.head_ref();
    block->mark = mark;

    // Direction bits
    bool has_steps = false;

    // 2024
    /*
    int bigaxis = 0;
    int32_t bigsteps = 0;
    */

    block->direction_bits = 0;
    for (size_t i = 0; i < n_motors; i++) {
        int32_t steps = THEROBOT.actuators[i]->steps_to_target(actuator_pos[i]);
        // Update current position
        if(steps != 0) {
            THEROBOT.actuators[i]->update_last_milestones(actuator_pos[i], steps);
            has_steps = true;
        }

        // find direction
        if(steps < 0) block->direction_bits |= 1 << i;
        // save actual steps in block
        block->steps[i] = labs(steps);

        // 2024
        /*
        if( labs(steps) > bigsteps ) {
			bigaxis = i;
			bigsteps = labs(steps);
		}*/
    }

    // sometimes even though there is a detectable movement it turns out there are no steps to be had from such a small move
    if (!has_steps) {
        block->clear();
        // we still return true so the tiny move will still be accumulated and eventually create steps
        return true;
    }

    // info needed by laser
    // 2024
    float power = s_value / THEROBOT.get_max_s_value();
    block->s_value = roundf((power < 0.0F ? 0.0F : power > 1.0F ? 1.0F : power) * (1 << 11));
    block->cutting = cutting;

    /*
	block->move_axis = bigaxis;
	block->s_count = s_count;
	for( int i = 0; i < s_count; i++ ) {
		block->s_values[i] = roundf(s_values[i] * (1<<11)); // 1.11 fixed point
	}
	block->s_value = block->s_values[0];

	block->cutting = cutting;
	*/

    // use default JD
    float junction_deviation = this->junction_deviation;

    // use either regular junction deviation or z specific and see if a primary axis move
    block->primary_axis = true;
    if(block->steps[ALPHA_STEPPER] == 0 && block->steps[BETA_STEPPER] == 0) {
        if(block->steps[GAMMA_STEPPER] != 0) {
            // z only move
            if(!isnan(this->z_junction_deviation)) junction_deviation = this->z_junction_deviation;

        } else {
            // is not a primary axis move
            block->primary_axis= false;
            #if N_PRIMARY_AXIS > 3
                for (int i = 3; i < N_PRIMARY_AXIS; ++i) {
                    if(block->steps[i] != 0){
                        block->primary_axis= true;
                        break;
                    }
                }
            #endif

        }
    }

    block->acceleration = acceleration; // save in block
    memcpy(block->share, share, sizeof(block->share));

    block->millimeters = distance;
    block->nominal_speed = distance > 0.0F ? rate_mm_s : 0.0F; // (mm/s)

    // Compute the acceleration rate for the trapezoid generator. Depending on the slope of the line
    // average travel per step event changes. For a line along one axis the travel per step event
    // is equal to the travel/step in the particular axis. For a 45 degree line the steppers of both
    // axes might step for every step event. Travel per step event is then sqrt(travel_x^2+travel_y^2).

    // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
    // Let a circle be tangent to both previous and current path line segments, where the junction
    // deviation is defined as the distance from the junction to the closest edge of the circle,
    // colinear with the circle center. The circular segment joining the two paths represents the
    // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
    // radius of the circle, defined indirectly by junction deviation. This may be also viewed as
    // path width or max_jerk in the previous grbl version. This approach does not actually deviate
    // from path, but used as a robust way to compute cornering speeds, as it takes into account the
    // nonlinearities of both the junction angle and junction velocity.

    // NOTE however it does not take into account independent axis, in most cartesian X and Y and Z are totally independent
    // and this allows one to stop with little to no decleration in many cases. This is particualrly bad on leadscrew based systems that will skip steps.
    float vmax_junction = minimum_planner_speed; // Set default max junction speed

    // if unit_vec was null then it was not a primary axis move so we skip the junction deviation stuff
    if (unit_vec != nullptr && !THECONVEYOR.is_queue_empty()) {
        Block *prev_block = THECONVEYOR.queue.item_ref(THECONVEYOR.queue.prev(THECONVEYOR.queue.head_i));
        float previous_nominal_speed = prev_block->primary_axis ? prev_block->nominal_speed : 0;

        if (junction_deviation > 0.0F && previous_nominal_speed > 0.0F) {
            // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
            // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
            float cos_theta = - this->previous_unit_vec[X_AXIS] * unit_vec[X_AXIS]
                              - this->previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS]
                              - this->previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS];
            #if N_PRIMARY_AXIS > 3
                for (int i = 3; i < N_PRIMARY_AXIS; ++i) {
                    cos_theta -= this->previous_unit_vec[i] * unit_vec[i];
                }
            #endif

            // Skip and use default max junction speed for 0 degree acute junction.
            if (cos_theta <= 0.9999F) {
                vmax_junction = std::min(previous_nominal_speed, block->nominal_speed);
                // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
                if (cos_theta >= -0.9999F) {
                    // Compute maximum junction velocity based on maximum acceleration and junction deviation
                    float sin_theta_d2 = sqrtf(0.5F * (1.0F - cos_theta)); // Trig half angle identity. Always positive.
                    float a = THEROBOT.path_accel(*block, vmax_junction);
                    float limit = sqrtf(a * junction_deviation * sin_theta_d2
                                        / (1.0F - sin_theta_d2));
                    vmax_junction = std::min(vmax_junction, limit);
                }
            }
        }
    }
    block->max_entry_speed = vmax_junction;

    // Update previous path unit_vector and nominal speed
    if(unit_vec != nullptr) {
        memcpy(previous_unit_vec, unit_vec, sizeof(previous_unit_vec)); // previous_unit_vec[] = unit_vec[]
    } else {
        memset(previous_unit_vec, 0, sizeof(previous_unit_vec));
    }

    // The block can now be used
    block->ready();

    THECONVEYOR.queue_head_block();

    return true;
}

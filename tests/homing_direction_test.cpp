// Homing must drive toward the switch whatever the previous move did.
//
// The observed fault: after a negative jog, $H sends X negative; after a positive jog it
// sends X positive. The step sign comes from the planner, which asks the actuator for
// steps_to_target(), so this drives that arithmetic through the same sequence the machine
// does -- jog, reset_axis_position, homing move -- and asserts the sign.
#include <cstdio>
#include <cmath>

static int fails = 0;
#define CHECK(c) do { if(!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); fails++; } } while(0)

// ---- StepperMotor, the parts that decide a step count ----------------------------------------
struct Motor {
    float   steps_per_mm   = 200.f;
    float   last_milestone_mm    = 0.f;
    int32_t last_milestone_steps = 0;
    int32_t current_position_steps = 0;

    // StepperMotor::change_last_milestone, verbatim
    void change_last_milestone(float mm)
    {
        last_milestone_mm = mm;
        last_milestone_steps = lroundf(last_milestone_mm * steps_per_mm);
        current_position_steps = last_milestone_steps;
    }

    // StepperMotor::update_last_milestones, as the planner calls it once a block is made
    void update_last_milestones(float mm, int32_t steps)
    {
        last_milestone_mm = mm;
        last_milestone_steps += steps;
    }

    // StepperMotor::steps_to_target, verbatim
    int32_t steps_to_target(float target) const
    {
        return lroundf(target * steps_per_mm) - last_milestone_steps;
    }
};

// ---- Robot, the position bookkeeping around a move --------------------------------------------
struct Machine {
    Motor x;
    float machine_position = 0.f;

    // Robot::reset_axis_position for one cartesian axis
    void reset_axis_position(float pos)
    {
        machine_position = pos;
        x.change_last_milestone(pos);
    }

    // Robot::delta_move: target is current position plus the delta, then plan it
    int32_t move(float delta)
    {
        float target = machine_position + delta;
        int32_t steps = x.steps_to_target(target);
        x.update_last_milestones(target, steps);
        machine_position = target;
        return steps;
    }
};

int main()
{
    const float max_travel = 500.f;   // what homing commands, toward max

    // A plain move goes the way it was asked, from anywhere
    {
        Machine m;
        CHECK(m.move(-50.f) < 0);
        CHECK(m.move(+20.f) > 0);
        CHECK(m.move(-20.f) < 0);
    }

    // Homing after a negative jog drives +max_travel from wherever the machine is
    {
        Machine m;
        m.move(-50.f);                       // jog away from the switch
        int32_t steps = m.move(+max_travel); // home() issues this
        printf("after -50 jog: homing steps = %d\n", (int)steps);
        CHECK(steps > 0);                    // must drive toward the switch
    }

    // Homing after a positive jog
    {
        Machine m;
        m.move(+20.f);
        int32_t steps = m.move(+max_travel);
        printf("after +20 jog: homing steps = %d\n", (int)steps);
        CHECK(steps > 0);
    }

    // Two homing cycles in a row must both drive positive
    {
        Machine m;
        CHECK(m.move(+max_travel) > 0);
        m.reset_axis_position(-1.f);         // homed position after the first $H
        CHECK(m.move(+max_travel) > 0);
    }

    // The retract after homing goes the other way, and the slow approach back again
    {
        Machine m;
        m.reset_axis_position(0.f);
        CHECK(m.move(+max_travel) > 0);      // fast approach
        m.reset_axis_position(0.f);          // reset_position_from_current_actuator_position
        CHECK(m.move(-1.f) < 0);             // retract
        CHECK(m.move(+2.f) > 0);             // slow re-approach
    }

    printf(fails == 0 ? "homing direction: all passed\n" : "homing direction: %d FAILED\n", fails);
    return fails != 0;
}

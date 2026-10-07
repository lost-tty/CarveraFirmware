/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include <cmath>
#include <string>
#include "Block.h"
#include "StepTicker.h"
#include "libs/Profile.h"
#include "Conveyor.h"
#include "Gcode.h"
#include "libs/Logging.h"

#include "mri.h"
#include <inttypes.h>

using std::string;
using std::min;

#define STEP_TICKER_FREQUENCY THEKERNEL->step_ticker.get_frequency()

uint8_t Block::n_actuators= 0;

// A block represents a movement, it's length for each stepper motor, and the corresponding acceleration curves.
// It's stacked on a queue, and that queue is then executed in order, to move the motors.
// Most of the accel math is also done in this class
// And GCode objects for use in on_gcode_execute are also help in here

Block::Block()
{
    mark = 0;
    clear();
}

void Block::init(uint8_t n)
{
    n_actuators= n;
}

void Block::clear()
{
    is_ready            = false;

    this->steps.fill(0);
    direction_bits      = 0;
    nominal_speed       = 0.0F;
    millimeters         = 0.0F;
    acceleration        = 100.0F; // we don't want to get divide by zeroes if this is not set
    max_entry_speed     = 0.0F;
    cutting             = false;
    s_value             = 0;

}

uint32_t Block::steps_event_count() const
{
    uint32_t n= 0;
    for(int i = 0; i < n_actuators; ++i) if(steps[i] > n) n= steps[i];
    return n;
}

void Block::debug() const
{
    printk("%p: steps-X:%lu Y:%lu Z:%lu ", this, this->steps[0], this->steps[1], this->steps[2]);
    for (size_t i = E_AXIS; i < n_actuators; ++i) {
        printk("%c:%lu ", 'A' + i-E_AXIS, this->steps[i]);
    }
    printk("(max:%lu) nominal:r%1.4f/s%1.4f mm:%1.4f acc:%1.2f junction:%1.4f primary:%d ready:%d\r\n",
           steps_event_count(), nominal_rate(), this->nominal_speed, this->millimeters,
           this->acceleration, this->max_entry_speed, this->primary_axis, this->is_ready);
}

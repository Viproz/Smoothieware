/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ACTUATORHOMING_MODULE_H
#define ACTUATORHOMING_MODULE_H

#include "libs/Module.h"
#include "libs/Pin.h"

#include <bitset>

class StepperMotor;
class Gcode;

class ActuatorHoming : public Module{
    public:
        ActuatorHoming();
        void on_module_loaded();
        void on_gcode_received(void* argument);

    private:
        void load_config();
        void home(char axes_to_move, Gcode* gcode);
        void process_home_command(Gcode* gcode);
        void on_get_public_data(void* argument);

        float homing_position[3];
        uint8_t homing_order;
        std::bitset<3> home_direction;

        float  retract_mm[3];
        float  fast_rates[3];
        float  slow_rates[3];
        Pin    pins[6];
        struct {
            volatile char status:3;
        };
};

#endif

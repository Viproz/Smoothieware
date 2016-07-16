/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/communication/utils/Gcode.h"
#include "modules/robot/Conveyor.h"
#include "modules/robot/ActuatorCoordinates.h"
#include "ActuatorHoming.h"
#include "libs/nuts_bolts.h"
#include "libs/Pin.h"
#include "libs/StepperMotor.h"
#include "wait_api.h" // mbed.h lib
#include "Robot.h"
#include "Config.h"
#include "SlowTicker.h"
#include "Planner.h"
#include "checksumm.h"
#include "utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"
#include "PublicDataRequest.h"
#include "EndstopsPublicAccess.h"
#include "StreamOutputPool.h"
#include "StepTicker.h"
#include "BaseSolution.h"
#include "SerialMessage.h"
#include "utils.h"

#include <ctype.h>

#define ALPHA_AXIS 0
#define BETA_AXIS  1
#define GAMMA_AXIS 2
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define actuator_homing_module_enable_checksum         CHECKSUM("actuator_homing_enable")

#define alpha_min_endstop_checksum       CHECKSUM("alpha_min_endstop")
#define beta_min_endstop_checksum        CHECKSUM("beta_min_endstop")
#define gamma_min_endstop_checksum       CHECKSUM("gamma_min_endstop")

#define alpha_max_endstop_checksum       CHECKSUM("alpha_max_endstop")
#define beta_max_endstop_checksum        CHECKSUM("beta_max_endstop")
#define gamma_max_endstop_checksum       CHECKSUM("gamma_max_endstop")

// these values are in steps and should be deprecated
#define alpha_fast_homing_rate_checksum  CHECKSUM("alpha_fast_homing_rate")
#define beta_fast_homing_rate_checksum   CHECKSUM("beta_fast_homing_rate")
#define gamma_fast_homing_rate_checksum  CHECKSUM("gamma_fast_homing_rate")

#define alpha_slow_homing_rate_checksum  CHECKSUM("alpha_slow_homing_rate")
#define beta_slow_homing_rate_checksum   CHECKSUM("beta_slow_homing_rate")
#define gamma_slow_homing_rate_checksum  CHECKSUM("gamma_slow_homing_rate")

#define alpha_homing_retract_checksum    CHECKSUM("alpha_homing_retract")
#define beta_homing_retract_checksum     CHECKSUM("beta_homing_retract")
#define gamma_homing_retract_checksum    CHECKSUM("gamma_homing_retract")

// same as above but in user friendly mm/s and mm
#define alpha_fast_homing_rate_mm_checksum  CHECKSUM("alpha_fast_homing_rate_mm_s")
#define beta_fast_homing_rate_mm_checksum   CHECKSUM("beta_fast_homing_rate_mm_s")
#define gamma_fast_homing_rate_mm_checksum  CHECKSUM("gamma_fast_homing_rate_mm_s")

#define alpha_slow_homing_rate_mm_checksum  CHECKSUM("alpha_slow_homing_rate_mm_s")
#define beta_slow_homing_rate_mm_checksum   CHECKSUM("beta_slow_homing_rate_mm_s")
#define gamma_slow_homing_rate_mm_checksum  CHECKSUM("gamma_slow_homing_rate_mm_s")

#define alpha_homing_retract_mm_checksum    CHECKSUM("alpha_homing_retract_mm")
#define beta_homing_retract_mm_checksum     CHECKSUM("beta_homing_retract_mm")
#define gamma_homing_retract_mm_checksum    CHECKSUM("gamma_homing_retract_mm")

#define endstop_debounce_count_checksum  CHECKSUM("endstop_debounce_count")

#define alpha_homing_direction_checksum  CHECKSUM("alpha_homing_direction")
#define beta_homing_direction_checksum   CHECKSUM("beta_homing_direction")
#define gamma_homing_direction_checksum  CHECKSUM("gamma_homing_direction")
#define home_to_max_checksum             CHECKSUM("home_to_max")
#define home_to_min_checksum             CHECKSUM("home_to_min")
#define alpha_min_checksum               CHECKSUM("alpha_min")
#define beta_min_checksum                CHECKSUM("beta_min")
#define gamma_min_checksum               CHECKSUM("gamma_min")

#define alpha_max_checksum               CHECKSUM("alpha_max")
#define beta_max_checksum                CHECKSUM("beta_max")
#define gamma_max_checksum               CHECKSUM("gamma_max")

#define homing_order_checksum            CHECKSUM("homing_order")

#define STEPPER THEKERNEL->robot->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())


// Homing States
enum {
    MOVING_TO_ENDSTOP_FAST, // homing move
    MOVING_BACK,            // homing move
    MOVING_TO_ENDSTOP_SLOW, // homing move
    NOT_HOMING,
    BACK_OFF_HOME,
    MOVE_TO_ORIGIN,
    LIMIT_TRIGGERED
};

ActuatorHoming::ActuatorHoming()
{
    this->status = NOT_HOMING;
}

void ActuatorHoming::on_module_loaded()
{
    // Do not do anything if not enabled
    if ( THEKERNEL->config->value( actuator_homing_module_enable_checksum )->by_default(true)->as_bool() == false ) {
        delete this;
        return;
    }

    register_for_event(ON_GCODE_RECEIVED);
    register_for_event(ON_GET_PUBLIC_DATA);

    // Settings
    this->load_config();
}

// Get config
void ActuatorHoming::load_config()
{
    this->pins[0].from_string( THEKERNEL->config->value(alpha_min_endstop_checksum          )->by_default("nc" )->as_string())->as_input();
    this->pins[1].from_string( THEKERNEL->config->value(beta_min_endstop_checksum           )->by_default("nc" )->as_string())->as_input();
    this->pins[2].from_string( THEKERNEL->config->value(gamma_min_endstop_checksum          )->by_default("nc" )->as_string())->as_input();
    this->pins[3].from_string( THEKERNEL->config->value(alpha_max_endstop_checksum          )->by_default("nc" )->as_string())->as_input();
    this->pins[4].from_string( THEKERNEL->config->value(beta_max_endstop_checksum           )->by_default("nc" )->as_string())->as_input();
    this->pins[5].from_string( THEKERNEL->config->value(gamma_max_endstop_checksum          )->by_default("nc" )->as_string())->as_input();

    // These are the old ones in steps still here for backwards compatibility
    this->fast_rates[0] =  THEKERNEL->config->value(alpha_fast_homing_rate_checksum     )->by_default(4000 )->as_number() / STEPS_PER_MM(0);
    this->fast_rates[1] =  THEKERNEL->config->value(beta_fast_homing_rate_checksum      )->by_default(4000 )->as_number() / STEPS_PER_MM(1);
    this->fast_rates[2] =  THEKERNEL->config->value(gamma_fast_homing_rate_checksum     )->by_default(6400 )->as_number() / STEPS_PER_MM(2);
    this->slow_rates[0] =  THEKERNEL->config->value(alpha_slow_homing_rate_checksum     )->by_default(2000 )->as_number() / STEPS_PER_MM(0);
    this->slow_rates[1] =  THEKERNEL->config->value(beta_slow_homing_rate_checksum      )->by_default(2000 )->as_number() / STEPS_PER_MM(1);
    this->slow_rates[2] =  THEKERNEL->config->value(gamma_slow_homing_rate_checksum     )->by_default(3200 )->as_number() / STEPS_PER_MM(2);
    this->retract_mm[0] =  THEKERNEL->config->value(alpha_homing_retract_checksum       )->by_default(400  )->as_number() / STEPS_PER_MM(0);
    this->retract_mm[1] =  THEKERNEL->config->value(beta_homing_retract_checksum        )->by_default(400  )->as_number() / STEPS_PER_MM(1);
    this->retract_mm[2] =  THEKERNEL->config->value(gamma_homing_retract_checksum       )->by_default(1600 )->as_number() / STEPS_PER_MM(2);

    // newer mm based config values override the old ones, convert to steps/mm and steps, defaults to what was set in the older config settings above
    this->fast_rates[0] = THEKERNEL->config->value(alpha_fast_homing_rate_mm_checksum )->by_default(this->fast_rates[0])->as_number();
    this->fast_rates[1] = THEKERNEL->config->value(beta_fast_homing_rate_mm_checksum  )->by_default(this->fast_rates[1])->as_number();
    this->fast_rates[2] = THEKERNEL->config->value(gamma_fast_homing_rate_mm_checksum )->by_default(this->fast_rates[2])->as_number();
    this->slow_rates[0] = THEKERNEL->config->value(alpha_slow_homing_rate_mm_checksum )->by_default(this->slow_rates[0])->as_number();
    this->slow_rates[1] = THEKERNEL->config->value(beta_slow_homing_rate_mm_checksum  )->by_default(this->slow_rates[1])->as_number();
    this->slow_rates[2] = THEKERNEL->config->value(gamma_slow_homing_rate_mm_checksum )->by_default(this->slow_rates[2])->as_number();
    this->retract_mm[0] = THEKERNEL->config->value(alpha_homing_retract_mm_checksum   )->by_default(this->retract_mm[0])->as_number();
    this->retract_mm[1] = THEKERNEL->config->value(beta_homing_retract_mm_checksum    )->by_default(this->retract_mm[1])->as_number();
    this->retract_mm[2] = THEKERNEL->config->value(gamma_homing_retract_mm_checksum   )->by_default(this->retract_mm[2])->as_number();

    // get homing direction and convert to boolean where true is home to min, and false is home to max
    int home_dir                    = get_checksum(THEKERNEL->config->value(alpha_homing_direction_checksum)->by_default("home_to_min")->as_string());
    this->home_direction[0]         = home_dir != home_to_max_checksum;

    home_dir                        = get_checksum(THEKERNEL->config->value(beta_homing_direction_checksum)->by_default("home_to_min")->as_string());
    this->home_direction[1]         = home_dir != home_to_max_checksum;

    home_dir                        = get_checksum(THEKERNEL->config->value(gamma_homing_direction_checksum)->by_default("home_to_min")->as_string());
    this->home_direction[2]         = home_dir != home_to_max_checksum;

    this->homing_position[0]        =  this->home_direction[0] ? THEKERNEL->config->value(alpha_min_checksum)->by_default(0)->as_number() : THEKERNEL->config->value(alpha_max_checksum)->by_default(200)->as_number();
    this->homing_position[1]        =  this->home_direction[1] ? THEKERNEL->config->value(beta_min_checksum )->by_default(0)->as_number() : THEKERNEL->config->value(beta_max_checksum )->by_default(200)->as_number();
    this->homing_position[2]        =  this->home_direction[2] ? THEKERNEL->config->value(gamma_min_checksum)->by_default(0)->as_number() : THEKERNEL->config->value(gamma_max_checksum)->by_default(200)->as_number();

    // see if an order has been specified, must be three characters, XYZ or YXZ etc
    string order = THEKERNEL->config->value(homing_order_checksum)->by_default("")->as_string();
    this->homing_order = 0;
    if(order.size() == 3) {
        int shift = 0;
        for(auto c : order) {
            uint8_t i = toupper(c) - 'X';
            if(i > 2) { // bad value
                this->homing_order = 0;
                break;
            }
            homing_order |= (i << shift);
            shift += 2;
        }
    }
}

void ActuatorHoming::home(char axes_to_move, Gcode* gcode)
{
    // check if on_halt (eg kill)
    if(THEKERNEL->is_halted()) return;

    //using slowest fast speed of moved axis
    float fastSpeed = 0;
    float stepPermm = 0;
    for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
        if ( ( axes_to_move >> c) & 1 ) {
        	STEPPER[c]->set_direction(this->home_direction[c]);
            if(fastSpeed == 0 || this->fast_rates[c] < fastSpeed)
                fastSpeed = this->fast_rates[c];
            if(stepPermm == 0 || stepPermm > STEPS_PER_MM(c))
                stepPermm = STEPS_PER_MM(c);
        }
    }
    
    // Start moving the axes to the origin
    this->status = MOVING_TO_ENDSTOP_FAST;
    
    float sps = stepPermm * fastSpeed;
    int steps = 10000000 * stepPermm;
    uint32_t delayus= 1000000.0F / sps;
    bool movedAxis = true;
    for(int s = 0 ; s < steps && movedAxis ; s++) {
        if(THEKERNEL->is_halted()) break;
        movedAxis = false;
        
        for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
            if ( ( axes_to_move >> c) & 1 ) {
                if(!this->pins[c + 3*(int)!this->home_direction[c]].get()) {
                    STEPPER[c]->manual_step(this->home_direction[c]);
                    movedAxis = true;
                }
            }
        }
        //Safe delay
        safe_delay_us(delayus);
    }
    
    return;

    //using slowest fast speed of moved axis
    float slowSpeed = 0;
    for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
        if ( ( axes_to_move >> c) & 1 ) {
            if(slowSpeed == 0 || this->slow_rates[c] < slowSpeed)
                slowSpeed = this->slow_rates[c];
        }
    }
    
    // Move back a small distance
    this->status = MOVING_BACK;
    sps = stepPermm * slowSpeed;
    steps = 10000000 * stepPermm;
    delayus= 1000000.0F / sps;
    movedAxis = true;
    for(int s = 0 ; s < steps && movedAxis ; s++) {
        if(THEKERNEL->is_halted()) break;
        movedAxis = false;
        
        for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
            if ( ( axes_to_move >> c) & 1 ) {
                if(s < this->retract_mm[c] * STEPS_PER_MM(c)) {
gcode->stream->printf("Moving2 %d\r\n", c);
                    STEPPER[c]->manual_step(!this->home_direction[c]);
                    movedAxis = true;
                }
            }
        }
        //Safe delay
        safe_delay_us(delayus);
    }

    // Start moving the axes to the origin slowly
    this->status = MOVING_TO_ENDSTOP_SLOW;
    
    sps = stepPermm * slowSpeed;
    steps = 10000000 * stepPermm;
    delayus= 1000000.0F / sps;
    movedAxis = true;
    for(int s = 0 ; s < steps && movedAxis ; s++) {
        if(THEKERNEL->is_halted()) break;
        movedAxis = false;
        
        for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
            if ( ( axes_to_move >> c) & 1 ) {
                if(this->pins[c + this->home_direction[c]].get()) {
gcode->stream->printf("Moving3 %d\r\n", c);
                    STEPPER[c]->manual_step(this->home_direction[c]);
                    movedAxis = true;
                }
            }
        }
        //Safe delay
        safe_delay_us(delayus);
    }
gcode->stream->printf("Done");
    
    this->status = NOT_HOMING;
}

void ActuatorHoming::process_home_command(Gcode* gcode)
{
    // G28 is received, we have homing to do
gcode->stream->printf("Start homing\r\n");
    // First wait for the queue to be empty
    THECONVEYOR->wait_for_idle();

    // do the actual homing
    if(homing_order != 0) {
        // if an order has been specified do it in the specified order
        // homing order is 0b00ccbbaa where aa is 0,1,2 to specify the first axis, bb is the second and cc is the third
        // eg 0b00100001 would be Y X Z, 0b00100100 would be X Y Z
        for (uint8_t m = homing_order; m != 0; m >>= 2) {
            int a = (1 << (m & 0x03)); // axis to move
            home(a, gcode);
gcode->stream->printf("Homing %d \r\n", a);
            // check if on_halt (eg kill)
            if(THEKERNEL->is_halted()) break;
        }
    } else {
        // they all home at the same time
gcode->stream->printf("Homing all\r\n");
        home(7, gcode);
    }

    // check if on_halt (eg kill)
    if(THEKERNEL->is_halted()) {
        if(!THEKERNEL->is_grbl_mode()) {
            THEKERNEL->streams->printf("Homing cycle aborted by kill\n");
        }
        return;
    }
    
    // set the actuator coordinate to homed value
    ActuatorCoordinates real_actuator_position = {this->homing_position[0], this->homing_position[1], this->homing_position[2]};
    THEKERNEL->robot->reset_actuator_position(real_actuator_position);

}

// Start homing sequences by response to GCode commands
void ActuatorHoming::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if ( gcode->has_g && gcode->g == 28) {
        process_home_command(gcode);
    }
}

void ActuatorHoming::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(pdr->second_element_is(get_homing_status_checksum)) {
        bool *homing = static_cast<bool *>(pdr->get_data_ptr());
        *homing = this->status != NOT_HOMING;
        pdr->set_taken();
    }
}

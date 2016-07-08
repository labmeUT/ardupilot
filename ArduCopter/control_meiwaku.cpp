/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_meiwaku.pde - init and run calls for meiwaku flight mode
 */

// meiwaku_init - initialise meiwaku controller
bool Copter::meiwaku_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors.armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) && (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);

    meiwaku_timer = millis();
    meiwaku_count = 0;

    return true;
}

// meiwaku_run - runs the main meiwaku controller
// should be called at 100hz or more
void Copter::meiwaku_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() || ap.throttle_zero || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    /* Meiwaku Mode Core : input changes in every 10 seconds */
	::printf("meiwakuda\n");
    int16_t pilot_input[4] = {}; /*input 1=roll 2=pitch 3=yawrate 4=throttle*/

    meiwaku_timer = millis();
    uint32_t time_now = millis();

    /* タイマーが10秒以上の時 */
    if( time_now - meiwaku_timer > 10000){
        cliSerial->printf("Meiwaku 10 Sec\n");
        meiwaku_count++;
        if(meiwaku_count > 3){
            meiwaku_count = 0;
        }
        meiwaku_timer = millis();
    }
	
    for(uint8_t i=0; i<4; i++){
        uint8_t channel = 0;
        channel = meiwaku_count+i;
        if(channel>3){
            channel = channel - 4;
        }
        switch(i){
        case 0:
            pilot_input[channel] = channel_roll->get_control_in();
            break;
        case 1:
            pilot_input[channel] = channel_pitch->get_control_in();
        	break;
        case 2:
            pilot_input[channel] = channel_yaw->get_control_in();
        	break;
        case 3:
            pilot_input[channel] = channel_throttle->get_control_in();
        	break;
        }
    }

    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(pilot_input[0], pilot_input[1], target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(pilot_input[2]);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(pilot_input[3]);

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}

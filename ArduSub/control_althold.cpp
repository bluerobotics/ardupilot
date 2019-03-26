#include "Sub.h"


/*
 * control_althold.pde - init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Sub::althold_init()
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    if(!control_check_barometer()) {
        return false;
    }
#endif

    // initialize vertical speeds and leash lengths
    // sets the maximum speed up and down returned by position controller
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Sub::althold_run()
{
    uint32_t tnow = AP_HAL::millis();

    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(motors.get_throttle_hover());
        last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // call attitude controller
    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        last_pilot_heading = ahrs.yaw_sensor;
        last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0; // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
            last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute absolute bearing
            attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, last_pilot_heading, true, get_smoothing_gain());
        }
    }

    position_control.set_target_velocity(Vector3f { 0, 0, 0});
        //position_control.set_target_position(Vector3f { 0, 0, barometer.get_altitude() + (channel_throttle->norm_input()-0.5)*10 });
    //printf("target: %f current: %f output: %f\r\n", position_control.get_target_z(), barometer.get_altitude(), position_control.get_output_z());
    position_control.update();
    motors.set_throttle((position_control.get_output_command().z+1.0f)/2.0f + position_control.get_velocity_ff(channel_throttle->norm_input()-0.5));
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}

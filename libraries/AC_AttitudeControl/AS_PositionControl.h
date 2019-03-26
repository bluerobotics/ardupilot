
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <AP_InertialNav/AP_InertialNav.h>
//#include <AP_Baro/AP_Baro.h>
#include <stdio.h>

class AS_PositionControl
{

public:
    AS_PositionControl(AP_InertialNav_NavEKF& inertial_nav) : inertial_nav_(inertial_nav)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    void set_target_position(Vector3f target) { target_position = target; }
    void set_target_velocity(Vector3f target) { target_velocity = target; }

    Vector3f get_target_position() { return target_position; }
    Vector3f get_target_velocity() { return target_velocity; }

    Vector3f get_error_position() { return target_position - get_current_position(); }
    Vector3f get_error_velocity() { return target_velocity - get_current_velocity(); }
    float get_velocity_ff(float desired_rate) { return pid_velocity_z.get_ff(desired_rate); }

    // Return 3D posiiton, altitude in meters
    Vector3f get_current_position() { return Vector3f { 0, 0, inertial_nav_.get_altitude()/100 }; }
    // Return 3D velocity, vertical in meters per second
    Vector3f get_current_velocity() { return inertial_nav_.get_velocity()/100;};

    void update()
    {
        update_position_controller();
        update_velocity_controller();
        //update_acceleration_controller();
        printf("target_pos: %f, current_pos: %f, pos_error: %f, target_speed: %f, current speed: %f, command.z: %f\n", target_position  .z, get_current_position().z, get_error_position().z , target_velocity.z, get_current_velocity().z, output_command.z);

    }

    Vector3f get_output_command() { return output_command; }

    // parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    AC_PID pid_position_x { 1.0f, 0, 0, 100.0f, 100.0f, 0.01f };
    AC_PID pid_velocity_x { 1.0f, 0, 0, 100.0f, 100.0f, 0.01f };

    AC_PID pid_position_y { 1.0f, 0, 0, 100.0f, 100.0f, 0.01f };
    AC_PID pid_velocity_y { 1.0f, 0, 0, 100.0f, 100.0f, 0.01f };

    AC_PID pid_position_z { 1.0f, 0.0f, 0.0f, 1.0f, 100.0f, 0.01f };
    AC_PID pid_velocity_z { 25.0f, 50.0f, 0.0f, 600.0f, 100.0f, 0.01f };

    // meters!
    Vector3f target_position = { 0, 0, 0 };
    Vector3f target_velocity = { 0, 0, 0 };
    Vector3f output_command = { 0, 0, 0 };

    const AP_InertialNav_NavEKF& inertial_nav_;
    bool _hold_pos = false;

    void update_position_controller()
    {
        pid_position_z.set_input_filter_all(-get_error_position().z);
        target_velocity.z = -pid_position_z.get_pid();
    }

    void update_velocity_controller()
    {
        pid_velocity_z.set_input_filter_all(get_error_velocity().z);
        output_command.z = pid_velocity_z.get_pid();
    }
};

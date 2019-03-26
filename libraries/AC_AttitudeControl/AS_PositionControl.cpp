#include "AS_PositionControl.h"

// parameters
const AP_Param::GroupInfo AS_PositionControl::var_info[] = {
    AP_SUBGROUPINFO(pid_position_x, "POS_X_", 0, AS_PositionControl, AC_PID),
    AP_SUBGROUPINFO(pid_position_y, "POS_Y_", 1, AS_PositionControl, AC_PID),
    AP_SUBGROUPINFO(pid_position_z, "POS_Z_", 2, AS_PositionControl, AC_PID),
    AP_SUBGROUPINFO(pid_velocity_x, "VEL_X_", 3, AS_PositionControl, AC_PID),
    AP_SUBGROUPINFO(pid_velocity_y, "VEL_Y_", 4, AS_PositionControl, AC_PID),
    AP_SUBGROUPINFO(pid_velocity_z, "VEL_Z_", 5, AS_PositionControl, AC_PID),
    AP_GROUPEND
};
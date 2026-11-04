#include "motor_POS.h"
#include "common.h"

void motor_pos_update(void)
{
    /* Use the calibrated mechanical angle directly, without filtering. */
    pm.pos.mechanical_angle_rad_fdb = pm.foc.mechanical_angle_rad;
}

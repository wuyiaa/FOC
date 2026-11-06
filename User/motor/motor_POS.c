#include "motor_POS.h"
#include "common.h"

void motor_pos_update(void)
{
    /* Use the calibrated mechanical angle directly, without filtering. */
    pm.pos.mechanical_angle_rad_fdb = pm.foc.mechanical_angle_rad;
    pm.pos.mechanical_angle_deg_fdb =
        pm.pos.mechanical_angle_rad_fdb * (360.0f / TWO_PI);

    /* Wrap the degree display only; keep the radian control feedback intact. */
    while (pm.pos.mechanical_angle_deg_fdb < 0.0f)
    {
        pm.pos.mechanical_angle_deg_fdb += 360.0f;
    }
    while (pm.pos.mechanical_angle_deg_fdb >= 360.0f)
    {
        pm.pos.mechanical_angle_deg_fdb -= 360.0f;
    }
}

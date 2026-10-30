#include "phase_auto_calibration.h"
#include "encoder_zero_calibration.h"

#include <stddef.h>

#if (FOC_PHASE_AUTO_CALIBRATION_MODE != 0U)
#define MOTOR_CAL_STATE_ALIGN    0U
#define MOTOR_CAL_STATE_DRIVE    1U
#define MOTOR_CAL_STATE_DONE     2U
#define MOTOR_CAL_STATE_FAILED   3U
#define MOTOR_CAL_STATE_FINAL_ALIGN  4U

static void motor_calibration_fail(void)
{
    calibration.state = MOTOR_CAL_STATE_FAILED;
    calibration.failed = 1U;
    encoder_zero_calibration_requested = 0U;
    foc_pwm_output_disable();
}
#endif

#if (FOC_PHASE_AUTO_CALIBRATION_MODE != 0U)
void foc_phase_auto_calibration_step(pmsm_t *pm, float v_dc, float v_min)
{
    if (pm == NULL)
    {
        return;
    }

    if ((pm->encoder.fault != 0U) ||
        (pm->foc.current_limit_fault != 0U))
    {
        motor_calibration_fail();
        return;
    }

    if ((calibration.state == MOTOR_CAL_STATE_DONE) ||
        (calibration.state == MOTOR_CAL_STATE_FAILED))
    {
        foc_pwm_output_disable();
        return;
    }

    if (foc_pwm_output_is_enabled() == 0U)
    {
        if ((calibration.started != 0U) ||
            (calibration.drive_start_cycle != 0U))
        {
            motor_calibration_fail();
        }
        return;
    }

    /* Alignment waits for a valid encoder sample before starting. */
    if ((pm->encoder_foc_sample.valid == 0U) &&
        (calibration.state == MOTOR_CAL_STATE_DRIVE))
    {
        return;
    }

    if (calibration.state == MOTOR_CAL_STATE_DRIVE)
    {
        int32_t delta;
        int32_t physical_delta;

        pm->foc.Ud_ref = 0.0f;
        pm->foc.Uq_ref = MOTOR_CALIBRATION_UQ_V;
        inverse_park_transform(pm);
        svpwm_calc(pm, v_dc, v_min);

        /* Ignore the initial shift from the aligned d-axis position. */
        if ((calibration.drive_observation_started == 0U) &&
            (DWT_GetElapsedCycles(calibration.drive_start_cycle) >=
             ((DWT_CPU_FREQ_HZ / 1000U) *
              MOTOR_CALIBRATION_SETTLE_TIME_MS)))
        {
            calibration.drive_observation_started = 1U;
            calibration.drive_last_raw = pm->encoder_foc_sample.raw;
            calibration.last_sample_sequence = pm->encoder_foc_sample.sequence;
            calibration.drive_delta_counts = 0;
        }

        if ((calibration.drive_observation_started != 0U) &&
            (pm->encoder_foc_sample.sequence != calibration.last_sample_sequence))
        {
            delta = (int32_t)pm->encoder_foc_sample.raw -
                    (int32_t)calibration.drive_last_raw;
            if (delta > (int32_t)((AS5047P_DATA_MASK + 1U) / 2U))
            {
                delta -= (int32_t)(AS5047P_DATA_MASK + 1U);
            }
            else if (delta < -(int32_t)((AS5047P_DATA_MASK + 1U) / 2U))
            {
                delta += (int32_t)(AS5047P_DATA_MASK + 1U);
            }
            calibration.drive_delta_counts += delta;
            calibration.drive_last_raw = pm->encoder_foc_sample.raw;
            calibration.last_sample_sequence = pm->encoder_foc_sample.sequence;
        }

        if (DWT_GetElapsedCycles(calibration.drive_start_cycle) <
            ((DWT_CPU_FREQ_HZ / 1000U) * MOTOR_CALIBRATION_DRIVE_TIME_MS))
        {
            return;
        }

        physical_delta = (int32_t)((float)calibration.drive_delta_counts *
                                    MOTOR_SPEED_DIRECTION);
        if ((physical_delta > -(int32_t)MOTOR_CALIBRATION_MIN_MOVE_COUNTS) &&
            (physical_delta < (int32_t)MOTOR_CALIBRATION_MIN_MOVE_COUNTS))
        {
            /* A stationary rotor means the control angle direction is wrong. */
            if (calibration.direction_retried != 0U)
            {
                motor_calibration_fail();
                return;
            }
            calibration.electrical_direction =
                (int8_t)-calibration.electrical_direction;
            calibration.direction_retried = 1U;
            calibration.drive_start_cycle = DWT->CYCCNT;
            calibration.drive_observation_started = 0U;
            calibration.drive_last_raw = pm->encoder_foc_sample.raw;
            calibration.drive_delta_counts = 0;
            return;
        }

        if (physical_delta > 0)
        {
            /* Keep the confirmed phase/direction and refresh zero once more. */
            calibration.current_polarity_required = 1U;
            calibration.state = MOTOR_CAL_STATE_FINAL_ALIGN;
            calibration.started = 0U;
            calibration.sampling = 0U;
            encoder_zero_calibration_requested = 1U;
            pm->foc.u_alpha = 0.0f;
            pm->foc.u_beta = 0.0f;
            svpwm_calc(pm, v_dc, v_min);
            return;
        }

        if (calibration.phase_retried != 0U)
        {
            motor_calibration_fail();
            return;
        }

        /* A clockwise result requires a true phase reversal and a new zero. */
        calibration.phase_reversed ^= 1U;
        calibration.electrical_direction =
            (int8_t)-calibration.electrical_direction;
        calibration.phase_retried = 1U;
        calibration.direction_retried = 0U;
        calibration.state = MOTOR_CAL_STATE_ALIGN;
        calibration.started = 0U;
        calibration.sampling = 0U;
        pm->foc.u_alpha = 0.0f;
        pm->foc.u_beta = 0.0f;
        svpwm_calc(pm, v_dc, v_min);
        return;
    }

    if (calibration.started == 0U)
    {
        encoder_zero_calibration_requested = 1U;
    }
    foc_encoder_zero_calibration_step(pm, v_dc, v_min);
    if (calibration.failed != 0U)
    {
        motor_calibration_fail();
        return;
    }
    if (encoder_zero_calibration_requested == 0U)
    {
        if (calibration.state == MOTOR_CAL_STATE_FINAL_ALIGN)
        {
            foc_pwm_output_disable();
            calibration.state = MOTOR_CAL_STATE_DONE;
            calibration.done = 1U;
            return;
        }
        calibration.state = MOTOR_CAL_STATE_DRIVE;
        calibration.drive_start_cycle = DWT->CYCCNT;
        calibration.drive_observation_started = 0U;
        calibration.drive_last_raw = pm->encoder_foc_sample.raw;
        calibration.drive_delta_counts = 0;
    }
}
#endif

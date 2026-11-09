#include "eccentricity_identification.h"
#include "encoder_zero_calibration.h"
#include "flash_save.h"
#include "pid.h"

#include <math.h>
#include <string.h>

float theta_e;

foc_open_loop_angle_measure_t foc_open_loop_angle_measure;

static void foc_open_loop_angle_measure_update(const pmsm_t *pm)
{
    static float previous_mechanical_angle_rad;
    static uint8_t initialized;
    const float angle_rad = pm->foc.mechanical_angle_rad;
    float delta_rad;

    if (initialized == 0U)
    {
        /* Pick the initial turn near zero without removing the actual offset. */
        foc_open_loop_angle_measure.mechanical_angle_rad_fdb = angle_rad;
        while (foc_open_loop_angle_measure.mechanical_angle_rad_fdb >= 0.5f * TWO_PI)
        {
            foc_open_loop_angle_measure.mechanical_angle_rad_fdb -= TWO_PI;
        }
        while (foc_open_loop_angle_measure.mechanical_angle_rad_fdb < -0.5f * TWO_PI)
        {
            foc_open_loop_angle_measure.mechanical_angle_rad_fdb += TWO_PI;
        }
        initialized = 1U;
    }
    else
    {
        delta_rad = angle_rad - previous_mechanical_angle_rad;
        if (delta_rad >= 0.5f * TWO_PI)
        {
            delta_rad -= TWO_PI;
        }
        else if (delta_rad < -0.5f * TWO_PI)
        {
            delta_rad += TWO_PI;
        }
        foc_open_loop_angle_measure.mechanical_angle_rad_fdb += delta_rad;
    }
    previous_mechanical_angle_rad = angle_rad;

    foc_open_loop_angle_measure.mechanical_angle_rad_ref =
        theta_e / (float)pm->motor.pn;
    foc_open_loop_angle_measure.mechanical_angle_rad_error =
        foc_open_loop_angle_measure.mechanical_angle_rad_ref -
        foc_open_loop_angle_measure.mechanical_angle_rad_fdb;
    foc_open_loop_angle_measure.mechanical_angle_deg_ref =
        foc_open_loop_angle_measure.mechanical_angle_rad_ref * (360.0f / TWO_PI);
    foc_open_loop_angle_measure.mechanical_angle_deg_fdb =
        foc_open_loop_angle_measure.mechanical_angle_rad_fdb * (360.0f / TWO_PI);
    foc_open_loop_angle_measure.mechanical_angle_deg_error =
        foc_open_loop_angle_measure.mechanical_angle_rad_error * (360.0f / TWO_PI);
    foc_open_loop_angle_measure.correction_deg =
        eccentricity_compensation_lookup_deg(angle_rad * (360.0f / TWO_PI));
    foc_open_loop_angle_measure.mechanical_angle_deg_fdb_ecc =
        foc_open_loop_angle_measure.mechanical_angle_deg_fdb +
        foc_open_loop_angle_measure.correction_deg;
    foc_open_loop_angle_measure.mechanical_angle_deg_error_ecc =
        foc_open_loop_angle_measure.mechanical_angle_deg_ref -
        foc_open_loop_angle_measure.mechanical_angle_deg_fdb_ecc;
}


eccentricity_measure_t eccentricity_measure;
static int8_t vf_sample_direction;
static uint8_t vf_complete;

void eccentricity_VF(pmsm_t *pm)
{
    static uint8_t temp, start = 0U;
    static uint32_t cnt = 0U;
    const float previous_theta_e = theta_e;

    vf_sample_direction = 0;
	
		pm->foc.id_ref = 0.5f;
		pm->foc.iq_ref = 0.0f;//foc_speed_pid.output 

    /* Freeze the ramp and startup delay while MOS outputs are off. */
    if (foc_pwm_output_is_enabled() != 0U)
    {
        cnt++;
        if (cnt >= 20000U)
        {
            start = 1U;
            cnt = 0U;
        }
        if (start == 1U)
        {
            if (temp == 0U)
            {
                theta_e += 50.0f * PWM_PERIOD_S;
            }
            //if (theta_e >= TWO_PI)
            //{
            //    theta_e -= TWO_PI;
            //}
            if (theta_e >= TWO_PI * 11.0f)
            {
                temp = 1U;
            }
            if (temp == 1U)
            {
                theta_e -= 50.0f * PWM_PERIOD_S;
                if (theta_e <= 0.0f)
                {
                    //temp = 0U;
                    theta_e = 0.0f;
                    vf_complete = 1U;
                }
            }
        }
        if (theta_e > previous_theta_e)
        {
            vf_sample_direction = 1;
        }
        else if (theta_e < previous_theta_e)
        {
            vf_sample_direction = -1;
        }
    }
    pm->foc.electrical_angle_rad = theta_e;
    pm->foc.electrical_angle_rad_camp = theta_e;
    foc_open_loop_angle_measure_update(pm);
		foc_current_pid_update(pm, PWM_PERIOD_S, 0U);
}

/* Match each reverse point to the forward curve by linear interpolation.
   No angle-bin averaging or interpolation of an assumed friction model. */
static float paired_sum_deg;

static void eccentricity_error_sample(float angle_deg, float error_deg,
                                     int8_t direction)
{
    static uint16_t reverse_lower_index;
    uint16_t index;
    float fraction;
    float forward_error;
    float paired_error;

    if (direction > 0)
    {
        index = eccentricity_measure.forward_sample_count;
        if (index < ECCENTRICITY_MAX_SAMPLES)
        {
            eccentricity_measure.curve.forward.angle_deg[index] = angle_deg;
            eccentricity_measure.curve.forward.error_deg[index] = error_deg;
            eccentricity_measure.forward_sample_count++;
        }
        eccentricity_measure.stage = 1U;
        return;
    }

    /* Only interpolate inside the measured forward range, no extrapolation. */
    if ((eccentricity_measure.forward_sample_count < 2U) ||
        (angle_deg < eccentricity_measure.curve.forward.angle_deg[0]) ||
        (angle_deg > eccentricity_measure.curve.forward.angle_deg[
            eccentricity_measure.forward_sample_count - 1U]) ||
        (eccentricity_measure.paired_sample_count >= ECCENTRICITY_MAX_SAMPLES))
    {
        return;
    }
    if (eccentricity_measure.stage != 2U)
    {
        reverse_lower_index = eccentricity_measure.forward_sample_count - 2U;
        eccentricity_measure.stage = 2U;
    }
    /* Reverse angles decrease, so the cursor only walks toward zero. */
    while ((reverse_lower_index > 0U) &&
           (angle_deg < eccentricity_measure.curve.forward.angle_deg[reverse_lower_index]))
    {
        reverse_lower_index--;
    }
    index = reverse_lower_index;
    fraction = (angle_deg - eccentricity_measure.curve.forward.angle_deg[index]) /
        (eccentricity_measure.curve.forward.angle_deg[index + 1U] -
         eccentricity_measure.curve.forward.angle_deg[index]);
    forward_error = eccentricity_measure.curve.forward.error_deg[index] + fraction *
        (eccentricity_measure.curve.forward.error_deg[index + 1U] -
         eccentricity_measure.curve.forward.error_deg[index]);
    paired_error = 0.5f * (forward_error + error_deg);

    index = eccentricity_measure.paired_sample_count;
    eccentricity_measure.paired_error_deg[index] = paired_error;
    eccentricity_measure.paired_angle_phase[index] =
        (uint16_t)(angle_deg * (65535.0f / 360.0f) + 0.5f);
    eccentricity_measure.paired_sample_count++;
    paired_sum_deg += paired_error;
    /* Publish only after all paired samples have had offset_b_deg removed. */
}

static float eccentricity_paired_angle_deg_get(uint16_t index)
{
    return (float)eccentricity_measure.paired_angle_phase[index] *
        (360.0f / 65535.0f);
}

/* Work on one source/output point per callback. The centered cyclic window
   removes the ordinary moving-average phase delay and wraps at the seam.
   It rejects spatial orders p, 2p, ...; other cogging orders can remain. */
static void eccentricity_fir_process(const pmsm_t *pm)
{
    static uint16_t resample_index;
    static uint16_t source_index;
    static uint16_t initialize_index;
    static uint16_t filter_index;
    static float window_sum;
    uint16_t taps;
    uint16_t count;
    uint16_t half;
    uint16_t index;
    uint16_t incoming;
    uint16_t outgoing;
    float angle;
    float upper_angle;
    float lower_angle;
    float upper_error;
    float lower_error;
    float fraction;

    if (eccentricity_measure.stage == 4U)
    {
        if (eccentricity_measure.fir_sample_count == 0U)
        {
            if ((pm->motor.pn == 0U) ||
                (eccentricity_measure.paired_sample_count < 2U) ||
                (eccentricity_measure.paired_sample_count < pm->motor.pn))
            {
                return;
            }
            taps = eccentricity_measure.paired_sample_count / pm->motor.pn;
            /* Odd M permits exact centering at an existing sample position. */
            if ((taps & 1U) == 0U)
            {
                taps--;
            }
            eccentricity_measure.fir_window_samples = taps;
            eccentricity_measure.fir_sample_count = taps * pm->motor.pn;
            eccentricity_measure.fir_window_deg = 360.0f / (float)pm->motor.pn;
        }
        count = eccentricity_measure.fir_sample_count;
        angle = (float)(count - 1U - resample_index) * (360.0f / (float)count);
        upper_angle = eccentricity_paired_angle_deg_get(0U);
        lower_angle = eccentricity_paired_angle_deg_get(
            eccentricity_measure.paired_sample_count - 1U);

        /* Interpolate across the periodic seam instead of padding zeros. */
        if (angle > upper_angle)
        {
            upper_angle = lower_angle + 360.0f;
            lower_angle = eccentricity_paired_angle_deg_get(0U);
            upper_error = eccentricity_measure.paired_error_deg[
                eccentricity_measure.paired_sample_count - 1U];
            lower_error = eccentricity_measure.paired_error_deg[0];
        }
        else if (angle <= lower_angle)
        {
            upper_angle = lower_angle;
            lower_angle = eccentricity_paired_angle_deg_get(0U) - 360.0f;
            upper_error = eccentricity_measure.paired_error_deg[
                eccentricity_measure.paired_sample_count - 1U];
            lower_error = eccentricity_measure.paired_error_deg[0];
        }
        else
        {
            while ((source_index + 1U < eccentricity_measure.paired_sample_count - 1U) &&
                   (angle < eccentricity_paired_angle_deg_get(source_index + 1U)))
            {
                source_index++;
            }
            upper_angle = eccentricity_paired_angle_deg_get(source_index);
            lower_angle = eccentricity_paired_angle_deg_get(source_index + 1U);
            upper_error = eccentricity_measure.paired_error_deg[source_index];
            lower_error = eccentricity_measure.paired_error_deg[source_index + 1U];
        }
        fraction = (angle - lower_angle) / (upper_angle - lower_angle);
        eccentricity_measure.curve.fir.uniform_error_deg[resample_index] =
            lower_error + fraction * (upper_error - lower_error);
        resample_index++;
        if (resample_index >= count)
        {
            eccentricity_measure.stage = 5U;
        }
        return;
    }

    count = eccentricity_measure.fir_sample_count;
    taps = eccentricity_measure.fir_window_samples;
    half = taps / 2U;
    if (eccentricity_measure.stage == 5U)
    {
        index = initialize_index + count - half;
        if (index >= count)
        {
            index -= count;
        }
        window_sum += eccentricity_measure.curve.fir.uniform_error_deg[index];
        initialize_index++;
        if (initialize_index >= taps)
        {
            eccentricity_measure.stage = 6U;
        }
        return;
    }

    eccentricity_measure.curve.fir.filtered_error_deg[filter_index] =
        window_sum / (float)taps;
    incoming = filter_index + half + 1U;
    if (incoming >= count)
    {
        incoming -= count;
    }
    outgoing = filter_index + count - half;
    if (outgoing >= count)
    {
        outgoing -= count;
    }
    window_sum += eccentricity_measure.curve.fir.uniform_error_deg[incoming] -
        eccentricity_measure.curve.fir.uniform_error_deg[outgoing];
    filter_index++;
    if (filter_index >= count)
    {
        eccentricity_measure.paired_error_deg_display =
            eccentricity_measure.curve.fir.filtered_error_deg[0];
        eccentricity_measure.paired_error_deg_before_fir_display =
            eccentricity_measure.curve.fir.uniform_error_deg[0];
        eccentricity_measure.paired_angle_deg_display =
            (float)(count - 1U) * (360.0f / (float)count);
        eccentricity_measure.paired_electrical_angle_deg_display =
            eccentricity_measure.paired_angle_deg_display * (float)pm->motor.pn;
        eccentricity_measure.stage = 7U;
        eccentricity_measure.done = 1U;
    }
}

static void eccentricity_error_process(const pmsm_t *pm)
{
    static int8_t record_direction;
    static uint8_t record_count;
    static uint16_t calculate_index;
    static uint16_t display_index;
    static uint8_t display_count;

    if (eccentricity_measure.done != 0U)
    {
        /* Replay at the same 200 us point interval used for recording. */
        display_count++;
        if (display_count >= ECCENTRICITY_RECORD_INTERVAL)
        {
            display_count = 0U;
            display_index++;
            if (display_index >= eccentricity_measure.fir_sample_count)
            {
                display_index = 0U;
            }
        }
        eccentricity_measure.paired_error_deg_display =
            eccentricity_measure.curve.fir.filtered_error_deg[display_index];
        eccentricity_measure.paired_error_deg_before_fir_display =
            eccentricity_measure.curve.fir.uniform_error_deg[display_index];
        eccentricity_measure.paired_angle_deg_display =
            (float)(eccentricity_measure.fir_sample_count - 1U - display_index) *
            (360.0f / (float)eccentricity_measure.fir_sample_count);
        eccentricity_measure.paired_electrical_angle_deg_display =
            eccentricity_measure.paired_angle_deg_display * (float)pm->motor.pn;
        return;
    }

    if (eccentricity_measure.stage >= 4U)
    {
        eccentricity_fir_process(pm);
        return;
    }

    if (eccentricity_measure.stage != 3U)
    {
        if (vf_sample_direction != 0)
        {
            if (record_direction != vf_sample_direction)
            {
                record_direction = vf_sample_direction;
                record_count = 0U;
            }
            if (record_count == 0U)
            {
                eccentricity_error_sample(
                    foc_open_loop_angle_measure.mechanical_angle_deg_ref,
                    foc_open_loop_angle_measure.mechanical_angle_deg_error,
                    vf_sample_direction);
            }
            record_count++;
            if (record_count >= ECCENTRICITY_RECORD_INTERVAL)
            {
                record_count = 0U;
            }
        }
        if (vf_complete == 0U)
        {
            return;
        }
        eccentricity_measure.stage = 3U;
        if (eccentricity_measure.paired_sample_count != 0U)
        {
            eccentricity_measure.offset_b_deg = paired_sum_deg /
                (float)eccentricity_measure.paired_sample_count;
        }
    }

    /* Subtract b one point per callback to keep callback work bounded. */
    if (calculate_index < eccentricity_measure.paired_sample_count)
    {
        eccentricity_measure.paired_error_deg[calculate_index] -=
            eccentricity_measure.offset_b_deg;
        calculate_index++;
    }
    if (calculate_index >= eccentricity_measure.paired_sample_count)
    {
        eccentricity_measure.stage = 4U;
    }
}

eccentricity_compensation_t eccentricity_compensation;

static float eccentricity_fir_error_get(uint16_t ascending_index)
{
    if (ascending_index == eccentricity_measure.fir_sample_count)
    {
        ascending_index = 0U;
    }
    return eccentricity_measure.curve.fir.filtered_error_deg[
        eccentricity_measure.fir_sample_count - 1U - ascending_index];
}

static void eccentricity_lut_build(void)
{
    static uint16_t lut_index;
    uint16_t lower;
    uint16_t upper;
    uint16_t middle;
    const uint16_t count = eccentricity_measure.fir_sample_count;
    float step;
    float origin;
    float target;
    float lower_angle;
    float upper_angle;
    float lower_error;
    float upper_error;
    float fraction;

    if ((eccentricity_measure.done == 0U) ||
        (lut_index >= ECCENTRICITY_LUT_SIZE) || (count < 2U))
    {
        return;
    }
    if (lut_index == 0U)
    {
        eccentricity_compensation.ready = 0U;
        eccentricity_compensation.flash_loaded = 0U;
        eccentricity_compensation.flash_saved = 0U;
        eccentricity_compensation.mechanical_zero_raw = calibration.mechanical_zero_raw;
        eccentricity_compensation.electrical_direction = calibration.electrical_direction;
    }
    step = 360.0f / (float)count;
    origin = -eccentricity_measure.offset_b_deg - eccentricity_fir_error_get(0U);
    target = (float)lut_index * (360.0f / (float)ECCENTRICITY_LUT_SIZE);
    while (target < origin)
    {
        target += 360.0f;
    }
    while (target >= origin + 360.0f)
    {
        target -= 360.0f;
    }
    lower = 0U;
    upper = count;
    while ((uint16_t)(upper - lower) > 1U)
    {
        middle = lower + (upper - lower) / 2U;
        if (target >= (float)middle * step - eccentricity_measure.offset_b_deg -
                      eccentricity_fir_error_get(middle))
        {
            lower = middle;
        }
        else
        {
            upper = middle;
        }
    }
    lower_error = eccentricity_fir_error_get(lower);
    upper_error = eccentricity_fir_error_get(upper);
    lower_angle = (float)lower * step - eccentricity_measure.offset_b_deg - lower_error;
    upper_angle = (float)upper * step - eccentricity_measure.offset_b_deg - upper_error;
    fraction = (target - lower_angle) / (upper_angle - lower_angle);
    eccentricity_compensation.correction_deg_lut[lut_index] =
        lower_error + fraction * (upper_error - lower_error);
    lut_index++;
    if (lut_index >= ECCENTRICITY_LUT_SIZE)
    {
        eccentricity_compensation.ready = 1U;
        eccentricity_compensation.save_pending = 1U;
        foc_pwm_output_disable();
    }
}

float eccentricity_compensation_lookup_deg(float mechanical_angle_deg)
{
    float position;
    float fraction;
    uint16_t index;
    uint16_t next;

    if (eccentricity_compensation.ready == 0U)
    {
        return 0.0f;
    }
    while (mechanical_angle_deg < 0.0f)
    {
        mechanical_angle_deg += 360.0f;
    }
    while (mechanical_angle_deg >= 360.0f)
    {
        mechanical_angle_deg -= 360.0f;
    }
    position = mechanical_angle_deg * ((float)ECCENTRICITY_LUT_SIZE / 360.0f);
    index = (uint16_t)position;
    if (index >= ECCENTRICITY_LUT_SIZE)
    {
        index = 0U;
        position = 0.0f;
    }
    fraction = position - (float)index;
    next = (index + 1U) & (ECCENTRICITY_LUT_SIZE - 1U);
    return eccentricity_compensation.correction_deg_lut[index] + fraction *
        (eccentricity_compensation.correction_deg_lut[next] -
         eccentricity_compensation.correction_deg_lut[index]);
}

void eccentricity_compensation_observe(pmsm_t *pm)
{
    eccentricity_compensation.mechanical_angle_rad_before = pm->foc.mechanical_angle_rad;
    eccentricity_compensation.mechanical_angle_deg_before =
        pm->foc.mechanical_angle_rad * (360.0f / TWO_PI);
    eccentricity_compensation.correction_deg = eccentricity_compensation_lookup_deg(
        eccentricity_compensation.mechanical_angle_deg_before);
    eccentricity_compensation.mechanical_angle_deg_after =
        eccentricity_compensation.mechanical_angle_deg_before +
        eccentricity_compensation.correction_deg;
    eccentricity_compensation.mechanical_angle_rad_after =
        pm->foc.mechanical_angle_rad +
        eccentricity_compensation.correction_deg * (TWO_PI / 360.0f);
    pm->foc.electrical_angle_rad_ecc =
        eccentricity_compensation.mechanical_angle_rad_after * (float)pm->motor.pn;
    while (pm->foc.electrical_angle_rad_ecc >= TWO_PI)
    {
        pm->foc.electrical_angle_rad_ecc -= TWO_PI;
    }
    while (pm->foc.electrical_angle_rad_ecc <= -TWO_PI)
    {
        pm->foc.electrical_angle_rad_ecc += TWO_PI;
    }
}

#define ECCENTRICITY_RECORD_TAG 0x45434301UL
#define ECCENTRICITY_RECORD_COMMIT 0x45434302UL

static uint32_t eccentricity_record_checksum(const volatile uint64_t *record)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint32_t byte;
    uint32_t bit;
    for (i = 0U; i < ECCENTRICITY_FLASH_RECORD_WORDS - 1U; i++)
    {
        const uint64_t word = record[i];
        for (byte = 0U; byte < 8U; byte++)
        {
            crc ^= (uint8_t)(word >> (byte * 8U));
            for (bit = 0U; bit < 8U; bit++)
            {
                crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
            }
        }
    }
    return ~crc;
}

uint8_t eccentricity_flash_record_valid(const volatile uint64_t *record)
{
    uint32_t i;
    uint32_t bits;
    float value;
    if (((uint32_t)(record[0] >> 32U) != ECCENTRICITY_RECORD_TAG) ||
        ((uint32_t)(record[ECCENTRICITY_FLASH_RECORD_WORDS - 1U] >> 32U) !=
         ECCENTRICITY_RECORD_COMMIT) ||
        ((uint32_t)record[ECCENTRICITY_FLASH_RECORD_WORDS - 1U] !=
         eccentricity_record_checksum(record)))
    {
        return 0U;
    }
    for (i = 0U; i < ECCENTRICITY_LUT_SIZE; i++)
    {
        bits = (uint32_t)(record[1U + i / 2U] >> ((i & 1U) * 32U));
        memcpy(&value, &bits, sizeof(value));
        if (!isfinite(value))
        {
            return 0U;
        }
    }
    return 1U;
}

HAL_StatusTypeDef eccentricity_compensation_flash_load(void)
{
    const volatile uint64_t *records =
        (const volatile uint64_t *)FLASH_SAVE_ADDRESS;
    const uint32_t coordinate = (uint32_t)calibration.mechanical_zero_raw |
        ((calibration.electrical_direction < 0) ? 0x10000UL : 0U);
    uint32_t i;
    uint32_t j;
    uint32_t bits;
    eccentricity_compensation.ready = 0U;
    eccentricity_compensation.flash_loaded = 0U;
    eccentricity_compensation.flash_saved = 0U;
    eccentricity_compensation.save_pending = 0U;
    for (i = 0U; i + ECCENTRICITY_FLASH_RECORD_WORDS <= FLASH_SAVE_RECORD_COUNT; i++)
    {
        if ((eccentricity_flash_record_valid(&records[i]) != 0U) &&
            ((uint32_t)records[i] == coordinate))
        {
            for (j = 0U; j < ECCENTRICITY_LUT_SIZE; j++)
            {
                bits = (uint32_t)(records[i + 1U + j / 2U] >> ((j & 1U) * 32U));
                memcpy(&eccentricity_compensation.correction_deg_lut[j], &bits, sizeof(bits));
            }
            eccentricity_compensation.mechanical_zero_raw = calibration.mechanical_zero_raw;
            eccentricity_compensation.electrical_direction = calibration.electrical_direction;
            eccentricity_compensation.ready = 1U;
            eccentricity_compensation.flash_loaded = 1U;
            eccentricity_compensation.flash_saved = 1U;
        }
    }
    eccentricity_compensation.flash_status =
        (eccentricity_compensation.flash_loaded != 0U) ? HAL_OK : HAL_ERROR;
    return eccentricity_compensation.flash_status;
}

void eccentricity_compensation_save_process(void)
{
    static uint64_t words[ECCENTRICITY_FLASH_RECORD_WORDS];
    uint32_t i;
    uint32_t low;
    uint32_t high;
    if ((eccentricity_compensation.save_pending == 0U) ||
        (foc_pwm_output_is_enabled() != 0U))
    {
        return;
    }
    eccentricity_compensation.save_pending = 0U;
    words[0] = ((uint64_t)ECCENTRICITY_RECORD_TAG << 32U) |
        eccentricity_compensation.mechanical_zero_raw |
        ((eccentricity_compensation.electrical_direction < 0) ? 0x10000UL : 0U);
    for (i = 0U; i < ECCENTRICITY_LUT_SIZE / 2U; i++)
    {
        memcpy(&low, &eccentricity_compensation.correction_deg_lut[i * 2U], sizeof(low));
        memcpy(&high, &eccentricity_compensation.correction_deg_lut[i * 2U + 1U], sizeof(high));
        words[i + 1U] = ((uint64_t)high << 32U) | low;
    }
    words[ECCENTRICITY_FLASH_RECORD_WORDS - 1U] =
        ((uint64_t)ECCENTRICITY_RECORD_COMMIT << 32U) |
        eccentricity_record_checksum(words);
    eccentricity_compensation.flash_status =
        flash_save_append(words, ECCENTRICITY_FLASH_RECORD_WORDS);
    eccentricity_compensation.flash_saved =
        (uint8_t)(eccentricity_compensation.flash_status == HAL_OK);
}

void eccentricity_identification(pmsm_t *pm)
{
    eccentricity_VF(pm);
    eccentricity_error_process(pm);
    eccentricity_lut_build();
    eccentricity_compensation_observe(pm);
}

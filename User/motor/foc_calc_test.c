#include <assert.h>
#include <stdio.h>

#include "common.h"

static float test_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void assert_float_near(float actual, float expected)
{
    assert(test_abs(actual - expected) < 0.00001f);
}

static void reset_current(pmsm_t *motor)
{
    motor->foc.i_a = 8.0f;
    motor->foc.i_b = -3.0f;
    motor->foc.i_c = -4.0f;
}

static void test_window_boundaries_are_inclusive(void)
{
    pmsm_t motor = {0};

    reset_current(&motor);
    foc_current_process(&motor, 600U, 3400U, 2000U,
                        CURRENT_RECONSTRUCTION_DISABLED);

    assert(motor.current_sample.valid_a == 1U);
    assert(motor.current_sample.valid_b == 1U);
    assert(motor.current_sample.valid_c == 1U);
}

static void test_invalid_window_reconstructs_largest_current_phase(void)
{
    pmsm_t motor = {0};

    reset_current(&motor);
    foc_current_process(&motor, 599U, 2000U, 2000U,
                        CURRENT_RECONSTRUCTION_MAX_CURRENT);

    assert(motor.current_sample.reconstructed_phase == PMSM_PHASE_A);
    assert_float_near(motor.foc.i_a, 7.0f);
    assert_float_near(motor.foc.i_alph, 7.0f);
    assert_float_near(motor.foc.i_beta, ONE_BY_SQRT3);
}

static void test_valid_windows_use_normal_clarke_in_max_current_mode(void)
{
    pmsm_t motor = {0};

    reset_current(&motor);
    foc_current_process(&motor, 800U, 1200U, 2000U,
                        CURRENT_RECONSTRUCTION_MAX_CURRENT);

    assert(motor.current_sample.reconstructed_phase == PMSM_PHASE_NONE);
    assert_float_near(motor.foc.i_a, 8.0f);
    assert_float_near(motor.foc.i_alph, 7.666666667f);
    assert_float_near(motor.foc.i_beta, ONE_BY_SQRT3);
}

static void test_min_window_mode_reconstructs_shortest_window_phase(void)
{
    pmsm_t motor = {0};

    reset_current(&motor);
    foc_current_process(&motor, 800U, 1200U, 2000U,
                        CURRENT_RECONSTRUCTION_MIN_WINDOW);

    assert(motor.current_sample.window_a == 800U);
    assert(motor.current_sample.window_b == 1200U);
    assert(motor.current_sample.window_c == 2000U);
    assert(motor.current_sample.reconstructed_phase == PMSM_PHASE_A);
    assert_float_near(motor.foc.i_a, 7.0f);
    assert_float_near(motor.foc.i_alph, 7.0f);
}

int main(void)
{
    test_window_boundaries_are_inclusive();
    test_invalid_window_reconstructs_largest_current_phase();
    test_valid_windows_use_normal_clarke_in_max_current_mode();
    test_min_window_mode_reconstructs_shortest_window_phase();
    puts("foc_calc tests passed");
    return 0;
}

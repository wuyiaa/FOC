#ifndef USER_MOTOR_MOTOR_CONFIG_H
#define USER_MOTOR_MOTOR_CONFIG_H

/* Motor electrical parameters: inductance in H, resistance in Ohm. */
#define FOC_MOTOR_LD_H                         0.001423f
#define FOC_MOTOR_LQ_H                         0.002849f
#define FOC_MOTOR_RS_OHM                       6.617f
#define FOC_MOTOR_POLE_PAIRS                   11U

/* Current-loop tuning and voltage limits. */
#define FOC_CURRENT_LOOP_BANDWIDTH_HZ          500.0f
#define FOC_DC_BUS_VOLTAGE_V                   24.0f
#define FOC_CURRENT_OUTPUT_UTILIZATION         0.95f
#define FOC_CURRENT_INTEGRAL_LIMIT_RATIO       0.5f
#define FOC_CURRENT_KD_V_S_PER_A                0.0f
#define FOC_CURRENT_DEADBAND_A                  0.0f

/* Latched phase-current trip threshold. */
#define FOC_CURRENT_TRIP_LIMIT_A               20.000f

#endif /* USER_MOTOR_MOTOR_CONFIG_H */

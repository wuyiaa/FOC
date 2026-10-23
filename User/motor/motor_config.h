#ifndef USER_MOTOR_MOTOR_CONFIG_H
#define USER_MOTOR_MOTOR_CONFIG_H

/* Motor electrical parameters: inductance in H, resistance in Ohm. */
#define FOC_MOTOR_LD_H                         0.001423f
#define FOC_MOTOR_LQ_H                         0.002849f
#define FOC_MOTOR_RS_OHM                       6.617f
#define FOC_MOTOR_POLE_PAIRS                   11U

/* Current-loop tuning and voltage limits. */
/* Live tuning variable (Hz), defined in pid.c. Edit this in Keil Watch. */
extern volatile float FOC_CURRENT_LOOP_BANDWIDTH_DEFAULT_HZ;
#define FOC_DC_BUS_VOLTAGE_V                   24.0f
/* Live limit margin, defined in pid.c; accepted range: 0.0f to 1.0f.
   The bus voltage remains fixed until measured voltages are validated. */
extern volatile float FOC_CURRENT_OUTPUT_UTILIZATION;
#define FOC_CURRENT_INTEGRAL_LIMIT_RATIO       0.5f
#define FOC_CURRENT_KD_V_S_PER_A                0.0f
#define FOC_CURRENT_DEADBAND_A                  0.0f

/* Latched phase-current trip threshold. */
#define FOC_CURRENT_TRIP_LIMIT_A               20.000f
/* AS5047P count decreases when the shaft turns counterclockwise. */
#define MOTOR_MECHANICAL_DRV                     -1.0f
#define MOTOR_SPEED_DIRECTION                    -1.0f
/* Existing 321 wiring uses an effective positive angle after the old
   MOTOR_MECHANICAL_DRV and sin(-angle) signs cancel. */
#define MOTOR_PHASE_REVERSED_DEFAULT             1U //1--321œ‡–Ú 0--123œ‡–Ú
#define MOTOR_ELECTRICAL_DIRECTION_DEFAULT       1	//1--+theta  -1--(-theta)

/* Automatic phase/direction calibration, enabled by the calibration mode. */
#define MOTOR_CALIBRATION_UQ_V                    5.0f
#define MOTOR_CALIBRATION_SETTLE_TIME_MS          200U
#define MOTOR_CALIBRATION_DRIVE_TIME_MS           800U
#define MOTOR_CALIBRATION_MIN_MOVE_COUNTS         164U

#endif /* USER_MOTOR_MOTOR_CONFIG_H */

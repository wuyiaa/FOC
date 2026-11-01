#ifndef USER_APP_TRIG_LUT_H
#define USER_APP_TRIG_LUT_H

#include <stdint.h>

typedef struct
{
    int16_t hCos;
    int16_t hSin;
} Trig_Components;

/* theta = hAngle * pi / 32768; output range: -32767 to +32767. */
Trig_Components trig_functions(int16_t hAngle);

/* Input [-2pi, 2pi]: normalize to [-pi, pi); float outputs are in [-1, 1]. */
void trig_functions_rad(float angle_rad, float *sin_theta, float *cos_theta);

#endif /* USER_APP_TRIG_LUT_H */

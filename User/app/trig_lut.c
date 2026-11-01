#include "trig_lut.h"

#define TRIG_LUT_PI              3.14159265358979323846f
#define TRIG_LUT_TWO_PI          6.28318530717958647692f
#define TRIG_LUT_RAD_TO_HANGLE   (32768.0f / TRIG_LUT_PI)

/* First-quadrant sine: sin(index * pi / 512) * 32767, rounded.
   Both endpoints are included: index 0 = 0 deg, index 256 = 90 deg. */
static const int16_t hSin_Cos_Table[257] =
{
        0,   201,   402,   603,   804,  1005,  1206,  1407,
     1608,  1809,  2009,  2210,  2410,  2611,  2811,  3012,
     3212,  3412,  3612,  3811,  4011,  4210,  4410,  4609,
     4808,  5007,  5205,  5404,  5602,  5800,  5998,  6195,
     6393,  6590,  6786,  6983,  7179,  7375,  7571,  7767,
     7962,  8157,  8351,  8545,  8739,  8933,  9126,  9319,
     9512,  9704,  9896, 10087, 10278, 10469, 10659, 10849,
    11039, 11228, 11417, 11605, 11793, 11980, 12167, 12353,
    12539, 12725, 12910, 13094, 13279, 13462, 13645, 13828,
    14010, 14191, 14372, 14553, 14732, 14912, 15090, 15269,
    15446, 15623, 15800, 15976, 16151, 16325, 16499, 16673,
    16846, 17018, 17189, 17360, 17530, 17700, 17869, 18037,
    18204, 18371, 18537, 18703, 18868, 19032, 19195, 19357,
    19519, 19680, 19841, 20000, 20159, 20317, 20475, 20631,
    20787, 20942, 21096, 21250, 21403, 21554, 21705, 21856,
    22005, 22154, 22301, 22448, 22594, 22739, 22884, 23027,
    23170, 23311, 23452, 23592, 23731, 23870, 24007, 24143,
    24279, 24413, 24547, 24680, 24811, 24942, 25072, 25201,
    25329, 25456, 25582, 25708, 25832, 25955, 26077, 26198,
    26319, 26438, 26556, 26674, 26790, 26905, 27019, 27133,
    27245, 27356, 27466, 27575, 27683, 27790, 27896, 28001,
    28105, 28208, 28310, 28411, 28510, 28609, 28706, 28803,
    28898, 28992, 29085, 29177, 29268, 29358, 29447, 29534,
    29621, 29706, 29791, 29874, 29956, 30037, 30117, 30195,
    30273, 30349, 30424, 30498, 30571, 30643, 30714, 30783,
    30852, 30919, 30985, 31050, 31113, 31176, 31237, 31297,
    31356, 31414, 31470, 31526, 31580, 31633, 31685, 31736,
    31785, 31833, 31880, 31926, 31971, 32014, 32057, 32098,
    32137, 32176, 32213, 32250, 32285, 32318, 32351, 32382,
    32412, 32441, 32469, 32495, 32521, 32545, 32567, 32589,
    32609, 32628, 32646, 32663, 32678, 32692, 32705, 32717,
    32728, 32737, 32745, 32752, 32757, 32761, 32765, 32766,
    32767
};

/* fraction is 0..63. Add half a step before shifting to round. */
static inline int16_t trig_lerp(int16_t a, int16_t b, uint16_t fraction)
{
    const uint32_t sum =
        (uint32_t)a * (64u - fraction) + (uint32_t)b * fraction;

    return (int16_t)((sum + 32u) >> 6);
}

Trig_Components trig_functions(int16_t hAngle)
{
    const uint16_t phase = (uint16_t)hAngle;
    const uint16_t quadrant = (uint16_t)(phase >> 14);
    const uint16_t offset = (uint16_t)(phase & 0x3FFFu);
    const uint16_t index = (uint16_t)(offset >> 6);
    const uint16_t fraction = (uint16_t)(offset & 0x003Fu);
    const int16_t s = trig_lerp(hSin_Cos_Table[index],
                                hSin_Cos_Table[index + 1u], fraction);
    const int16_t c = trig_lerp(hSin_Cos_Table[256u - index],
                                hSin_Cos_Table[255u - index], fraction);
    Trig_Components result;

    switch (quadrant)
    {
        case 0u:
            result.hSin = s;
            result.hCos = c;
            break;

        case 1u:
            result.hSin = c;
            result.hCos = (int16_t)-s;
            break;

        case 2u:
            result.hSin = (int16_t)-s;
            result.hCos = (int16_t)-c;
            break;

        default:
            result.hSin = (int16_t)-c;
            result.hCos = s;
            break;
    }

    return result;
}

void trig_functions_rad(float angle_rad, float *sin_theta, float *cos_theta)
{
    int16_t hAngle;
    Trig_Components result;

    /* FOC supplies an angle in [-2pi, 2pi]; one correction is sufficient. */
    if (angle_rad >= TRIG_LUT_PI)
    {
        angle_rad -= TRIG_LUT_TWO_PI;
    }
    else if (angle_rad < -TRIG_LUT_PI)
    {
        angle_rad += TRIG_LUT_TWO_PI;
    }

    hAngle = (int16_t)(angle_rad * TRIG_LUT_RAD_TO_HANGLE);
    result = trig_functions(hAngle);
    *sin_theta = (float)result.hSin * (1.0f / 32767.0f);
    *cos_theta = (float)result.hCos * (1.0f / 32767.0f);
}

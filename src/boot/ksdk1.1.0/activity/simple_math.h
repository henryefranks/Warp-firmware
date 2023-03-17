/*
 * QUICK MATHS
 *
 * Some questionably-implemented maths functions to help calculate angles.
 * The flag ENABLE_FP allows for arctan to be calcualted with an approximate
 * scaled integer implementation, but there is no benefit to this and
 * performance is pretty cromulent with floating points.
 *
 * Stay groovy.
 */

#include <stdint.h>
#include <math.h>

#define ARCTAN_SCALE 1000 /* integer scale */
#define PI_2 1571         /* pi / 2 * 1000 */
#define RAD2DEG 573       /* 180 / pi * 10 */

#define ENABLE_FP 1 /* enable/disable floating point */

int sign(int x)
{
    return (x < 0) ? -1 : 1;
}

int
quick_arctan(int x)
{ /* quick 'n' dirty integer arctan (based on taylor series) */
    return x - (x * x * x) / (3 * ARCTAN_SCALE * ARCTAN_SCALE);
}

int
arctan(int16_t a_h, int16_t a_v)
{
    /* returns arctan(a_h / a_v) between -180 - +180 degrees,  */
    /* as an integer representation of angle in degrees * 1000 */
    /* could use floats to improve performance */

    /* a wrapper function like this is needed, even if using floats,         */
    /* to correctly bound and offset the angle to give a smooth +/-180 range */
    int angle_deg;

    if (a_v == 0) return (a_h > 0) ? -90000 :  90000;
    if (a_h == 0) return (a_v > 0) ?      0 : 180000;

    #if (ENABLE_FP)
        float x, angle;

        x = (float) a_h / a_v;
        angle = atan(x);

        angle_deg = (int)(ARCTAN_SCALE * (180.0f / M_PI) * angle);

        if (a_v < 0) {
            if (a_h > 0) {
                angle_deg = 180000 + angle_deg;
            } else {
                angle_deg = -180000 + angle_deg;
            }
        }
    #else
        /* horrible integer implementation */
        /* this works, but is best avoided */
        int x, angle;

        x = (abs(a_h) > abs(a_v)) ?
            (ARCTAN_SCALE * a_v / a_h) : (ARCTAN_SCALE * a_h / a_v);

        angle = quick_arctan(x);

        angle_deg = RAD2DEG * angle / 10;

        if (a_v > 0) {
            angle_deg = (abs(a_h) > abs(a_v)) ?
                sign(a_h) * 90000 - angle_deg : angle_deg;
        } else {
            angle_deg = (abs(a_h) > abs(a_v)) ?
                sign(a_h) * 90000 - angle_deg :
                sign(a_h) * 180000 + angle_deg;
        }
    #endif

    /* rounding is non-ideal, could just return a float */
    return angle_deg;
}
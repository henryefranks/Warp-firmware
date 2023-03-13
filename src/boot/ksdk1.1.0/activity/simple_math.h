/*
 * QUICK MATHS
 *
 * Stay groovy.
 */

#include <stdint.h>
#include <math.h>

#define ARCTAN_SCALE 1000
#define PI_2 1571
#define RAD2DEG 57

#define ENABLE_FP 0 /* enable/disable floating point */

int
quick_arctan(int x)
{
    return (
        x - (x * x * x) / (3 * ARCTAN_SCALE * ARCTAN_SCALE)
    );
}

int
arctan(int16_t a_h, int16_t a_v)
{
    if (a_v == 0) return (a_h > 0) ? 90000 : -90000;
    #if (ENABLE_FP)
        float ratio, angle;

        ratio = (float) a_h / a_v;
        angle = atan(ratio);

        return (int)(ARCTAN_SCALE * (180.0f / M_PI) * angle);
    #else
        int x, at;

        if (abs(a_h) > abs(a_v)) {
            x = ARCTAN_SCALE * a_v / a_h;
            at = quick_arctan(x);
            
            if (a_h > 0) {
                if (a_v > 0) {
                    return  -RAD2DEG * at + (180 * ARCTAN_SCALE);
                } else {
                    return  -RAD2DEG * at - (180 * ARCTAN_SCALE);
                }
            } else {
                return -RAD2DEG * at;
            }
        } else {
            x = ARCTAN_SCALE * a_h / a_v;
            at = quick_arctan(x);
            
            if (a_v > 0) {
                return RAD2DEG * at + (90 * ARCTAN_SCALE);
            } else {
                return RAD2DEG * at - (90 * ARCTAN_SCALE);
            }
        }
    #endif
}
#ifndef _GFX_H_
#define _GFX_H_

#include <stdint.h>

#define ZSCALE 10.0f

typedef struct {
    uint8_t r, g, b;
} colour;

typedef struct {
    float x, y;
} fpoint_2d;

typedef struct {
    float x, y, z;
} fpoint_3d;

int drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, colour col);
int drawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, colour col);

int draw3DLine(fpoint_3d start, fpoint_3d end, colour col);

int cls();

#endif


#define ZSCALE 20.0f

typedef struct {
    uint8_t r, g, b;
} colour;

typedef struct {
    float x, y;
} fpoint_2d;

typedef struct {
    float x, y, z;
} fpoint_3d;

void drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, colour col);
void drawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, colour col);
void draw3DLine(fpoint_3d start, fpoint_3d end, colour col);
void drawCube(float width, colour col);


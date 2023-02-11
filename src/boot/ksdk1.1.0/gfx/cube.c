#include <stdint.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "warp.h"
#include "cube.h"
#include "gfx.h"

static fpoint_3d cube[] = {
    { -width/2, -width/2,          1.0f + offset },
    {  width/2, -width/2,          1.0f + offset },
    {  width/2,  width/2,          1.0f + offset },
    { -width/2,  width/2,          1.0f + offset },

    { -width/2, -width/2,  width + 1.0f + offset },
    {  width/2, -width/2,  width + 1.0f + offset },
    {  width/2,  width/2,  width + 1.0f + offset },
    { -width/2,  width/2,  width + 1.0f + offset },
};

static void
drawCube()
{
    static const colour col = {255, 0, 0};
    size_t i = 0;

    // front face
    for (i = 0; i < 3; i++)
        draw3DLine(cube[i], cube[i+1], col);
    draw3DLine(cube[3], cube[0], col);

    // back face
    for (i = 4; i < 7; i++)
        draw3DLine(cube[i], cube[i+1], col);
    draw3DLine(cube[7], cube[4], col);

    // connect faces
    for (i = 0; i < 4; i++)
        draw3DLine(cube[i], cube[i+4], col);
}

static void
rotateCube()
{
    static const float zaxis = width/2 + offset;

    static const float s = 0.08715574275f;
    static const float c = 0.99619469810f;

    for (size_t i = 0; i < 8; i++)
    {
        float x = cube[i].x;
        float z = cube[i].z - zaxis;
        cube[i].x = c * x - s * z;
        cube[i].z = s * x + c * z + zaxis;
    }
}

void
drawSpinningCube()
{
    drawCube();
    rotateCube();
}


#include <stdint.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "warp.h"
#include "gfx.h"
#include "devSSD1331.h"

static volatile uint8_t	inBuffer[1];

/*
 *	MAKE SURE THIS MATCHES devSSD1331.c
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOA, 2),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 3),
};

static fpoint_2d
rasterizePoint(fpoint_3d p)
{
    static const float WIDTH = 96.0f;
    static const float HEIGHT = 64.0f;

    fpoint_2d pt = {
        (uint8_t) (WIDTH  / 2 + p.x / p.z * ZSCALE ),
        (uint8_t) (HEIGHT / 2 + p.y / p.z * ZSCALE )
    };

    return pt;
}

int
drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, colour col)
{
	spi_status_t status;

	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

    uint8_t buf[] = {
        kSSD1331CommandDRAWRECT,
        x, y,
        x + w, y + h,
        col.r, col.g, col.b
    };

	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)buf,
					(uint8_t * restrict)&inBuffer[0],
					8		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

    return status;
}

int
drawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, colour col)
{
	spi_status_t status;

	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

    uint8_t buf[] = {
        kSSD1331CommandDRAWLINE,
        x1, y1,
        x2, y2,
        col.r, col.g, col.b
    };

	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)buf,
					(uint8_t * restrict)&inBuffer[0],
					8		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

    return status;
}

int
draw3DLine(fpoint_3d start, fpoint_3d end, colour col)
{
    fpoint_2d s = rasterizePoint(start);
    fpoint_2d e = rasterizePoint(end);

    return drawLine(
        (uint8_t) s.x, (uint8_t) s.y,
        (uint8_t) e.x, (uint8_t) e.y,
        col
    );
}


int
cls()
{
	spi_status_t status;

	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

    uint8_t buf[] = {
        kSSD1331CommandCLEAR,
        0x00, 0x00,
        0x5F, 0x3F
    };

	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)buf,
					(uint8_t * restrict)&inBuffer[0],
					5		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

    return status;
}

#include <stdint.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t	inBuffer[1];
volatile uint8_t	payloadBytes[1];

#define ZSCALE 20.0f
#define offset  0.0f
#define width  30.0f

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
void drawCube();
void rotateCube(size_t angle);

/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOA, 2),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 3),
};

static float
depthCalc(float z)
{
    //return (uint8_t)1 << (int)(z / ZSCALE);
    return 1 + z / ZSCALE;
}

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}


void
devSSD1331clear()
{
	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
}

void
devSSD1331drawCube()
{
    drawCube();
    rotateCube(1);
}

int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	warpEnableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x06);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0x50);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);


    devSSD1331clear();

	/*
	 *	Any post-initialization drawing commands go here.
	 */
	//...

    devSSD1331drawCube();

    //writeCommand(kSSD1331CommandDRAWRECT);

    // start x, y
    //writeCommand(0);
    //writeCommand(0);

    // end x, y
    //writeCommand(95);
    //writeCommand(62);

    // outline
    //writeCommand(0);
    //writeCommand(255);
    //writeCommand(0);

    // fill
    //writeCommand(0);
    //writeCommand(255);
    //writeCommand(0);

    //drawRect(0, 0, 10, 10, {255, 0, 0});

	return 0;
}

void
drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, colour col)
{
    writeCommand(kSSD1331CommandDRAWRECT);

    // start x, y
    writeCommand(x);
    writeCommand(y);

    // end x, y
    writeCommand(x+w-1);
    writeCommand(y+h-1);

    // outline
    writeCommand(col.r);
    writeCommand(col.g);
    writeCommand(col.b);

    // fill
    writeCommand(col.r);
    writeCommand(col.g);
    writeCommand(col.b);
}

void
drawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, colour col)
{
    writeCommand(kSSD1331CommandDRAWLINE);

    // start x, y
    writeCommand(x1);
    writeCommand(y1);

    // end x, y
    writeCommand(x2);
    writeCommand(y2);

    // color
    writeCommand(col.r);
    writeCommand(col.g);
    writeCommand(col.b);
}

static int
drawLine2(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, colour col)
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

fpoint_2d
rasterizePoint(fpoint_3d p)
{
    static const float WIDTH = 96.0f;
    static const float HEIGHT = 64.0f;

    fpoint_2d pt = {
        (uint8_t) (WIDTH  / 2 + p.x / depthCalc(p.z) ),
        (uint8_t) (HEIGHT / 2 + p.y / depthCalc(p.z) )
    };

    return pt;
}

void
draw3DLine(fpoint_3d start, fpoint_3d end, colour col)
{
    fpoint_2d s = rasterizePoint(start);
    fpoint_2d e = rasterizePoint(end);

    drawLine2(s.x, s.y, e.x, e.y, col);
}

//void
//draw3DRect(float width, float z, colour col)
//{
//    fpoint_2d origin = rasterizePoint(
//        (fpoint_3d) { -width/2, -width/2, z }
//    );
//
//    uint8_t start_x = (uint8_t) origin.x;
//    uint8_t start_y = (uint8_t) origin.y;
//    uint8_t w = (uint8_t) width / (1 + z/ZSCALE);
//
//    drawRect(start_x, start_y, w, w, col);
//}

static fpoint_3d cube[] = {
    { -width/2, -width/2,   0.0f + offset },
    {  width/2, -width/2,   0.0f + offset },
    {  width/2,  width/2,   0.0f + offset },
    { -width/2,  width/2,   0.0f + offset },
    { -width/2, -width/2,  width + offset },
    {  width/2, -width/2,  width + offset },
    {  width/2,  width/2,  width + offset },
    { -width/2,  width/2,  width + offset },
};

float ssin(size_t angle);
float ccos(size_t angle);

float ssin(size_t angle)
{
    static const float lookup[] = {
        0.000,
        0.174,
        0.342,
        0.500,
        0.643,
        0.766,
        0.866,
        0.940,
        0.985,
        1.000,
        0.985,
        0.940,
        0.866,
        0.766,
        0.643,
        0.500,
        0.342,
        0.174,
        0.000
    };

    return lookup[angle];
}

float ccos(size_t angle)
{
    static const float lookup[] = {
        1.000,
        0.985,
        0.940,
        0.866,
        0.766,
        0.643,
        0.500,
        0.342,
        0.174,
        0.000,
        -0.174,
        -0.342,
        -0.500,
        -0.643,
        -0.766,
        -0.866,
        -0.940,
        -0.985,
        -1.000
    };

    return lookup[angle];
}

void
rotateCube(size_t angle)
{
    static const float zaxis = width/2 + offset;

    float s = ssin(angle);
    float c = ccos(angle);
    warpPrint("%f - %f", s, c);

    for (size_t i = 0; i < 8; i++)
    {
        float x = cube[i].x;
        float z = cube[i].z - zaxis;
        cube[i].x = c * x - s * z;
        cube[i].z = s * x + c * z + zaxis;
    }
}

void
drawCube()
{
    #define col ( (colour) {255, 0, 0} )
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


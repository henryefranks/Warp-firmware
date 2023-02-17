#include <stdlib.h>
#include <math.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devINA219.h"


extern volatile WarpI2CDeviceState	deviceINA219State;

extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;


void
devINA219init(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
    deviceINA219State.i2cAddress = i2cAddress;
    deviceINA219State.operatingVoltageMillivolts = operatingVoltageMillivolts;

    devINA219writeRegister(kINA219RegisterConfig, config.raw_val);
    devINA219writeRegister(kINA219RegisterCalibration, INA219_CAL);

    OSA_TimeDelay(100);

    devINA219writeRegisterPointer(kINA219RegisterCurrent);

    return;
}

WarpStatus
devINA219writeRegister(INA219Register deviceRegister, uint16_t payload)
{
	uint8_t		payloadByte[2], commandByte[1];
	i2c_status_t	status;

    if (deviceRegister > 0x05) {
	    return kWarpStatusBadDeviceCommand;
    }

	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadByte[0] = (uint8_t) (payload >> 8);
	payloadByte[1] = (uint8_t) (payload & 0xFF);
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							2,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
devINA219writeRegisterPointer(INA219Register deviceRegister)
{
	uint8_t		commandByte[1];
	i2c_status_t	status;

    if (deviceRegister > 0x05) {
	    return kWarpStatusBadDeviceCommand;
    }

	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							NULL,
							0,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
devINA219read(void)
{
	i2c_status_t	status;

	USED(2);

	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C instance */,
							&slave,
							NULL,
							0,
							(uint8_t *)deviceINA219State.i2cBuffer,
							2,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

extern void warpPrint(const char *fmt, ...);

unsigned int
devINA219getCurrent_uA(void)
{
    uint16_t current_raw;

    if (devINA219read() != kWarpStatusOK)  /* read from device */
        return NAN;                       /* error condition  */

    current_raw = (uint16_t) (
        deviceINA219State.i2cBuffer[1] |
        deviceINA219State.i2cBuffer[0] << 8
    );

    return current_raw * INA219_CURRENT_LSB;
}



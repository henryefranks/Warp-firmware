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

#define INA219_DEBUG 0

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

#if (INA219_DEBUG)
    extern void warpPrint(const char *fmt, ...);
#endif

int
devINA219getCurrent(void)
{
    uint16_t current_raw;
    WarpStatus status;

    /* set register pointer to current */
    status = devINA219writeRegisterPointer(kINA219RegisterCurrent);
    if (status != kWarpStatusOK) return 0; /* error condition  */

    /* read from device */
    status = devINA219read();
    if (status != kWarpStatusOK) return 0; /* error condition  */

    current_raw = (int16_t) (
        deviceINA219State.i2cBuffer[1] |
        deviceINA219State.i2cBuffer[0] << 8
    );

    return (int) (current_raw * INA219_CURRENT_LSB);
}

unsigned int
devINA219getBusVoltage(void)
{
    ina219_reg_bus_voltage_t voltage_raw;
    WarpStatus status;

    /* set register pointer to current */
    status = devINA219writeRegisterPointer(kINA219RegisterBusVoltage);
    if (status != kWarpStatusOK) return 0; /* error condition  */

    /* read from device */
    status = devINA219read();
    if (status != kWarpStatusOK) return 0; /* error condition  */

    voltage_raw.lsb = deviceINA219State.i2cBuffer[1];
    voltage_raw.msb = deviceINA219State.i2cBuffer[0];

    /* LSB = 4mV */
    return (unsigned int) (voltage_raw.bd * 4);
}

int
devINA219getShuntVoltage(void)
{
    int16_t voltage_raw;
    WarpStatus status;

    /* set register pointer to current */
    status = devINA219writeRegisterPointer(kINA219RegisterShuntVoltage);
    if (status != kWarpStatusOK) return 0; /* error condition  */

    /* read from device */
    status = devINA219read();
    if (status != kWarpStatusOK) return 0; /* error condition  */

    voltage_raw = (int16_t) (
        deviceINA219State.i2cBuffer[1] |
        deviceINA219State.i2cBuffer[0] << 8
    );

    /* LSB = 10uV */
    return (int) (voltage_raw * 10);
}


unsigned int
devINA219getPower(void)
{
    uint16_t power_raw;
    WarpStatus status;

    /* set register pointer to current */
    status = devINA219writeRegisterPointer(kINA219RegisterPower);
    if (status != kWarpStatusOK) return 0; /* error condition  */

    /* read from device */
    status = devINA219read();
    if (status != kWarpStatusOK) return 0; /* error condition  */

    power_raw = (uint16_t) (
        deviceINA219State.i2cBuffer[1] |
        deviceINA219State.i2cBuffer[0] << 8
    );

    /* LSB = 4mV */
    return (unsigned int) (power_raw * INA219_POWER_LSB);
}

ina219_reading_set_t
devINA219readAllTriggered(void)
{
    WarpStatus status;

    ina219_reading_set_t res;
    ina219_reg_bus_voltage_t bus_voltage;

    static const ina219_reading_set_t error_val = { .error = true };

    res.error = false;

    /* write config to trigger a conversion */
    status = devINA219writeRegister(kINA219RegisterConfig, config.raw_val);
    if (status != kWarpStatusOK) return error_val; /* error condition  */

    /* small delay to allow conv to propagate */
    OSA_TimeDelay(10);
    
    status = devINA219writeRegisterPointer(kINA219RegisterBusVoltage);
    if (status != kWarpStatusOK) return error_val; /* error condition  */

    do {
        /* read from device */
        status = devINA219read();
        if (status != kWarpStatusOK) return error_val; /* error condition  */

        bus_voltage.lsb = deviceINA219State.i2cBuffer[1];
        bus_voltage.msb = deviceINA219State.i2cBuffer[0];
    } while (bus_voltage.cnvr == 0);

    res.busVoltage = bus_voltage.bd * 4;
    res.current = devINA219getCurrent();
    res.shuntVoltage = devINA219getShuntVoltage();
    res.power = devINA219getPower();

    return res;
}


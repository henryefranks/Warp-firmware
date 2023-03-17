/*
 * MMA8451Q DRIVER
 * 2023 Henry Franks
 * 
 * ...and party on dudes!
 */

/*
    FOR SECTIONS OF THE CODE BELOW, SEE THE FOLLOWING COPYRIGHT NOTICE
    ==================================================================

	Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards, see git log.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/

/*
 *	config.h needs to come first
 */
#include "../config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "../SEGGER_RTT.h"
#include "../warp.h"

#include "devMMA8451Q.h"

#include "reg/ctrl.h"
#include "reg/fifo.h"
#include "reg/data.h"

#include "simple_math.h"

extern volatile WarpI2CDeviceState	deviceMMA8451QState;

extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

extern void warpEnableI2Cpins(void);

static const devMMA8451Q_ctrl_t cfg = {
    .active        = fs_mode_active,
    .f_read        = fr_mode_normal,
    .lnoise        = lnoise_mode_normal,
    .dr            = odr_50hz,
    .aslp_rate     = aslp_rate_1_56hz,

    .mods          = mods_high_resolution,
    .slpe          = auto_sleep_disable,
    .smods         = mods_low_power,
    .rst           = reset_disable,
    .st            = self_test_disable,

    .pp_od         = int_push_pull,
    .ipol          = int_active_high,
    .wake_ff_mt    = fifo_gate_block,

    .wake_pulse    = cfg_flag_disabled,
    .wake_lndprt   = cfg_flag_disabled,
    .wake_trans    = cfg_flag_disabled,
    .fifo_gate     = cfg_flag_disabled,

    .int_en_drdy   = cfg_flag_disabled,
    .int_en_ff_mt  = cfg_flag_disabled,
    .int_en_pulse  = cfg_flag_disabled,
    .int_en_lndprt = cfg_flag_disabled,
    .int_en_trans  = cfg_flag_disabled,
    .int_en_fifo   = cfg_flag_disabled,
    .int_en_aslp   = cfg_flag_disabled,

    .int_cfg_drdy   = cfg_flag_disabled,
    .int_cfg_ff_mt  = cfg_flag_disabled,
    .int_cfg_pulse  = cfg_flag_disabled,
    .int_cfg_lndprt = cfg_flag_disabled,
    .int_cfg_trans  = cfg_flag_disabled,
    .int_cfg_fifo   = cfg_flag_disabled,
    .int_cfg_aslp   = cfg_flag_disabled,
};

static const devMMA8451Q_f_setup_t f_setup = {
	/* enable FIFO circular buffer mode */
    .f_wmrk = 0,
    .f_mode = fifo_circular_buffer,
};

static const devMMA8451Q_xyz_data_cfg_t xyz_data_cfg = {
    .fs      = fsr_2g,
    .hpf_out = data_flag_disabled,
};

WarpStatus
devMMA8451Q_readReg(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03: 
		case 0x04: case 0x05: case 0x06: case 0x09:
		case 0x0a: case 0x0b: case 0x0c: case 0x0d:
		case 0x0e: case 0x0f: case 0x10: case 0x11:
		case 0x12: case 0x13: case 0x14: case 0x15:
		case 0x16: case 0x17: case 0x18: case 0x1d:
		case 0x1e: case 0x1f: case 0x20: case 0x21:
		case 0x22: case 0x23: case 0x24: case 0x25:
		case 0x26: case 0x27: case 0x28: case 0x29:
		case 0x2a: case 0x2b: case 0x2c: case 0x2d:
		case 0x2e: case 0x2f: case 0x30: case 0x31:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceMMA8451QState.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
devMMA8451Q_writeReg(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x09: case 0x0a: case 0x0e: case 0x0f:
		case 0x11: case 0x12: case 0x13: case 0x14:
		case 0x15: case 0x17: case 0x18: case 0x1d:
		case 0x1f: case 0x20: case 0x21: case 0x23:
		case 0x24: case 0x25: case 0x26: case 0x27:
		case 0x28: case 0x29: case 0x2a: case 0x2b:
		case 0x2c: case 0x2d: case 0x2e: case 0x2f:
		case 0x30: case 0x31:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
devMMA8451Q_init(
    const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	WarpStatus status = kWarpStatusOK;

	deviceMMA8451QState.i2cAddress					= i2cAddress;
	deviceMMA8451QState.operatingVoltageMillivolts	= operatingVoltageMillivolts;
	
	// for (uint8_t i = 0; i < 1; i++) {
	// 	status |= devMMA8451Q_writeReg(
	// 		kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1 + i,
	// 		cfg.raw_bytes[i]);
	// }

	status |= devMMA8451Q_writeReg(
		kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1,
		cfg.raw_bytes[0]);

	status |= devMMA8451Q_writeReg(
		kWarpSensorConfigurationRegisterMMA8451QF_SETUP,
		f_setup.raw_byte);

	status |= devMMA8451Q_writeReg(
		kWarpSensorConfigurationRegisterMMA8451QXYZ_DATA_CFG,
		xyz_data_cfg.raw_byte);

	return status;
}

devMMA8451Q_accel_reading_t
devMMA8451Q_getAccel(void)
{
	devMMA8451Q_accel_reading_t data;
	WarpStatus	i2cReadStatus;

    i2cReadStatus = devMMA8451Q_readReg(0x01, 6);

	if (i2cReadStatus != kWarpStatusOK) {
		data.error = true;
		return data;
	}

	data.error = false;

	data.x = ((deviceMMA8451QState.i2cBuffer[0] & 0xFF) << 6) | (deviceMMA8451QState.i2cBuffer[1] >> 2);
	data.x = (data.x ^ (1 << 13)) - (1 << 13);
    
	data.y = ((deviceMMA8451QState.i2cBuffer[2] & 0xFF) << 6) | (deviceMMA8451QState.i2cBuffer[3] >> 2);
	data.y = (data.y ^ (1 << 13)) - (1 << 13);

	data.z = ((deviceMMA8451QState.i2cBuffer[4] & 0xFF) << 6) | (deviceMMA8451QState.i2cBuffer[5] >> 2);
	data.z = (data.z ^ (1 << 13)) - (1 << 13);

    return data;
}

int angle_buffer[10];
bool angle_buffer_full = false;
size_t wt_idx = 0;

void
devMMA8451Q_updateBuffer(void)
{
	WarpStatus status;
	devMMA8451Q_f_status_t f_status;
	devMMA8451Q_accel_reading_t data;

	int ret;

	static int count = 0;

	status = kWarpStatusOK;

	status |= devMMA8451Q_readReg(
		kWarpSensorConfigurationRegisterMMA8451QF_STATUS, 1);

	f_status.raw_byte = deviceMMA8451QState.i2cBuffer[0];
	// warpPrint("Count: %d\n", f_status.f_cnt);

	for (uint8_t i = 0; i < f_status.f_cnt; i++)
	{
		/* record every 50th reading (1s period) */
		data = devMMA8451Q_getAccel();
		if (count == 0) {
			int angle = arctan(data.x, data.z);
			angle_buffer[wt_idx++] = angle;
			if (wt_idx == 10) {
				angle_buffer_full = true;
				wt_idx = 0;
			}
		}

		if (i == f_status.f_cnt - 1)
			ret = data.y;

		if (count == 49) count = 0;
		else 			 count++;
	}

	return data.y;
}

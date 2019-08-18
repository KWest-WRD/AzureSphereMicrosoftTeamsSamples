/*
 *  Some of the code in this file was copied from ST Micro.  Below is their required information.
 *
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <azureiot/iothub_device_client_ll.h>

 // applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"

#include <applibs/log.h>
#include <applibs/i2c.h>

#include "mt3620_avnet_dev.h"
#include "build_options.h"
#include "i2c.h"
#include "lsm6dso_reg.h"

/* Private variables ---------------------------------------------------------*/

static axis1bit16_t data_raw_temperature;
static float lsm6dsoTemperature_degC;

static uint8_t whoamI, rst;
static int accelTimerFd = -1;
const uint8_t lsm6dsOAddress = LSM6DSO_ADDRESS;     // Addr = 0x6A
lsm6dso_ctx_t dev_ctx;
//lps22hh_ctx_t pressure_ctx;

//Extern variables
int i2cFd = -1;
extern int epollFd;

//Private functions

// Routines to read/write to the LSM6DSO device
static int32_t platform_write(int* fD, uint8_t reg, uint8_t* bufp, uint16_t len);
static int32_t platform_read(int* fD, uint8_t reg, uint8_t* bufp, uint16_t len);

// Routines to read/write to the LPS22HH device connected to the LSM6DSO sensor hub
//static int32_t lsm6dso_write_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data, uint16_t len);
//static int32_t lsm6dso_read_lps22hh_cx(void* ctx, uint8_t reg, uint8_t* data, uint16_t len);

/// <summary>
///     Sleep for delayTime ms
/// </summary>
void HAL_Delay(int delayTime) {
	struct timespec ts;
	ts.tv_sec = 0;
	ts.tv_nsec = delayTime * 10000;
	nanosleep(&ts, NULL);
}

/// <summary>
///     Print latest data from on-board sensors.
/// </summary>
float GetTemp(void)
{
	uint8_t reg;

	
	// Read the sensors on the lsm6dso device

	lsm6dso_temp_flag_data_ready_get(&dev_ctx, &reg);
	if (reg)
	{
		// Read temperature data
		memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
		lsm6dso_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
		lsm6dsoTemperature_degC = lsm6dso_from_lsb_to_celsius(data_raw_temperature.i16bit);

		Log_Debug("LSM6DSO: Temperature  [degC]: %.2f\r\n", lsm6dsoTemperature_degC);
		return lsm6dsoTemperature_degC;
	}
}

/// <summary>
///     Initializes the I2C interface.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
int initI2c(void) {

	// Begin MT3620 I2C init 

	i2cFd = I2CMaster_Open(MT3620_RDB_HEADER4_ISU2_I2C);
	if (i2cFd < 0) {
		Log_Debug("ERROR: I2CMaster_Open: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	int result = I2CMaster_SetBusSpeed(i2cFd, I2C_BUS_SPEED_STANDARD);
	if (result != 0) {
		Log_Debug("ERROR: I2CMaster_SetBusSpeed: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	result = I2CMaster_SetTimeout(i2cFd, 100);
	if (result != 0) {
		Log_Debug("ERROR: I2CMaster_SetTimeout: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	// Start lsm6dso specific init

	// Initialize lsm6dso mems driver interface
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &i2cFd;

	// Check device ID
	lsm6dso_device_id_get(&dev_ctx, &whoamI);
	if (whoamI != LSM6DSO_ID) {
		Log_Debug("LSM6DSO not found!\n");
		return -1;
	}
	else {
		Log_Debug("LSM6DSO Found!\n");
	}

	// Restore default configuration
	lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
	do {
		lsm6dso_reset_get(&dev_ctx, &rst);
	} while (rst);

	// Disable I3C interface
	lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);

	// Enable Block Data Update
	lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	// Set Output Data Rate
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_12Hz5);
		
	return 0;
}

/// <summary>
///     Closes the I2C interface File Descriptors.
/// </summary>
void closeI2c(void) {

	CloseFdAndPrintError(i2cFd, "i2c");
	CloseFdAndPrintError(accelTimerFd, "accelTimer");
}

/// <summary>
///     Writes data to the lsm6dso i2c device
/// </summary>
/// <returns>0</returns>

static int32_t platform_write(int* fD, uint8_t reg, uint8_t* bufp,
	uint16_t len)
{

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("platform_write()\n");
	Log_Debug("reg: %0x\n", reg);
	Log_Debug("len: %0x\n", len);
	Log_Debug("bufp contents: ");
	for (int i = 0; i < len; i++) {

		Log_Debug("%0x: ", bufp[i]);
	}
	Log_Debug("\n");
#endif 

	// Construct a new command buffer that contains the register to write to, then the data to write
	uint8_t cmdBuffer[len + 1];
	cmdBuffer[0] = reg;
	for (int i = 0; i < len; i++) {
		cmdBuffer[i + 1] = bufp[i];
	}

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("cmdBuffer contents: ");
	for (int i = 0; i < len + 1; i++) {

		Log_Debug("%0x: ", cmdBuffer[i]);
	}
	Log_Debug("\n");
#endif

	// Write the data to the device
	int32_t retVal = I2CMaster_Write(*fD, lsm6dsOAddress, cmdBuffer, (size_t)len + 1);
	if (retVal < 0) {
		Log_Debug("ERROR: platform_write: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}
#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("Wrote %d bytes to device.\n\n", retVal);
#endif
	return 0;
}

/// <summary>
///     Reads generic device register from the i2c interface
/// </summary>
/// <returns>0</returns>

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(int* fD, uint8_t reg, uint8_t* bufp,
	uint16_t len)
{

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("platform_read()\n");
	Log_Debug("reg: %0x\n", reg);
	Log_Debug("len: %d\n", len);
	;
#endif

	// Set the register address to read
	int32_t retVal = I2CMaster_Write(i2cFd, lsm6dsOAddress, &reg, 1);
	if (retVal < 0) {
		Log_Debug("ERROR: platform_read(write step): errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	// Read the data into the provided buffer
	retVal = I2CMaster_Read(i2cFd, lsm6dsOAddress, bufp, len);
	if (retVal < 0) {
		Log_Debug("ERROR: platform_read(read step): errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("Read returned: ");
	for (int i = 0; i < len; i++) {
		Log_Debug("%0x: ", bufp[i]);
	}
	Log_Debug("\n\n");
#endif 	   

	return 0;
}

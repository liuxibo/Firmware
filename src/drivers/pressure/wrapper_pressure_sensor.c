/****************************************************************************
 *
 *   Copyright (C) 2015 Ramakrishna Kintada. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#include <px4_log.h>
#include <bmp280_api.h>

#include "pressure_context.h"
#include "pressure_sensor.h"

static struct pressure_context context;

/**
 * Initialize the open I2C bus with device specific parameters, such as the slave address of the
 * pressure sensor.
 *
 * @param i2c_device_path
 * Indicates the DSPAL device path referencing the particular I2C bus.
 * @param read_reg_func_ptr
 * Not used.  Should be set to NULL.
 * @param write_reg_func_ptr
 * Not used. Should be set to NULL.
 * @param out_handle
 * Out parameter containing an opaque reference to the pressure context.
 * @return
 * -1 - indicates an error
 * 0  - success
 */
int pressure_sensor_open(const char *i2c_device_path,
			 read_reg_func_t read_reg_func_ptr, write_reg_func_t write_reg_func_ptr,
			 struct pressure_context **out_context)
{
	int result = 0;
	uint32_t driver_handle;

	if (out_context == NULL) {
		PX4_WARN("error: parameter value is invalid.");
		return -1;
	}

	result = bmp280_open(i2c_device_path, &driver_handle);

	if (result < 0) {
		PX4_WARN("error: unable to open the pressure sensor driver.");
		return -1;
	}

	/*
	 * Save the driver handle in the pressure sensor context. Only one
	 * instance is supported so the static context is used.
	 */
	context.fildes = driver_handle;

	/* Return the context to the caller as an opaque handle. */
	*out_context = &context;
	return 0;
}

void pressure_sensor_close(struct pressure_context *context)
{
	bmp280_close(context->fildes);
}

int pressure_sensor_read_data(struct pressure_context *context)
{
	return bmp280_get_sensor_data(context->fildes, (struct bmp280_sensor_data *)&context->sensor_data, TRUE);
}

uint32_t pressure_sensor_get_pressure_in_pa(struct pressure_context *context)
{
	return context->sensor_data.pressure_in_pa;
}

float pressure_sensor_get_temperature_in_c(struct pressure_context *context)
{
	return context->sensor_data.temperature_in_c;
}













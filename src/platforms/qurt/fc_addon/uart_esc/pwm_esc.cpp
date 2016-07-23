/****************************************************************************
*
*   Copyright (c) 2016 James Y. Wilson. All rights reserved.
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

#include <fcntl.h>
#include <stdint.h>
#include <dev_fs_lib_pwm.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <sys/ioctl.h>
#include <drivers/drv_hrt.h>

#include "pwm_esc.h"

// Function implemented in uart_esc_main.c
namespace uart_esc
{
	extern void uart_esc_rotate_motors(int16_t *motor_rpm, int num_rotors); // motor re-mapping
};

// singleton instance variable initialization
PwmEsc* PwmEsc::_instance = NULL;
bool PwmEsc::_initialized = false;

// static initializers
char PwmEsc::DEVICE_PATH[] = "/dev/pwm-1";

PwmEsc* PwmEsc::get_instance()
{
	PX4_INFO("PwmEsc::get_instance() called");

	if (_instance == NULL) {
		_instance = new PwmEsc();
	}

	return _instance;
}

PwmEsc::~PwmEsc()
{
	PX4_INFO("PwmEsc:~PwmEsc called");

	if (_initialized) {
		close(_fd);
		_initialized = false;
	}
}

int PwmEsc::initialize(uint32_t period_in_usecs, uint32_t *gpio_ids,
		uint32_t num_gpio_ids, uint32_t min_pulse_width_in_usecs,
		uint32_t max_pulse_width_in_usecs)
{
	PX4_INFO("PwmEsc::initialize being called.");

	if (_initialized) {
		PX4_ERR("PwmEsc is already initialized.");
		return -1;
	}

	int return_value = 0;
	_fd = open(DEVICE_PATH, 0);

	if (_fd > 0) {
		/*
		 * Configure PWM device
		 */
		struct dspal_pwm pwm_gpio[num_gpio_ids];
		struct dspal_pwm_ioctl_signal_definition signal_definition;
		int gpio_index;

		for (gpio_index = 0; gpio_index < num_gpio_ids; gpio_index++) {
			// Define the initial pulse width and number of the GPIO to
			// use for this signal definition.
			pwm_gpio[gpio_index].gpio_id = gpio_ids[gpio_index];
			// Use the minimum pulse width as the starting pulse to arm the
			// motor.
			pwm_gpio[gpio_index].pulse_width_in_usecs = min_pulse_width_in_usecs;
			PX4_INFO("PwmEsc::initialize: Defining pulse width: ID: %d, width: %d",
					pwm_gpio[gpio_index].gpio_id, pwm_gpio[gpio_index].pulse_width_in_usecs);
		}

		// Describe the overall signal and reference the above array.
		signal_definition.num_gpios = num_gpio_ids;
		signal_definition.period_in_usecs = period_in_usecs;
		signal_definition.pwm_signal = &pwm_gpio[0];

		// Send the signal definition to the DSP.
		if (ioctl(_fd, PWM_IOCTL_SIGNAL_DEFINITION, &signal_definition) != 0) {
			PX4_ERR("PwmEsc::initialize signal definition error");
			return_value = -1;
		}

		// Retrieve the shared buffer which will be used below to update the desired
		// pulse width.
		if (ioctl(_fd, PWM_IOCTL_GET_UPDATE_BUFFER, &_update_buffer) != 0) {
			PX4_ERR("PwmEsc::initialize get update buffer");
			return_value = -1;
		}

		// Save the values min/max PWM values specified.
		_minimum_pulse_width_in_usecs = min_pulse_width_in_usecs;
		_maximum_pulse_width_in_usecs = max_pulse_width_in_usecs;
	}

	if (return_value == 0) {
		_initialized = true;
		PX4_INFO("PwmEsc::initialize successful.");
	}
	else
	{
		PX4_ERR("PwmEsc::initialize error");
	}

	return return_value;
}

int PwmEsc::set(float *outputs, int num_escs)
{
	if (!_initialized) {
		return -1;
	}

	int16_t pulses[num_escs];
	int pulse_with_range_in_usecs = _maximum_pulse_width_in_usecs - _minimum_pulse_width_in_usecs;

	for (int esc_index = 0; esc_index < num_escs; esc_index++) {
		pulses[esc_index] =_minimum_pulse_width_in_usecs +
				(uint32_t)((float)pulse_with_range_in_usecs * ((outputs[esc_index] + 1.0) / 2.0));
	}
	uart_esc::uart_esc_rotate_motors(&pulses[0], num_escs);

	// TODO-JYW: TESTING-TESTING: for ESC calibration
//	static hrt_abstime start_calib_time_in_usecs;
//	static hrt_abstime current_calib_time_in_usecs;
//
//	if (start_calib_time_in_usecs == 0) {
//		start_calib_time_in_usecs = hrt_absolute_time();
//	}
//	current_calib_time_in_usecs = hrt_absolute_time();
//
//	for (int esc_index = 0; esc_index < num_escs; esc_index++) {
//		if ((current_calib_time_in_usecs - start_calib_time_in_usecs) > 1000000 * 10) {
//			PX4_INFO("minimum PWM set for ESC calibration");
//			_update_buffer->pwm_signal[esc_index].pulse_width_in_usecs = _minimum_pulse_width_in_usecs;
//		} else {
//			PX4_INFO("maximum PWM set for ESC calibration");
//			_update_buffer->pwm_signal[esc_index].pulse_width_in_usecs = _maximum_pulse_width_in_usecs;
//		}
//	}
	// TODO-JYW: TESTING-TESTING: for ESC calibration

	for (int esc_index = 0; esc_index < num_escs; esc_index++) {
		_update_buffer->pwm_signal[esc_index].pulse_width_in_usecs = pulses[esc_index];
	}

	return 0;
}



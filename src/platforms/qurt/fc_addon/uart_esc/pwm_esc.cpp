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
#include <dev_fs_lib_pwm.h>
#include "pwm_esc.h"

// TODO-JYW: LEFT-OFF: Verify that uart_esc_main.cpp is correctly using this new class and
// then rebuild/retest.

// singleton instance variable initialization
PwmEsc* PwmEsc::_instance = NULL;
PwmEsc* PwmEsc::_initialized = false;

PwmEsc* PwmEsc::get_instance()
{
	FARF(MEDIUM, "PwmEsc::get_instance() called");

	if (_instance == NULL) {
		_instance = new PwmEsc();
	}

	return _instance;
}

PwmEsc::~PwmEsc()
{
	if (_initialized) {
		close(_fd);
		_initialized = false;
	}
}

int PwmEsc::initialize(uint32_t period_in_usecs, uint32_t *gpio_ids,
		uint32_t num_gpio_ids, uint32_t min_pulse_width_in_usecs = 0,
		uint32_t max_pulse_width_in_usecs = 0)
{
	if (_initialized) {
		FARF(HIGH, "PwmEsc is already initialized.");
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
		}

		// Describe the overall signal and reference the above array.
		signal_definition.num_gpios = num_gpio_ids;
		signal_definition.period_in_usecs = period_in_usecs;
		signal_definition.pwm_signal = &pwm_gpio[0];

		// Send the signal definition to the DSP.
		if (ioctl(fd, PWM_IOCTL_SIGNAL_DEFINITION, &signal_definition) != 0) {
			return_value = ERROR;
		}

		// Retrieve the shared buffer which will be used below to update the desired
		// pulse width.
		if (ioctl(fd, PWM_IOCTL_GET_UPDATE_BUFFER, &_update_buffer) != 0) {
			return_value = ERROR;
		}

		// Save the values min/max PWM values specified.
		_minimum_pulse_width_in_usecs = min_pulse_width_in_usecs;
		_maximum_pulse_width_in_usecs = max_pulse_width_in_usecs;
	}

	if (return_value == 0) {
		_initialized = true;
	}

	return return_value;
}

int PwmEsc::set(int16_t *outputs, int num_escs)
{
	if (!_initialized) {
		return -1;
	}

	int pulse_with_range_in_usecs = _maximum_pulse_width_in_usecs - _minimum_pulse_width_in_usecs;

	for (int esc_index = 0; esc_index < num_escs; esc_index++) {
		_update_buffer->pwm_signal[esc_index].pulse_width_in_usecs =
				_minimum_pulse_width_in_usecs +
				(uint32_t)((float)pulse_with_range_in_usecs * ((outputs[esc_index] + 1.0) / 2.0));
	}

	return 0;
}



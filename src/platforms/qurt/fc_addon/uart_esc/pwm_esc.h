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

#pragma once

#include <stdint.h>

class PwmEsc {
public:
	/**
	 * The minimum duration between successive pulses on a given GPIO line, or between GPIO lines.
	 */
	const uint32_t MINIMUM_PULSE_WIDTH_IN_USECS = 20;

	static PwmEsc* get_instance();

	/**
	 * This does nothing but creates a new instance. See initialize() for
	 * the actual intiialization work that needs to be done before operating
	 * the ESCs.
	 */
	PwmEsc();

	/**
	 * Configure the PWM signal and specify the GPIO lines to be used to generate
	 * the pulses.
	 * @param period_in_usecs
	 * The period of all pulses generated, defined in usecs.
	 * @param gpio_ids
	 * A list of GPIO ID's used to generated the PWM signal for each motor, in order from motor 1 to motor n.
	 * The placement of motor 1 to motor n on the vehicle is defined by the flight stack.
	 * @param num_gpiod_ids
	 * The number of GPIO ID's in the gpio_ids parameter.
	 * @param min_pulse_width_in_usecs
	 * Optional parameter that defines the minimum pulse width that the caller will define
	 * using the set() function.  This value defaults to 0 usecs.
	 * @param max_pulse_width_in_usecs
	 * Optional parameter that defines the maximum pulse width that the caller will define
	 * using the set() function.  This value default to the length of the period, minus
	 * the minimum pulse width.
	 * @return
	 * 0 - success
	 * -1 - error
	 */
	int initialize(uint32_t period_in_usecs, uint32_t *gpio_ids,
			uint32_t num_gpio_ids, uint32_t min_pulse_width_in_usecs = 0,
			uint32_t max_pulse_width_in_usecs = 0);

	/**
	 * If this UartEsc instance has been initialized, this destructor closes
	 * the PWM device connected to the ESCs
	 */
	~PwmEsc();

	/**
	 * Scale the specified motor RPM's to PWM settings.
	 * @param rpms
	 * The desired motor RPM of each motor, in order from motor 1 to motor n. The specified
	 * RPM is scaled within the minimum and maximum pulse width defined in initialize().
	 * @param num_esc
	 * The number of motors referenced in the rpms parameter.
	 */
	int set(int16_t *rpms, int num_esc);

private:
	static PwmEsc *_instance;
};

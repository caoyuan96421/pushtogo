/*
 #include <StepOut.h>
 * ControllablePWMOut.cpp
 *
 *  Created on: 2018��2��9��
 *      Author: caoyuan9642
 */

#include <math.h>
#include "StepOut.h"
#include "PeripheralPins.h"

#define MAX_PERIOD 1000000LL

void StepOut::start() {
	if (status == IDLE && freq > 0) // Start only when idle and frequency is not zero
			{
		core_util_critical_section_enter();
		status = STEPPING;
		output_on();
		tim.reset();
		core_util_critical_section_exit();
	}
}

void StepOut::stop() {
	if (status == STEPPING) {
		core_util_critical_section_enter();
		status = IDLE;
		output_off();
		stepCount += (int64_t) (freq * tim.read_high_resolution_us() / 1.0E6);
		core_util_critical_section_exit();
	}
}

double StepOut::setFrequency(double frequency) {
	if (frequency > 0) {
		int64_t us_period = ceil(1.0E6 / frequency); /*Ceil to the next microsecond*/
		if (us_period > MAX_PERIOD) {
			// Prevent overflow
			us_period = MAX_PERIOD;
		}
		if (status == IDLE)
			this->period_us(us_period);
		else {
			core_util_critical_section_enter();
			/*Stop to correctly update the stepCount*/
//			output_off();
			stepCount +=
					(int64_t) (freq * tim.read_high_resolution_us() / 1.0E6);
			this->period_us(us_period);
			this->write(0.5f);
//			output_on();
			tim.reset();
			core_util_critical_section_exit();
		}
		freq = 1.0E6 / us_period; // get CORRECT frequency!
	} else {
		// frequency=0 effectively means stop
		freq = 0;
		if (status == STEPPING) {
			stop();
		}
	}
	return freq; // Return the accurate frequency
}

void StepOut::resetCount() {
	core_util_critical_section_enter();
	stepCount = 0;
	if (status == STEPPING)
		tim.reset();
	core_util_critical_section_exit();
}

int64_t StepOut::getCount() {
	if (status == IDLE)
		return stepCount;
	else {
		// Fixed: use read_high_resolution_us() to prevent overflow every ~30min
		return stepCount
				+ (int64_t) (freq * tim.read_high_resolution_us() / 1.0E6); /*Calculate count at now*/
	}
}

double StepOut::getFrequency() {
	return freq;
}

void StepOut::output_on() {
	pinmap_pinout(_pwm.pin, PinMap_PWM);
//	write(0.5);
}

void StepOut::output_off() {
	pin_function(_pwm.pin, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0));
//	write(0);
}

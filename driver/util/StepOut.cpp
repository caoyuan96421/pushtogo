/*
 #include <StepOut.h>
 * ControllablePWMOut.cpp
 *
 *  Created on: 2018Äê2ÔÂ9ÈÕ
 *      Author: caoyuan9642
 */

#include <math.h>
#include "StepOut.h"
#include "PeripheralPins.h"

#define MAX_PERIOD 1000000LL

void StepOut::start() {
	if (status == IDLE && freq > 0) { // Start only when idle and frequency is not zero
		mutex.lock();
		status = STEPPING;

		core_util_critical_section_enter();
		output_on();
		tim.reset();
		core_util_critical_section_exit();

		mutex.unlock();
	}
}

void StepOut::stop() {
	if (status == STEPPING) {
		int64_t us_time;
		mutex.lock();
		status = IDLE;

		core_util_critical_section_enter();
		us_time = tim.read_high_resolution_us();
		output_off();
		core_util_critical_section_exit();

		stepCount += (int64_t) (freq * us_time / 1.0E6);
		mutex.unlock();
	}
}

double StepOut::setFrequency(double frequency) {
	mutex.lock();
	if (frequency > 0) {
		int64_t us_period = ceil(1.0E6 / frequency); /*Ceil to the next microsecond*/
		if (us_period > MAX_PERIOD) {
			// Prevent overflow
			us_period = MAX_PERIOD;
		}
		if (status == IDLE) {
			this->period_us(us_period);
			this->write(0.5f);
		} else {
			core_util_critical_section_enter();
			uint64_t us_time = tim.read_high_resolution_us();
			this->period_us(us_period);
			this->write(0.5f);
			core_util_critical_section_exit();

			stepCount += (int64_t) (freq * us_time / 1.0E6);
			tim.reset();
		}
		freq = 1.0E6 / us_period; // get CORRECT frequency!
	} else {
		// frequency=0 effectively means stop
		freq = 0;
		if (status == STEPPING) {
			stop();
		}
	}
	mutex.unlock();
	return freq; // Return the accurate frequency
}

void StepOut::resetCount() {
	mutex.lock();
	stepCount = 0;
	if (status == STEPPING)
		tim.reset();
	mutex.unlock();
}

int64_t StepOut::getCount() {
	int64_t count;
	mutex.lock();
	if (status == IDLE)
		count = stepCount;
	else {
		// Fixed: use read_high_resolution_us() to prevent overflow every ~30min
		count = stepCount
				+ (int64_t) (freq * tim.read_high_resolution_us() / 1.0E6); /*Calculate count at now*/
	}
	mutex.unlock();
	return count;
}

double StepOut::getFrequency() {
	return freq;
}

void StepOut::output_on() {
	pinmap_pinout(_pwm.pin, PinMap_PWM);
//	write(0.5f);
}

void StepOut::output_off() {
	pin_function(_pwm.pin, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0));
//	write(0);
}

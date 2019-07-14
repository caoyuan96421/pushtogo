/*
 * ControllablePWMOut.h
 *
 *  Created on: 2018Äê2ÔÂ9ÈÕ
 *      Author: caoyuan9642
 */

#ifndef TELESCOPE_STEPOUT_H_
#define TELESCOPE_STEPOUT_H_

#include "mbed.h"

class StepOut: protected PwmOut
{
public:

	StepOut(PinName pin) :
			PwmOut(pin), stepCount(0), freq(1), status(IDLE)
	{
		// Stop the output
		this->period(1);
		this->write(0);
		tim.start();
	}
	virtual ~StepOut()
	{
		// Stop the PWM Output
		this->write(0);
	}

	void start();
	void stop();

	double setFrequency(double frequency);
	void resetCount();
	int64_t getCount();

private:
	typedef enum
	{
		IDLE = 0, STEPPING
	} stepstatus_t;

	int64_t stepCount;
	double freq;
	stepstatus_t status;
	Timer tim;

};

#endif /* TELESCOPE_STEPOUT_H_ */



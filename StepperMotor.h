/*
 * GenericStepperMotor.h
 *
 *  Created on: 2018Äê2ÔÂ8ÈÕ
 *      Author: caoyuan9642
 */

#ifndef _STEPPERMOTOR_H_
#define _STEPPERMOTOR_H_

#include <stdint.h>
#include "Callback.h"

typedef enum
{
	STEP_FORWARD = 0, STEP_BACKWARD = 1
} stepdir_t;

/**
 * Interface of a generic stepper motor
 */
class StepperMotor
{
protected:
	bool invert;
public:
	StepperMotor(bool invert = false) :
			invert(invert)
	{
	}
	virtual ~StepperMotor()
	{
	}

	/**
	 * Start the stepper motor in the specified direction
	 * @param dir direction to move
	 * @note must be implemented
	 */
	virtual void start(stepdir_t dir) = 0;

	/**
	 * Stop the stepper motor;
	 */
	virtual void stop() = 0;

	/**
	 * Get the (fractional) step count. The unit is full steps, so half step will be 0.5 step and etc
	 * @return step count
	 */
	virtual double getStepCount() = 0;

	/**
	 * Set the current step count to specified value
	 * @param count step count to be set to
	 */
	virtual void setStepCount(double count) = 0;

	/** Set frequency of the stepping. In unit of full steps per second
	 * @param freq Target frequency in full steps per second
	 * @return Actual frequency been set to. In full steps per second.
	 * @note This value should be always SMALLER than the target frequency
	 * when the accurate frequency cannot be achieved
	 */
	virtual double setFrequency(double freq) = 0;

	/**
	 * Get frequency of the stepping. In unit of full steps per second
	 * @return Actual frequency the stepper motor is running
	 */
	virtual double getFrequency() = 0;

	/**
	 * Turn off the motor power
	 */
	virtual void powerOff(){
	}

	/**
	 * Turn on the motor power
	 */
	virtual void powerOn(){
	}

	/**
	 * Set microsteps
	 * @param microstep Microstep setting to use
	 * @note to be overriden by application if available
	 */
	virtual void setMicroStep(int microstep)
	{
	}

	/**
	 * Set drive current
	 * @param current new current to use
	 * @note to be overriden by application if available
	 */
	virtual void setCurrent(double current)
	{
	}

	/**
	 * Set error callback
	 * @param Callback to call when error occurs
	 */
	virtual void setErrorCallback(mbed::Callback<void()> cb){
	}

};

/**
 * Take inverse of the stepping direction
 */
inline stepdir_t operator!(const stepdir_t dir)
{
	if (dir == STEP_FORWARD)
		return STEP_BACKWARD;
	else
		return STEP_FORWARD;
}

#endif /* TELESCOPE_STEPPERMOTOR_H_ */



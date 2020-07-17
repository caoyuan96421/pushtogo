/*
 * TMC2130.h
 *
 *  Created on: Jul 26, 2019
 *      Author: caoyu
 */

#ifndef _TMC2130_H_
#define _TMC2130_H_

#include "mbed.h"
#include "driver/util/StepOut.h"
#include "StepperMotor.h"

class TMC2130: public StepperMotor {
public:
	TMC2130(SPI &spi, PinName step, PinName dir, PinName err=NC, PinName iref=NC, bool invert=false);
	virtual ~TMC2130();

	/**
	 * Start the stepper motor in the specified direction
	 * @param dir direction to move
	 * @note must be implemented
	 */
	virtual void start(stepdir_t dir);

	/**
	 * Stop the stepper motor;
	 */
	virtual void stop();

	/**
	 * Get the (fractional) step count. The unit is full steps, so half step will be 0.5 step and etc
	 * @return step count
	 */
	virtual double getStepCount();

	/**
	 * Set the current step count to specified value
	 * @param count step count to be set to
	 */
	virtual void setStepCount(double count);

	/** Set frequency of the stepping. In unit of full steps per second
	 * @param freq Target frequency in full steps per second
	 * @return Actual frequency been set to. In full steps per second.
	 * @note This value should be always SMALLER than the target frequency
	 * when the accurate frequency cannot be achieved
	 */
	virtual double setFrequency(double freq);

	/**
	 * Get frequency of the stepping. In unit of full steps per second
	 * @return Actual frequency the stepper motor is running
	 */
	virtual double getFrequency();

	/**
	 * Turn off the motor power
	 */
	virtual void powerOff();

	/**
	 * Turn on the motor power
	 */
	virtual void powerOn();

	/**
	 * Set microsteps
	 * @param microstep Microstep setting to use
	 * @note to be overriden by application if available
	 */
	virtual void setMicroStep(int microstep);

	/**
	 * Set drive current
	 * @param current new current to use
	 * @note to be overriden by application if available
	 */
	virtual void setCurrent(double current);

	void debug();

	void setStealthChop(bool enable);

	void recoverFromError(){
		if (status == ERROR){
			status = IDLE;
			powerOn();
		}
	}

	virtual void setErrorCallback(Callback<void()> cb){
		this->cb = cb;
	}

protected:
	SPI &spi;
	StepOut step;
	DigitalOut dir;
	InterruptIn err;
	PwmOut *iref;
	enum stepstatus_t{
		IDLE = 0, STEPPING, ERROR
	} status;
	int inc;
	double stepCount;
	int microstep;
	bool useDir;
	bool useErr;
	bool useIref;

	uint32_t CHOPCONF_VALUE;
	uint32_t GCONF_VALUE;
	uint32_t PWMCONF_VALUE;
	uint32_t IHOLD_IRUN_VALUE;

	EventQueue eq;
	Thread eq_thread;

	Callback<void()> cb;


	uint8_t hw_readwrite(uint8_t addr, uint32_t *data, bool write);

	void hw_writereg(uint8_t addr, uint32_t data);
	uint32_t hw_readreg(uint8_t addr);
	uint8_t hw_status();

	void irq();
	void err_cb();

	void error_stop();
};

#endif /* _TMC2130_H_ */

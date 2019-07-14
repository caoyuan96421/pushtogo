/*
 * AMIS30543StepperDriver.h
 *
 *  Created on: 2018Äê2ÔÂ8ÈÕ
 *      Author: caoyuan9642
 */

#ifndef TELESCOPE_DRIVER_AMIS30543STEPPERDRIVER_H_

#define TELESCOPE_DRIVER_AMIS30543STEPPERDRIVER_H_

#include <StepperMotor.h>
#include <StepOut.h>
#include "mbed.h"

class AMIS30543StepperDriver: public StepperMotor
{

public:
	AMIS30543StepperDriver(SPI *spi, PinName cs, PinName step, PinName dir = NC,
			PinName err = NC, bool invert = false);
	virtual ~AMIS30543StepperDriver();

	void start(stepdir_t dir);

	void stop();

	double setFrequency(double freq);

	double getStepCount();

	void setStepCount(double set)
	{
		stepCount = set;
	}

	void setMicroStep(int microstep);

	void setCurrent(double current);

	void poweron();
	void poweroff();


protected:
	typedef enum
	{
		WR = 0,
		CR0 = 1,
		CR1 = 2,
		CR2 = 3,
		SR0 = 4,
		SR1 = 5,
		SR2 = 6,
		SR3 = 7,
		SR4 = 8,
		CR3 = 9
	} regaddr_t;

	typedef enum
	{
		IDLE = 0, STEPPING
	} stepstatus_t;

	SPI *spi;
	DigitalOut cs;
	StepOut step;
	DigitalOut dir;
	DigitalIn err;
	stepstatus_t status;
	int inc;
	double stepCount;
	int microstep;

	bool useDIR, useERR;

	char txbuf[2], rxbuf[2];

	void assertCS();
	void deassertCS();
	void writeReg(regaddr_t addr, uint8_t data);
	uint8_t readReg(regaddr_t addr);
};

#endif /* TELESCOPE_DRIVER_AMIS30543STEPPERDRIVER_H_ */



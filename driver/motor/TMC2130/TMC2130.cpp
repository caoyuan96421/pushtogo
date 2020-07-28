/*
#include <TMC2130/TMC2130.h>
 * TMC2130.cpp
 *
 *  Created on: Jul 26, 2019
 *      Author: caoyu
 */

#include "mbed.h"
#include "TMC2130.h"
#define TMC_DEBUG 1

#define TMC2130_GCONF			0x00
#define TMC2130_GSTAT			0x01
#define TMC2130_IOIN			0x04
#define TMC2130_IHOLD_IRUN		0x10
#define TMC2130_TPOWERDOWN		0x11
#define TMC2130_TSTEP			0x12
#define TMC2130_TPWMTHRS		0x13
#define TMC2130_TCOOLTHRS		0x14
#define TMC2130_THIGH			0x15
#define TMC2130_XDIRECT			0x2D
#define TMC2130_VDCMIN			0x33
#define TMC2130_MSLUT			0x60	// 0x60-0x67
#define TMC2130_MSLUTSEL		0x68
#define TMC2130_MSLUTSTART		0x69
#define TMC2130_MSCNT			0x6A
#define TMC2130_MSCURACT		0x6B
#define TMC2130_CHOPCONF		0x6C
#define TMC2130_COOLCONF		0x6D
#define TMC2130_DCCTRL			0x6E
#define TMC2130_DRV_STATUS		0x6F
#define TMC2130_PWMCONF			0x70
#define TMC2130_PWM_SCALE		0x71
#define TMC2130_ENCM_CTRL		0x72
#define TMC2130_LOST_STEPS		0x73

// Sense resistor value. Change if using a different one
#define RSENSE	0.33

// Use freewheel or not. 0 to turn off, 1-3 to use FREEWHEEL, LS brake or HS brake
#define FREEWHEEL 0

// Percentage of Hold current
#define IHOLD_PERCENT	0.4

#define IHOLD_DELAY		1

// VSENSE
#define VSENSE	1

// INTERPOLATION to 256-microstep?
#define INTERP	1

// Switch between stealthChop and spreadCycle, fCLK ~ 10MHz, set to 100 is approx. 60 RPM (1 Hz)
#define SWITCH_THD 1000

// stallGuard speed. Only detect stall above this speed. Set to zero to turn off stallGuard
#define SG_SPEED 200

// stallGuard sensitivity, lower value is more sensitive
#define SG_THD	0

TMC2130::TMC2130(SPI &spi, PinName step, PinName dir, PinName err, PinName iref,
		bool invert) :
		StepperMotor(invert), spi(spi), step(step), dir(dir), err(err, PullUp), iref(
		NULL), status(IDLE), inc(1), stepCount(0), eq_thread(
				osPriorityAboveNormal, 1024, NULL, "TMC2130_error_thd"), cb(NULL) {
	useDir = (dir != NC);
	useErr = (err != NC);
	useIref = (iref != NC);

	// Stop the stepping
	this->step.stop();

	if (useDir)
		this->dir = 0;

	// Register values
	CHOPCONF_VALUE = 0x300101C3 | (VSENSE << 17) | (INTERP << 28) | (1 << 30); //DOUBLE edge, TOFF=3, HSTRT=4, HEND=3, TBL=2, CHM=0 (spreadCycle)
	GCONF_VALUE = 0x00000000; // disable stealthChop/
	PWMCONF_VALUE = 0x00050480 | (FREEWHEEL << 20); // PWM_CONF: Default value + Freewheel: LS brake
	IHOLD_IRUN_VALUE = ((FREEWHEEL ? 0 : uint8_t(31 * IHOLD_PERCENT)) << 0)
			| (31 << 8) | (IHOLD_DELAY << 16); //  IHOLD=16, IRUN=31, IHOLDDELAY=5

	// Use Iref?
	if (useIref) {
		// Allocate Iref pin for PWM output
		this->iref = new PwmOut(iref);
		this->iref->period_us(256); // Use 0.256ms period
		this->iref->write(0.666); // Max output
		GCONF_VALUE |= 0x00000001; // Use external I reference from AIN
	}

	// Enable error outputs?
	if (useErr) {
		GCONF_VALUE |= (1 << 6);
		this->err.fall(callback(this, &TMC2130::irq));
	}

	// Configuration
	hw_writereg(TMC2130_GCONF, GCONF_VALUE);
	hw_writereg(TMC2130_PWMCONF, PWMCONF_VALUE);
	hw_writereg(TMC2130_CHOPCONF, CHOPCONF_VALUE);
	hw_writereg(TMC2130_IHOLD_IRUN, IHOLD_IRUN_VALUE);
	hw_writereg(TMC2130_COOLCONF,
			0x01000000 | ((uint32_t) (SG_THD & 0x7F) << 16)); // Enable stallGuard filter
	hw_writereg(TMC2130_TPWMTHRS, SWITCH_THD);
	hw_writereg(TMC2130_TCOOLTHRS, SG_SPEED);
	hw_writereg(TMC2130_TPOWERDOWN, 0x00000004); // 4 cycle before standstill shutdown

	debug_if(TMC_DEBUG, "GCONF: 0x%08x\r\n",
			(unsigned int) hw_readreg(TMC2130_GCONF));
	debug_if(TMC_DEBUG, "DRV_STATUS: 0x%08x\r\n",
			(unsigned int) hw_readreg(TMC2130_DRV_STATUS));
	debug_if(TMC_DEBUG, "CHOPCONF: 0x%08x\r\n",
			(unsigned int) hw_readreg(TMC2130_CHOPCONF));
	debug_if(TMC_DEBUG, "IHOLD_IRUN: 0x%08x\r\n",
			(unsigned int) IHOLD_IRUN_VALUE);

	microstep = 256; // Default to max resolution

	// Start event queue
	eq_thread.start(callback(&eq, &EventQueue::dispatch_forever));
}

TMC2130::~TMC2130() {
	if (iref)
		delete iref;
}

uint8_t TMC2130::hw_readwrite(uint8_t addr, uint32_t *data, bool write) {
	char txbuf[5];
	char rxbuf[5];
	txbuf[0] = (addr & 0x7F) | (write ? 0x80 : 0);
	txbuf[1] = (*data >> 24) & 0xFF;
	txbuf[2] = (*data >> 16) & 0xFF;
	txbuf[3] = (*data >> 8) & 0xFF;
	txbuf[4] = (*data) & 0xFF;
	spi.write(txbuf, 5, rxbuf, 5);

	char status = rxbuf[0];
	*data = rxbuf[4];
	*data |= rxbuf[3] << 8;
	*data |= rxbuf[2] << 16;
	*data |= rxbuf[1] << 24;

	return status;
}

void TMC2130::hw_writereg(uint8_t addr, uint32_t data) {
	hw_readwrite(addr, &data, true);
}

uint32_t TMC2130::hw_readreg(uint8_t addr) {
	uint32_t data = 0xACACACAC; // dummy data to transmit
	hw_readwrite(addr, &data, false); // Send addr
	hw_readwrite(addr, &data, false); // Get data
	return data;
}

void TMC2130::start(stepdir_t dir) {
	if (status == IDLE) {
		if (useDir) {
			/*Hard switch*/
			if ((dir == STEP_FORWARD) ^ invert)
				this->dir = 0;
			else
				this->dir = 1;
		} else {
			/*Soft switch*/
			uint32_t gconf = hw_readreg(TMC2130_GCONF) & ~(1 << 4); // SHAFT
			if ((dir == STEP_FORWARD) ^ invert)
				gconf |= 0;
			else
				gconf |= (1 << 4);
			hw_writereg(TMC2130_GCONF, gconf);
		}

		inc = (dir == STEP_FORWARD) ? 1 : -1;

		// Start stepping
		step.start();
		step.resetCount();
		status = STEPPING;
	}
}

void TMC2130::stop() {
	if (status == STEPPING) {
		status = IDLE;
		step.stop();
		stepCount += ((double) step.getCount()) * 2 * inc / microstep; // the stepCount should be divided by microstep
	}
}

double TMC2130::getStepCount() {
	if (status == IDLE)
		return stepCount;
	else
		return stepCount + ((double) step.getCount()) * 2 * inc / microstep; // the step count should be divided by microstep
}

void TMC2130::setStepCount(double count) {
	step.resetCount();
	stepCount = count;
}

double TMC2130::setFrequency(double frequency) {
	return step.setFrequency(frequency * microstep / 2) / microstep * 2; //Use the microstepping frequency
}

double TMC2130::getFrequency() {
	return step.getFrequency() / microstep * 2; //Use the microstepping frequency
}

void TMC2130::powerOff() {
	uint32_t chopconf = hw_readreg(TMC2130_CHOPCONF);
	hw_writereg(TMC2130_CHOPCONF, (chopconf & ~0x0F));
}

void TMC2130::powerOn() {
	uint32_t chopconf = hw_readreg(TMC2130_CHOPCONF);
	hw_writereg(TMC2130_CHOPCONF, (chopconf & ~0x0F) | (CHOPCONF_VALUE & 0x0F));
}

void TMC2130::setMicroStep(int microstep) {
	uint32_t mres = 0;
	if (microstep & (microstep - 1)) {
		error("Microstep must be 2^n");
	}
	this->microstep = microstep;
	mres = 8;
	while (microstep > 1) {
		microstep >>= 1;
		mres--;
	}
	mres <<= 24;
	uint32_t chopconf = hw_readreg(TMC2130_CHOPCONF);
	hw_writereg(TMC2130_CHOPCONF, (chopconf & ~0x0F000000) | mres);
}

void TMC2130::setCurrent(double current) {
	float maxcurrent = (VSENSE ? 0.18 : 0.32) / (RSENSE + 0.02);
	if (current > maxcurrent)
		current = maxcurrent;
	if (current < 0.1)
		current = 0.1;
	if (useIref) {
		// 0.666 (2.5V on 3.3V MCU) gives maximum current
		// IRUN is always 31 in this mode
		// Calculate peak current using VSENSE
		iref->write(0.666f * (float) current / maxcurrent);
	} else {
		uint32_t irun = (uint32_t) (31 * (float) current / maxcurrent + 0.5)
				& 0x1F; // Maximum is 31
		uint32_t ihold = (FREEWHEEL ? 0 : (uint32_t) (irun * IHOLD_PERCENT)); // 50% for ihold
		IHOLD_IRUN_VALUE = (ihold << 0) | (irun << 8) | (IHOLD_DELAY << 16);
		hw_writereg(TMC2130_IHOLD_IRUN, IHOLD_IRUN_VALUE);
	}
}

uint8_t TMC2130::hw_status() {
	uint32_t data = 0xBDBDBDBD; // dummy data to send
	return hw_readwrite(0, &data, false);
}

void TMC2130::irq() {
	eq.call(this, &TMC2130::err_cb);
}

void TMC2130::setStealthChop(bool enable) {
	// Can only change in standstill
	if (status != IDLE) {
		return;
	}
	if (enable) {
		GCONF_VALUE |= 0x00000004; // enable stealthChop/
//		GCONF_VALUE &= ~0x0000080; // disable stall detection
	} else {
		GCONF_VALUE &= ~0x00000004; // disable stealthChop/
//		GCONF_VALUE |= 0x0000080; // enable stall detection
	}
	hw_writereg(TMC2130_GCONF, GCONF_VALUE);
}

void TMC2130::err_cb() {
	uint32_t drv_stat = hw_readreg(TMC2130_DRV_STATUS);
	debug_if(TMC_DEBUG, "ERR: 0x%08x\r\n", (unsigned) drv_stat);
	uint32_t gstat = hw_readreg(TMC2130_GSTAT);
	debug_if(TMC_DEBUG, "GSTAT: 0x%08x\r\n", (unsigned) gstat);
	if (drv_stat & (1 << 28)) {
		status = ERROR;
		error_stop();
		fprintf(stderr, "TMC2130: Phase B short circuit");
	}
	if (drv_stat & (1 << 27)) {
		status = ERROR;
		error_stop();
		fprintf(stderr, "TMC2130: Phase A short circuit");
	}
	if (drv_stat & ((1 << 26) | (1 << 25))) {
		status = ERROR;
		error_stop();
		fprintf(stderr, "TMC2130: Over temperature");
	}
//	if ((drv_stat & (1 << 24)) && status == STEPPING) {
//		uint32_t tstep = hw_readreg(TMC2130_TSTEP);
//		if (tstep <= SG_SPEED - 20) { // Ignore the range just above SG_SPEED
//			status = ERROR;
//			error_stop();
//			fprintf(stderr, "TMC2130: Motor stalled");
//		}
//	}
//	if ((drv_stat & ((1 << 29) | (1 << 30))) && status == STEPPING) {
//		// Open load flag, need to confirm
//		uint32_t tstep = hw_readreg(TMC2130_TSTEP);
//		if (tstep >= SWITCH_THD) {
//			// stealthChop mode, check PWMSCALE
//			uint32_t pwmscale = hw_readreg(TMC2130_PWM_SCALE);
//			printf("PWMSCALE: 0x%08x\r\n", pwmscale);
//			static int stall_count = 0;
//			if (pwmscale == 255) {
//				// Might be stalled
//				if (++stall_count <= 3) {
//					return; //
//				}
//			} else {
//				stall_count = 0; // Clear it if not stalled
//				return;
//			}
//		}
//		// Generate error otherwise
//		error_stop();
//		fprintf(stderr, "TMC2130: Phase open circuit");
//	}
}

void TMC2130::debug() {
	uint32_t drv_stat = hw_readreg(TMC2130_DRV_STATUS);
	debug_if(TMC_DEBUG, "0x%08x\r\n", (unsigned) drv_stat);
}

void TMC2130::error_stop() {
	status = ERROR;
	powerOff();
	if (cb)
		cb.call();
}

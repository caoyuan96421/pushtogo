/*
 * PEC.h
 *
 *  Created on: Sep 9, 2018
 *      Author: caoyu
 */

#ifndef PUSHTOGO_PEC_H_
#define PUSHTOGO_PEC_H_
#include "mbed.h"

// Maximum correctable error per step, in arcsecond
#define MAX_PEC_VALUE 20

class Axis;

class PEC {
public:
	PEC(Axis &a);
	virtual ~PEC() {
		thread->terminate();
		delete thread;
		delete[] pecData;
	}

	bool isEnabled() const {
		return enabled;
	}

	void setEnabled(bool enabled) {
		this->enabled = enabled;
	}

	int getSize() const {
		return granularity;
	}

	float getPECData(int index) const {
		if (index >= 0 && index < granularity) {
			return pecData[index];
		} else
			return NAN;
	}

	void setPECData(int index, float value) {
		if (index >= 0 && index < granularity && fabsf(value) < MAX_PEC_VALUE) {
			pecData[index] = value;
		}
	}

	float getIndexOffset() const {
		return indexOffset;
	}

	/**
	 * Index offset is the phase on the worm cycle at the reference time for the PEC
	 * When axis.getAngleDeg() equals indexOffset, the first bin of PEC data will be executed, etc.
	 */
	void setIndexOffset(float indexOffset) {
		this->indexOffset = indexOffset;
	}

protected:
	Axis &axis;
	volatile bool enabled;
	Thread *thread;

	float indexOffset;

	float *pecData;
	int granularity;

	void task();
};

#endif /* PUSHTOGO_PEC_H_ */

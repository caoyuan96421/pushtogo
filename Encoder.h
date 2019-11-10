/*
 * GenericAbsEncoder.h
 *
 *	Interface for reading position from an absolute encoder
 *
 *  Created on: 2018/2/7
 *      Author: caoyuan9642
 */

#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <stdint.h>
#include <math.h>
#include "mbed.h"

/**
 * Interface of a generic Absolute Encoder
 */
class Encoder {
public:
	Encoder(uint32_t maxcount) :
			maxCount(maxcount), offset(0), reverse(false) {
	}
	virtual ~Encoder() {
	}

	/**
	 * Get max count of the encoder
	 * @return max count
	 */
	uint32_t getMaxCount() const {
		return maxCount;
	}

	/**
	 * Read angle represented by this encoder in degrees, from -180~180
	 * Offset is substracted, so you can use zero to neutralize the offset and use this function to read out
	 * @return angle in degrees
	 */
	double read() {
		mutex.lock();
		uint32_t cnt = readPos();
		mutex.unlock();
		double pos = remainder((double) (cnt - offset) / maxCount, 1) * 360;
		return reverse ? -pos : pos;
	}

	/**
	 * Zero the encoder to current position, or optionally current position with an displacement
	 */
	void zero(int disp) {
		mutex.lock();
		offset = readPos() + disp;
		mutex.unlock();
	}

	/**
	 * Set counting direction of the encoder. dir=true will reverse the actual counting direction of the hardware
	 */
	void setDirection(bool dir) {
		reverse = dir;
	}

	bool getDirection() {
		return reverse;
	}

//protected:
	/**
	 * Low-level readout of encoder position, must be implemented
	 * Call to this function is protected by mutex, so the api is thread safe
	 * @return Encoder count
	 */
	virtual uint32_t readPos() = 0;

private:
	uint32_t maxCount;
	uint32_t offset;
	bool reverse;
	Mutex mutex;
};

// Gray encoder, an extension of Encoder interface with function to translate Gray code into binary
// @param N number of bits of the encoder
template<uint8_t N>
class GrayAbsEncoder: public Encoder {
public:
	GrayAbsEncoder() :
			Encoder(1 << N) {
	}
	virtual ~GrayAbsEncoder() {
	}
//protected:
	/**
	 * Low-level readout of position
	 */
	virtual uint32_t readPosGray() = 0;

	/**
	 * Read the count of the encoder, converting Gray code to binary code
	 */
	virtual uint32_t readPos() {
		/*Convert Gray code to Binary code*/
		uint32_t grayPos = readPosGray();
		uint32_t binPos = grayPos;
		for (grayPos >>= 1; grayPos; grayPos >>= 1)
			binPos ^= grayPos;
		return binPos;
	}

	/**
	 * @return Number of bits
	 */
	uint8_t getBits() const {
		return N;
	}
};

#endif /* _ENCODER_H_ */


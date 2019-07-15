/*
 * GenericAbsEncoder.h
 *
 *	Interface for reading position from an absolute encoder
 *
 *  Created on: 2018/2/7
 *      Author: caoyuan9642
 */

#ifndef _ABSENCODER_H_
#define _ABSENCODER_H_

#include <stdint.h>
/**
* Interface of a generic Absolute Encoder
*/
template<uint32_t maxCount>
class AbsEncoder
{
public:
	AbsEncoder(){
	}
	virtual ~AbsEncoder(){
	}

	virtual uint32_t readPos() = 0;
	virtual void zero()
	{
	}

	uint32_t getMaxCount() const
	{
		return maxCount;
	}
};

#endif /* _ABSENCODER_H_ */



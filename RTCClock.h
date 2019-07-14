#ifndef RTCCLOCK_H_
#define RTCCLOCK_H_

#include "mbed.h"
#include "UTCClock.h"
/**
 * RTCClock class implements the UTCClock interface and provides time through the MBED interface to hardware RTC found on most ARM MCUs
 */
class RTCClock: public UTCClock
{
protected:
	time_t t;
public:

	RTCClock()
	{
		time(&t);
	}
	~RTCClock()
	{
	}

	time_t getTime()
	{
		time(&t);
		return t;
	}

	void setTime(time_t newtime)
	{
		set_time(newtime);
		t = newtime;
	}

	static RTCClock& getInstance()
	{
		static RTCClock clock;
		return clock;
	}
};

#endif /*RTCCLOCK_H_*/


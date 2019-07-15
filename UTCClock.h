#include <time.h>

#ifndef _UTCCLOCK_H_
#define _UTCCLOCK_H_
/**
 * This class provides an interface to a general clock that provides information about the current time.
 * In most systems, this class is implemented by RTCClock to read time from the RTC.
 */
class UTCClock {
public:
	UTCClock() {
	}
	virtual ~UTCClock() {
	}

	/** @return current UTC time in Unix timestamp format
	 */
	virtual time_t getTime() = 0;

	/** @return current UTC time in high resolution (fraction of seconds)
	 */
	virtual double getTimeHighResolution() {
		return (double) getTime();
	}

	/** Set system time
	 */
	virtual void setTime(time_t time) = 0;

};

#endif /*UTCCLOCK_H_*/


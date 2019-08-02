/*
 * Axis.h
 *
 *  Created on: 2018��2��24��
 *      Author: caoyuan9642
 */

#ifndef _AXIS_H_
#define _AXIS_H_
#include "StepperMotor.h"
#include <math.h>
#include "mbed.h"
#include "CelestialMath.h"
#include "config/TelescopeConfiguration.h"
#include "PEC.h"

//#define AXIS_SLEW_SIGNAL				0x00010000
#define AXIS_GUIDE_SIGNAL				0x00020000
#define AXIS_STOP_SIGNAL				0x00040000
#define AXIS_EMERGE_STOP_SIGNAL			0x00080000
#define AXIS_STOP_KEEPSPEED_SIGNAL		0x00100000
#define AXIS_SPEEDCHANGE_SIGNAL			0x00200000

/**
 * status of the Axis object
 */
typedef enum
{
	AXIS_STOPPED = 0, AXIS_SLEWING, AXIS_TRACKING, AXIS_INERTIAL
} axisstatus_t;

typedef enum
{
	AXIS_NOT_SLEWING,
	AXIS_SLEW_ACCELERATING,
	AXIS_SLEW_CONSTANT_SPEED,
	AXIS_SLEW_DECELERATING
} axisslewstate_t;

/** Define the rotation direction
 * AXIS_ROTATE_POSITIVE: +angle
 * AXIS_ROTATE_NEGATIVE: -angle
 */
typedef enum
{
	AXIS_ROTATE_STOP = 0, AXIS_ROTATE_POSITIVE = 1, AXIS_ROTATE_NEGATIVE = 2
} axisrotdir_t;

typedef enum
{
	FINISH_COMPLETE = 0,
	FINISH_STOPPED = 1,
	FINISH_EMERG_STOPPED = 2,
	FINISH_ERROR = 4
} finishstate_t;

class PEC;

/** General Rotating Axis class
 * Handles low-level stepper timing, calculates the speed and distance to rotate
 * API provides comprehensive slewing, tracking and guiding.
 */
class Axis
{
public:

	/**
	 * Create a RotationAxis object
	 * @param stepsPerDeg Steps per degree of the stepper motor
	 * @param stepper Pointer to stepper driver to use
	 * @param name Name of the axis, for example "RA" or "DEC"
	 */
	Axis(double stepsPerDeg, StepperMotor *stepper, const char *name = "Axis");

	virtual ~Axis();

	friend class PEC;

	/**
	 * Asynchronously perform a goto to a specified angle (in degrees) in the specified direction with slewing rate
	 * @param dir Rotation direction
	 * @param angleDeg Angle to rotate
	 * @param withCorrection Use correction or not. If false, the mount will roughly slew to the target. A further correction step is needed (slewed again) if precision positioning is needed. Useful
	 * for long slews where the target would have changed during the slew.
	 * @return osStatus
	 */
	osStatus startSlewTo(axisrotdir_t dir, double angle, bool withCorrection =
	true)
	{
		msg_t *message = task_pool.alloc();
		if (!message)
		{
			return osErrorNoMemory;
		}
		message->signal = msg_t::SIGNAL_SLEW_TO;
		message->value = angle;
		message->dir = dir;
		message->withCorrection = withCorrection;
		osStatus s;

		debug_if(0, "%s: CLR SLEW 0x%08x\n", axisName, ThisThread::get_id());
		slew_finish_sem.try_acquire(); // Make sure the semaphore is cleared. THIS MUST BE DONE BEFORE THE MESSAGE IS ENQUEUED

		if ((s = task_queue.put(message)) != osOK)
		{
			task_pool.free(message);
			return s;
		}

		return osOK;
	}

	/**
	 * Wait for a slew to finish. Must be called after and only once after a call to startSlewTo, from the same thread
	 * @return Finish state of the slew, signaling whether there are errors or being stopped/emergency stopped
	 */
	finishstate_t waitForSlew()
	{
		debug_if(0, "%s: WAIT SLEW 0x%08x\n", axisName, ThisThread::get_id());
		slew_finish_sem.acquire();
		// Check mount status
		return slew_finish_state;
	}

	/** 
	 * Synchronously perform a goto to a specified angle (in degrees) in the specified direction with slewing rate
	 * It will perform an acceleration, a GoTo, and a deceleration before returning
	 * @param dir Rotation direction
	 * @param angleDeg Angle to rotate
	 * @return osStatus
	 * @sa{startSlewTo}
	 */
	osStatus slewTo(axisrotdir_t dir, double angle)
	{
		osStatus s;
		if ((s = startSlewTo(dir, angle)) != osOK)
			return s;
		return waitForSlew();
	}

	/** Perform a indefinite slewing, until stop() is called
	 * @param dir Direction to start continuous slewing
	 * @return osStatus
	 * @sa{stop}
	 */
	osStatus startSlewingIndefinite(axisrotdir_t dir)
	{
		msg_t *message = task_pool.alloc();
		if (!message)
		{
			return osErrorNoMemory;
		}
		message->signal = msg_t::SIGNAL_SLEW_INDEFINITE;
		message->dir = dir;
		message->withCorrection = false;
		osStatus s;
		if ((s = task_queue.put(message)) != osOK)
		{
			task_pool.free(message);
			return s;
		}

		return osOK;
	}

	/** Start tracking, until stop() is called
	 * @param dir Direction to start continuous slewing
	 * @return osStatus
	 * @sa{RotationAxis::stop}
	 */
	osStatus startTracking(axisrotdir_t dir)
	{
		msg_t *message = task_pool.alloc();
		if (!message)
		{
			return osErrorNoMemory;
		}
		message->signal = msg_t::SIGNAL_TRACK;
		message->dir = dir;
		osStatus s;
		if ((s = task_queue.put(message)) != osOK)
		{
			task_pool.free(message);
			return s;
		}

		return osOK;
	}

	/**
	* Guide on specified direction for specified time
	* @param dir Direction of guiding
	* @param time_ms Time to guide in milliseconds
	* @return osStatus
	*/
	osStatus guide(axisrotdir_t dir, int time_ms)
	{
		if (dir == AXIS_ROTATE_NEGATIVE)
			time_ms = -time_ms;
		// Put the guide pulse into the queue
		osStatus s;
		if ((s = guide_queue.put((void*) (time_ms))) != osOK)
		{
			return s;
		}
		task_thread->flags_set(AXIS_GUIDE_SIGNAL); // Signal the task thread to read the queue
		return osOK;
	}

	/**
	 * Remove all queued commands if there are any. This function should be called if you want to ensure the mount if completely stopped
	 */
	void flushCommandQueue()
	{
		while (!task_queue.empty())
		{
			osEvent evt = task_queue.get(0);
			if (evt.status == osEventMessage)
			{
				task_pool.free((msg_t*) evt.value.p);
			}
		}
	}

	/**
	 * Stop slewing or tracking. Calling this function will stop the axis from slewing, or tracking
	 * @note In the case of slewing, the axis will perform a deceleration and then stop
	 * @note If there are queued commands, they will be run immediately afterwards
	 */
	void stop()
	{
		flushCommandQueue();
		task_thread->flags_set(AXIS_STOP_SIGNAL);
	}

	/**
	 * Perform an emergency stop. This should stop in ALL situations IMMEDIATELY without performing deceleration.
	 * @note this call will kill all queued commands, so the mount will be fully stopped
	 */
	void emergency_stop()
	{
		flushCommandQueue();
		task_thread->flags_set(AXIS_EMERGE_STOP_SIGNAL);
	}

	/**
	 * Only takes effect when decelerating from a slew. Signals the axis to keep its current speed, and enters AXIS_INERTIAL state.
	 * This state can be exited by performing another slew/slew_indefinite and stopped in the normal way
	 */
	void stopKeepSpeed()
	{
		task_thread->flags_set(AXIS_STOP_KEEPSPEED_SIGNAL);
	}

	/** Set current angle of the axis in degrees.
	 * @param new angle
	 * @note Must be called only when the axis is stopped, otherwise behavior is unexpected
	 */
	void setAngleDeg(double angle)
	{
		stepper->setStepCount(angle * stepsPerDeg);
	}

	/** Returns the current angle of the axis in degrees
	 * @note Can be called in any context
	 */
	double getAngleDeg()
	{
		return remainder(stepper->getStepCount() / stepsPerDeg, 360);
	}

	/** Returns the axis status
	*/
	axisstatus_t getStatus()
	{
		return status;
	}

	/** Returns the current slew speed (only for indefinite slew)
	 */
	double getSlewSpeed() const
	{
		return slewSpeed;
	}

	/** Set slew speed of the axis (only for indefinite slew)
	 * @param new slew speed in deg/s
	 * @note Can now be called when a indefinite slew is in progress. 
	 * @note If called during a target slew, the speed will be updated on the next slew
	 */
	void setSlewSpeed(double slewSpeed)
	{
		if (slewSpeed > 0)
			this->slewSpeed = slewSpeed;

		task_thread->flags_set(AXIS_SPEEDCHANGE_SIGNAL); // Signal the thread to use the new speed during a indefinite slew
	}

	/**
	* Returns track speed of the axis in units of sidereal speed
	*/
	double getTrackSpeedSidereal() const
	{
		return trackSpeed / sidereal_speed;
	}

	/** Set track speed in sidereal speed 
	 * @param new track speed in sidereal rate
	 * @note Must be called only when the axis is stopped
	 */
	void setTrackSpeedSidereal(double trackSpeed)
	{
		if (trackSpeed >= 0)
		{
			this->trackSpeed = trackSpeed * sidereal_speed;
		}
	}

	/**
	* Returns guide speed of the axis in units of sidereal speed
	*/
	double getGuideSpeedSidereal() const
	{
		return guideSpeed / sidereal_speed;
	}

	/** @param new guide speed in sidereal rate.
	 * @note If called when a pulse guide is being performed, the value will be updated on the next pulse
	 */
	void setGuideSpeedSidereal(double guideSpeed)
	{
		if (guideSpeed > 0)
			this->guideSpeed = guideSpeed * sidereal_speed;
	}

	/**
	* Returns current speed (deg/s) of the axis
	*/
	double getCurrentSpeed() const
	{
		return currentSpeed;
	}

	/**
	* Returns which phase of slewing the axis is currently in
	*/
	axisslewstate_t getSlewState() const
	{
		return slewState;
	}

	/**
	* Returns current axis rotation direction
	*/
	axisrotdir_t getCurrentDirection() const
	{
		return currentDirection;
	}
	
	/**
	* Return true if a guiding pulse is being performed
	*/
	bool isGuiding() const
	{
		return guiding;
	}

	PEC* getPEC(){
		return pec;
	}

	void setPEC(PEC* pec) {
		this->pec = pec;
	}

	bool isPECEnabled() const {
		return pecEnabled;
	}

	void setPECEnabled(bool pecEnabled) {
		this->pecEnabled = pecEnabled;
	}

protected:

	typedef struct
	{
		enum sig_t
		{
			SIGNAL_SLEW_TO = 0, SIGNAL_SLEW_INDEFINITE, SIGNAL_TRACK
		} signal;
		double value;
		axisrotdir_t dir;bool withCorrection;
	} msg_t; /// Message for inter-thread communication

	/*Configurations*/
	double stepsPerDeg; ///steps per degree
	StepperMotor *stepper; ///Pointer to stepper motor
	const char *axisName; /// Name of the axis
	char *taskName; /// Name of the thread

	/*Runtime values*/
	volatile double currentSpeed; /// Current speed in deg/s
	volatile axisrotdir_t currentDirection; // Current direction
	double slewSpeed; /// Slewing speed in deg/s (only for indefinite slew)
	double trackSpeed; /// Tracking speed in deg/s (no accel/deceleration)
	double guideSpeed; /// Guide speed in deg/s. this amount will be subtracted/added to the trackSpeed
	volatile axisstatus_t status; /// State of the axis
	volatile axisslewstate_t slewState; /// Phase of slewing
	Thread *task_thread; ///Thread for executing all lower-level tasks
	Queue<msg_t, 16> task_queue; ///Queue of messages
	Queue<void, 16> guide_queue; ///Guide pulse queue
	MemoryPool<msg_t, 16> task_pool; ///MemoryPool for allocating messages
	Semaphore slew_finish_sem; /// Semaphore for signaling finished slew
	volatile finishstate_t slew_finish_state; /// Finish state of slew
	Timer tim; /// Timer
	bool guiding; /// isGuiding

	PEC *pec; /// PEC
	bool pecEnabled; /// Is pec enabled

	void task(); /// Task thread entrance

	/*Low-level functions for internal use*/
	void slew(axisrotdir_t dir, double dest, bool indefinite, bool useCorrection);
	void track(axisrotdir_t dir);

	/*These functions can be overriden to provide mode selection before each type of operation is performed, such as microstepping and current setting*/
	
	/// Change low-level mode before slewing
	virtual void slew_mode()
	{
	}
	
	/// Change low-level mode before tracking
	virtual void track_mode()
	{
	}
	
	/// Change low-level mode before correcting
	virtual void correction_mode()
	{
	}
	
	/// Change low-level mode before going idle
	virtual void idle_mode()
	{
	}

	/// Stop the mount in case of error
	virtual void err_cb();
}
;

#endif /* _AXIS_H_ */


/*
 * Axis.cpp
 *
 *  Created on: 2018��2��24��
 *      Author: caoyuan9642
 */

#include <Axis.h>

#define AXIS_DEBUG 1

static inline double min(double x, double y) {
	return (x < y) ? x : y;
}
Axis::Axis(double stepsPerDeg, StepperMotor *stepper, const char *name) :
		stepsPerDeg(stepsPerDeg), stepper(stepper), axisName(name), currentSpeed(
				0), currentDirection(AXIS_ROTATE_POSITIVE), slewSpeed(
		MBED_CONF_PUSHTOGO_DEFAULT_SLEW_SPEED), trackSpeed(
		MBED_CONF_PUSHTOGO_DEFAULT_TRACK_SPEED_SIDEREAL * sidereal_speed), guideSpeed(
		MBED_CONF_PUSHTOGO_DEFAULT_GUIDE_SPEED_SIDEREAL * sidereal_speed), status(
				AXIS_STOPPED), slewState(AXIS_NOT_SLEWING), slew_finish_sem(0,
				1), slew_finish_state(FINISH_COMPLETE), guiding(false), pec(
		NULL), pecEnabled(false) {
	if (stepsPerDeg <= 0)
		error("Axis: steps per degree must be > 0");

	if (!stepper)
		error("Axis: stepper must be defined");

	taskName = new char[strlen(name) + 10];
	strcpy(taskName, name);
	strcat(taskName, " task");
	/*Start the task-handling thread*/
	task_thread = new Thread(osPriorityAboveNormal,
	OS_STACK_SIZE, NULL, taskName);
	task_thread->start(callback(this, &Axis::task));
	tim.start();
	// Register error callback
	stepper->setErrorCallback(callback(this, &Axis::err_cb));
}

Axis::~Axis() {
// Wait until the axis is stopped
	if (status != AXIS_STOPPED) {
		stop();
		while (status != AXIS_STOPPED) {
			ThisThread::sleep_for(100);
		}
	}

// Terminate the task thread to prevent illegal access to destroyed objects.
	task_thread->terminate();
	delete task_thread;
	delete taskName;
}

void Axis::task() {

	/*Main loop of RotationAxis*/
	while (true) {
		/*Get next message*/
		msg_t *message;
		enum msg_t::sig_t signal;
		float value;
		axisrotdir_t dir;
		bool wc;

		// Wait for next message
		osEvent evt = task_queue.get();
		if (evt.status == osEventMessage) {
			/*New message arrived. Copy the data and free is asap*/
			message = (msg_t*) evt.value.p;
			signal = message->signal;
			value = message->value;
			dir = message->dir;
			wc = message->withCorrection;
			task_pool.free(message);
		} else {
			/*Error*/
			debug("%s: Error fetching the task queue.\n", axisName);
			continue;
		}
		debug_if(AXIS_DEBUG, "%s: MSG %d %f %d 0x%8x\n", axisName, signal,
				value, dir);

		/*Check the type of the signal, and start corresponding operations*/
		switch (signal) {
		case msg_t::SIGNAL_SLEW_TO:
			if (status == AXIS_STOPPED) {
				slew(dir, value, false, wc);
			} else {
				debug("%s: being slewed while not in STOPPED mode.\n",
						axisName);
			}
			debug_if(0, "%s: SIG SLEW 0x%08x\n", axisName, Thread::gettid());
			slew_finish_sem.release(); /*Send a signal so that the caller is free to run*/
			break;
		case msg_t::SIGNAL_SLEW_INDEFINITE:
			if (status == AXIS_STOPPED || status == AXIS_INERTIAL) {
				slew(dir, 0.0, true, false);
			} else {
				debug("%s: being slewed while not in STOPPED mode.\n",
						axisName);
			}
			break;
		case msg_t::SIGNAL_TRACK:
			if (status == AXIS_STOPPED) {
				track(dir);
			} else {
				debug("%s: trying to track while not in STOPPED mode.\n",
						axisName);
			}
			break;
		default:
			debug("%s: undefined signal %d\n", axisName, message->signal);
		}
	}
}

void Axis::slew(axisrotdir_t dir, double dest, bool indefinite,
		bool useCorrection) {
	if (!indefinite && (isnan(dest) || isinf(dest))) {
		debug("%s: invalid angle.\n", axisName);
		return;
	}
	if (dir == AXIS_ROTATE_STOP) {
		debug("%s: skipped.\n", axisName);
		return;
	}

	slew_mode(); // Switch to slew mode
	ThisThread::flags_clear(
	AXIS_STOP_SIGNAL | AXIS_EMERGE_STOP_SIGNAL | AXIS_SPEEDCHANGE_SIGNAL); // Clear flags
	bool isInertial = (status == AXIS_INERTIAL);
	status = AXIS_SLEWING;
	slewState = AXIS_NOT_SLEWING;
	slew_finish_state = FINISH_COMPLETE;
	currentDirection = dir;
	stepdir_t sd = (dir == AXIS_ROTATE_POSITIVE) ? STEP_FORWARD : STEP_BACKWARD;

	/* Calculate the angle to rotate*/
	bool skip_slew = false;
	double angleDeg = getAngleDeg();
	double delta;
	delta = (dest - angleDeg) * (dir == AXIS_ROTATE_POSITIVE ? 1 : -1); /*delta is the actual angle to rotate*/
	if (fabs(delta) < 1e-5) { // FIX: delta 0->360degrees when angleDeg is essentially equal to dest
		delta = 0;
	}
	delta = remainder(delta - 180.0, 360.0) + 180.0; /*Shift to 0-360 deg*/
	debug_if(AXIS_DEBUG, "%s: start=%f, end=%f, delta=%f\n", axisName, angleDeg,
			dest, delta);

	double startSpeed = 0;
	double endSpeed = slewSpeed, waitTime;
	unsigned int ramp_steps;
	double acceleration = TelescopeConfiguration::getDouble("acceleration");

	if (!indefinite) {
		// Use the GOTO speed from configuration for slewing
		endSpeed = TelescopeConfiguration::getDouble("goto_slew_speed");
		// Ensure that delta is more than the minimum slewing angle, calculate the correct endSpeed and waitTime
		if (delta > MBED_CONF_PUSHTOGO_MIN_SLEW_ANGLE) {
			/*The motion angle is decreased to ensure the correction step is in the same direction*/
			delta = delta - 0.5 * MBED_CONF_PUSHTOGO_MIN_SLEW_ANGLE;

			double angleRotatedDuringAcceleration;

			/* The actual endSpeed we get might be different than this due to the finite time resolution of the stepper driver
			 * Here we set the dummy frequency and obtain the actual frequency it will be set to, so that the slewing time will be more accurate
			 */

			// Calculate the desired endSpeed. If delta is small, then endSpeed will correspondingly be reduced
			endSpeed = min(sqrt(delta * acceleration),
					stepper->setFrequency(stepsPerDeg * endSpeed)
							/ stepsPerDeg);
			ramp_steps = (unsigned int) (endSpeed
					/ (MBED_CONF_PUSHTOGO_ACCELERATION_STEP_TIME / 1000.0)
					/ acceleration);
			if (ramp_steps < 1)
				ramp_steps = 1;

			angleRotatedDuringAcceleration = 0;
			/*Simulate the slewing process to get an accurate estimate of the actual angle that will be slewed*/
			for (unsigned int i = 1; i <= ramp_steps; i++) {
				double speed = stepper->setFrequency(
						stepsPerDeg * endSpeed / ramp_steps * i) / stepsPerDeg;
				angleRotatedDuringAcceleration += speed
						* (MBED_CONF_PUSHTOGO_ACCELERATION_STEP_TIME / 1000.0)
						* (i == ramp_steps ? 1 : 2); // Count both acceleration and deceleration
			}

			waitTime = (delta - angleRotatedDuringAcceleration) / endSpeed;
			if (waitTime < 0.0)
				waitTime = 0.0; // With the above calculations, waitTime should no longer be zero. But if it happens to be so, let the correction do the job

			debug_if(AXIS_DEBUG, "%s: endspeed = %f deg/s, time=%f, acc=%f\n",
					axisName, endSpeed, waitTime, acceleration);
		} else {
			// Angle difference is too small, skip slewing
			skip_slew = true;
		}
	} else {
		// Indefinite slewing mode
		waitTime = INFINITY;
		// If was in inertial mode, use startSpeed
		if (isInertial)
			startSpeed = currentSpeed;
		// No need for correction
		useCorrection = false;
	}

	/*Slewing -> accel, wait, decel*/
	if (!skip_slew) {
		int wait_ms;
		uint32_t flags;
		/*Acceleration*/
		slewState = AXIS_SLEW_ACCELERATING;
		ramp_steps = (unsigned int) ((endSpeed - startSpeed)
				/ (MBED_CONF_PUSHTOGO_ACCELERATION_STEP_TIME / 1000.0)
				/ acceleration);

		if (ramp_steps < 1)
			ramp_steps = 1;

		debug_if(AXIS_DEBUG, "%s: accelerate in %d steps\n", axisName,
				ramp_steps); // TODO: DEBUG

		for (unsigned int i = 1; i <= ramp_steps; i++) {
			currentSpeed = stepper->setFrequency(
					stepsPerDeg
							* ((endSpeed - startSpeed) / ramp_steps * i
									+ startSpeed)) / stepsPerDeg; // Set and update currentSpeed with actual speed

			if (i == 1)
				stepper->start(sd);

			/*Monitor whether there is a stop/emerge stop signal*/
			uint32_t flags = osThreadFlagsWait(
			AXIS_STOP_SIGNAL | AXIS_EMERGE_STOP_SIGNAL, osFlagsWaitAny,
			MBED_CONF_PUSHTOGO_ACCELERATION_STEP_TIME);

			if (flags == osFlagsErrorTimeout) {
				/*Nothing happened, we're good*/
				continue;
			} else if ((flags & osFlagsError) == 0) {
				// We're stopped!
				useCorrection = false;
				if (flags & AXIS_EMERGE_STOP_SIGNAL) {
					slew_finish_state = FINISH_EMERG_STOPPED;
					goto emerge_stop;
				} else if (flags & AXIS_STOP_SIGNAL) {
					slew_finish_state = FINISH_STOPPED;
					goto stop;
				}
			}
		}

		/*Keep slewing and wait*/
		slewState = AXIS_SLEW_CONSTANT_SPEED;
		debug_if(AXIS_DEBUG, "%s: wait for %f\n", axisName, waitTime); // TODO
		wait_ms = (isinf(waitTime)) ? osWaitForever : (int) (waitTime * 1000);

		tim.reset();

		while (isinf(waitTime) || wait_ms > 0) {
			flags = osThreadFlagsWait(
					AXIS_STOP_SIGNAL | AXIS_EMERGE_STOP_SIGNAL
							| (indefinite ? AXIS_SPEEDCHANGE_SIGNAL : 0),
					osFlagsWaitAny, wait_ms); /*Wait the remaining time*/
			if (flags != osFlagsErrorTimeout) {
				if (flags & AXIS_EMERGE_STOP_SIGNAL) {
					slew_finish_state = FINISH_EMERG_STOPPED;
					useCorrection = false;
					goto emerge_stop;
				} else if (flags & AXIS_STOP_SIGNAL) {
					slew_finish_state = FINISH_STOPPED;
					useCorrection = false;
					goto stop;
				} else if (flags & AXIS_SPEEDCHANGE_SIGNAL) {
					// Change speed, therefore also changing waittime. Only applies to indefinite slew
					double newSpeed = slewSpeed;
					startSpeed = currentSpeed;
					endSpeed = newSpeed;

					ramp_steps = (unsigned int) (fabs(endSpeed - startSpeed)
							/ (MBED_CONF_PUSHTOGO_ACCELERATION_STEP_TIME
									/ 1000.0) / acceleration);

					if (ramp_steps < 1)
						ramp_steps = 1;

					debug_if(AXIS_DEBUG, "%s: accelerate in %d steps\n",
							axisName, ramp_steps); // TODO: DEBUG

					for (unsigned int i = 1; i <= ramp_steps; i++) {
						currentSpeed = stepper->setFrequency(
								stepsPerDeg
										* ((endSpeed - startSpeed) / ramp_steps
												* i + startSpeed))
								/ stepsPerDeg; // Set and update currentSpeed with actual speed

						/*Monitor whether there is a stop/emerge stop signal*/
						uint32_t flags = osThreadFlagsWait(
						AXIS_STOP_SIGNAL | AXIS_EMERGE_STOP_SIGNAL,
						osFlagsWaitAny,
						MBED_CONF_PUSHTOGO_ACCELERATION_STEP_TIME);

						if (flags == osFlagsErrorTimeout) {
							/*Nothing happened, we're good*/
							continue;
						} else if ((flags & osFlagsError) == 0) {
							// We're stopped!
							useCorrection = false;
							if (flags & AXIS_EMERGE_STOP_SIGNAL) {
								slew_finish_state = FINISH_EMERG_STOPPED;
								goto emerge_stop;
							} else if (flags & AXIS_STOP_SIGNAL) {
								slew_finish_state = FINISH_STOPPED;
								goto stop;
							}
						}
					}
				}
			}
			if (!indefinite) {
				wait_ms -= tim.read_ms();
				tim.reset();
			}
		}

		stop:
		/*Now deceleration*/
		slewState = AXIS_SLEW_DECELERATING;
		endSpeed = currentSpeed;
		ramp_steps = (unsigned int) (currentSpeed
				/ (MBED_CONF_PUSHTOGO_ACCELERATION_STEP_TIME / 1000.0)
				/ acceleration);

		if (ramp_steps < 1)
			ramp_steps = 1;

		debug_if(AXIS_DEBUG, "%s: decelerate in %d steps from %f\n", axisName,
				ramp_steps, endSpeed); // TODO: DEBUG

		for (unsigned int i = ramp_steps - 1; i >= 1; i--) {
			currentSpeed = stepper->setFrequency(
					stepsPerDeg * endSpeed / ramp_steps * i) / stepsPerDeg; // set and update accurate speed
			// Wait. Now we only handle EMERGENCY STOP signal, since stop has been handled already
			flags = osThreadFlagsWait(
			AXIS_EMERGE_STOP_SIGNAL | AXIS_STOP_KEEPSPEED_SIGNAL,
			osFlagsWaitAny,
			MBED_CONF_PUSHTOGO_ACCELERATION_STEP_TIME);

			if (flags != osFlagsErrorTimeout) {
				if (flags & AXIS_EMERGE_STOP_SIGNAL) {
					// We're stopped!
					useCorrection = false;
					slew_finish_state = FINISH_EMERG_STOPPED;
					goto emerge_stop;
				} else if (flags & AXIS_STOP_KEEPSPEED_SIGNAL) {
					// Keep current speed
					status = AXIS_INERTIAL;
					slewState = AXIS_NOT_SLEWING;
					return;
				}
			}
		}

		emerge_stop:
		/*Fully pull-over*/
		slewState = AXIS_NOT_SLEWING;
		stepper->stop();
		currentSpeed = 0;
	}

	if (useCorrection) {
		// Switch mode
		correction_mode();
		double correctionSpeed = MBED_CONF_PUSHTOGO_CORRECTION_SPEED_SIDEREAL
				* sidereal_speed;
		/*Use correction to goto the final angle with high resolution*/
		angleDeg = getAngleDeg();
		debug_if(AXIS_DEBUG, "%s: correct from %f to %f deg\n", axisName,
				angleDeg, dest); // TODO: DEBUG

		double diff = remainder(angleDeg - dest, 360.0);
		if (diff > MBED_CONF_PUSHTOGO_MAX_CORRECTION_ANGLE) {
			debug(
					"%s: correction too large: %f. Check hardware configuration.\n",
					axisName, diff);
			stop();
			idle_mode();
			currentSpeed = 0;
			slew_finish_state = FINISH_EMERG_STOPPED;
			status = AXIS_STOPPED;
			return;
		}

		int nTry = 3; // Try 3 corrections at most
		while (--nTry && fabsf(diff) > MBED_CONF_PUSHTOGO_CORRECTION_TOLERANCE) {
			/*Determine correction direction and time*/
			sd = (diff > 0.0) ? STEP_BACKWARD : STEP_FORWARD;

			/*Perform correction*/
			currentSpeed = stepper->setFrequency(stepsPerDeg * correctionSpeed)
					/ stepsPerDeg; // Set and update actual speed

			int correctionTime_ms = (int) (fabs(diff) / currentSpeed * 1000); // Use the accurate speed for calculating time

			debug_if(AXIS_DEBUG,
					"%s: correction: from %f to %f deg. time=%d ms\n", axisName,
					angleDeg, dest, correctionTime_ms); //TODO: DEBUG
			if (correctionTime_ms < MBED_CONF_PUSHTOGO_MIN_CORRECTION_TIME) {
				break;
			}

			/*Start, wait, stop*/
			stepper->start(sd);
			uint32_t flags = osThreadFlagsWait(AXIS_EMERGE_STOP_SIGNAL,
			osFlagsWaitAny, correctionTime_ms);
			stepper->stop();
			if (flags != osFlagsErrorTimeout) {
				// Emergency stop!
				slew_finish_state = FINISH_EMERG_STOPPED;
				goto emerge_stop2;
			}

			angleDeg = getAngleDeg();
			diff = remainder(angleDeg - dest, 360.0);
		}

		if (!nTry) {
			debug("%s: correction failed. Check hardware configuration.\n",
					axisName);
		}

		debug_if(AXIS_DEBUG, "%s: correction finished: %f deg\n", axisName,
				angleDeg); //TODO:DEBUG
	}
	emerge_stop2:
// Set status to stopped
	currentSpeed = 0;
	status = AXIS_STOPPED;
	idle_mode();
}

void Axis::track(axisrotdir_t dir) {
	track_mode();
	if (trackSpeed != 0 && dir != AXIS_ROTATE_STOP) {
		stepdir_t sd =
				(dir == AXIS_ROTATE_POSITIVE) ? STEP_FORWARD : STEP_BACKWARD;
		currentSpeed = stepper->setFrequency(trackSpeed * stepsPerDeg)
				/ stepsPerDeg;
		currentDirection = dir;
		stepper->start(sd);
	} else {
		// For DEC axis
		dir = AXIS_ROTATE_STOP;
		trackSpeed = 0;
		currentSpeed = 0;
		currentDirection = AXIS_ROTATE_POSITIVE;
	}
	status = AXIS_TRACKING;
	ThisThread::flags_clear(
	AXIS_STOP_SIGNAL | AXIS_EMERGE_STOP_SIGNAL | AXIS_GUIDE_SIGNAL);
	// Empty the guide queue
	while (!guide_queue.empty())
		guide_queue.get();

	while (true) {
		// Now we wait for SOMETHING to happen - either STOP, EMERGE_STOP or GUIDE
		uint32_t flags = osThreadFlagsWait(
		AXIS_GUIDE_SIGNAL | AXIS_STOP_SIGNAL | AXIS_EMERGE_STOP_SIGNAL,
		osFlagsWaitAny, osWaitForever);
		if ((flags & osFlagsError) == 0) // has flag
				{
			if (flags & (AXIS_STOP_SIGNAL | AXIS_EMERGE_STOP_SIGNAL)) {
				// We stop tracking
				break;
			} else if (flags & AXIS_GUIDE_SIGNAL) {
				bool stopped = false;
				guiding = true;
				// Guide. Process all commands in the queue
				while (true) {
					osEvent evt = guide_queue.get(0); // try to get a message
					if (evt.status == osEventMessage) {
						int guideTime_ms = (int) evt.value.p;
						if (guideTime_ms == 0)
							continue; // Nothing to guide
						// Determine guide direction
						axisrotdir_t guide_dir =
								(guideTime_ms > 0) ?
										AXIS_ROTATE_POSITIVE :
										AXIS_ROTATE_NEGATIVE;

						double newSpeed =
								(guide_dir == currentDirection) ?
										trackSpeed + guideSpeed :
										trackSpeed - guideSpeed; /*Determine speed in the original direction (currentDirection)*/

						// Clamp to maximum guide time
						guideTime_ms = abs(guideTime_ms);
						if (guideTime_ms > MBED_CONF_PUSHTOGO_MAX_GUIDE_TIME) {
							debug("Axis: Guiding time too long: %d ms\n",
									abs(guideTime_ms));
							guideTime_ms = MBED_CONF_PUSHTOGO_MAX_GUIDE_TIME;
						}

						bool dirswitch = false;
						if (newSpeed > 0) {
							currentSpeed = stepper->setFrequency(
									newSpeed * stepsPerDeg) / stepsPerDeg; //set and update accurate speed
							if (trackSpeed == 0) {
								// For DEC, we also need to start the motor
								stepper->start(STEP_FORWARD); // Reverse direction
							}
						} else if (newSpeed < 0) {
							//
							stepper->stop();
							currentSpeed = stepper->setFrequency(
									-newSpeed * stepsPerDeg) / stepsPerDeg; //set and update accurate speed
							stepper->start(
									(currentDirection == AXIS_ROTATE_POSITIVE) ?
											STEP_BACKWARD : STEP_FORWARD); // Reverse direction
							dirswitch = true;
						} else {
							// newSpeed == 0, just stop the motor
							currentSpeed = 0;
							stepper->stop();
							dirswitch = true; // Make sure to recover the original speed
						}

						uint32_t flags = osThreadFlagsWait(
						AXIS_STOP_SIGNAL | AXIS_EMERGE_STOP_SIGNAL,
						osFlagsWaitAny, guideTime_ms);
						if (flags != osFlagsErrorTimeout) {
							//break and stop;
							stopped = true;
							break;
						}
						if (dirswitch || trackSpeed == 0) {
							stepper->stop();
							// Restart the motor in original direction
							if (trackSpeed != 0) {
								stepper->start(
										(currentDirection
												== AXIS_ROTATE_POSITIVE) ?
												STEP_FORWARD : STEP_BACKWARD); // Reverse direction
							}
						}
						// Restore to normal speed
						if (trackSpeed != 0)
							currentSpeed = stepper->setFrequency(
									trackSpeed * stepsPerDeg) / stepsPerDeg;
						else
							currentSpeed = 0;

						// End guiding
					} else {
						// No more message to get. Break out
						break;
					}
				}
				guiding = false;
				if (stopped) {
					//Complete break out
					break;
				}
			}
		}
	}

// Stop
	currentSpeed = 0;
	stepper->stop();
	status = AXIS_STOPPED;
	idle_mode();
}

void Axis::err_cb() {
	debug_if(AXIS_DEBUG, "%s: stopped on error\n", axisName);
	emergency_stop();
}

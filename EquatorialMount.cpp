#include <math.h>
#include "EquatorialMount.h"
#include "Logger.h"

EquatorialMount::EquatorialMount(Axis &ra, Axis &dec, UTCClock &clk,
		LocationProvider &loc) :
		ra(ra), dec(dec), clock(clk), loc(loc), inclinometer(NULL), curr_pos(0,
				0), curr_nudge_dir(NUDGE_NONE), nudgeSpeed(0), pier_side(
				PIER_SIDE_EAST), num_alignment_stars(0), pec(ra), thd_monitor(
				osPriorityBelowNormal, 1024, NULL, "EqMount Monitor") {
	south = loc.getLatitude() < 0.0;

	Logger::log("Equatorial Mount started.");

	// Set RA and DEC positions to zero if no encoder is installed
	if (!(ra.getEncoder() && dec.getEncoder())) {
		ra.setAngleDeg(0);
		dec.setAngleDeg(0);
	}
	dec.setTrackSpeedSidereal(0); // Make sure dec doesn't move during tracking
	updatePosition();

	// Set PEC to RA axis
	this->ra.setPEC(&pec);
	this->ra.setPECEnabled(true);

	// Obtain encoder offset from config
	if (ra.getEncoder()
			&& TelescopeConfiguration::isConfigExist("ra_encoder_offset")) {
		ra.setEncoderOffset(
				TelescopeConfiguration::getDouble("ra_encoder_offset"));

		Logger::log("EqMount RA Encoder offset loaded: %.2f.",
				TelescopeConfiguration::getDouble("ra_encoder_offset"));
	}
	if (dec.getEncoder()
			&& TelescopeConfiguration::isConfigExist("dec_encoder_offset")) {
		dec.setEncoderOffset(
				TelescopeConfiguration::getDouble("dec_encoder_offset"));

		Logger::log("EqMount DEC Encoder offset loaded: %.2f.",
				TelescopeConfiguration::getDouble("dec_encoder_offset"));
	}

	// Load calibration
	if (TelescopeConfiguration::isConfigExist("calib_num_stars") // If has stars, load stars and calibrate from there
			&& (num_alignment_stars = TelescopeConfiguration::getInt(
					"calib_num_stars")) > 0) {
		for (int i = 0; i < num_alignment_stars; i++) {
			char buf[32];
			sprintf(buf, "cs_%d_r_ra", i);
			if (TelescopeConfiguration::isConfigExist(buf)) {
				alignment_stars[i].star_ref.ra =
						TelescopeConfiguration::getDouble(buf);
			} else {
				num_alignment_stars = i;
				break;
			}
			sprintf(buf, "cs_%d_r_dec", i);
			if (TelescopeConfiguration::isConfigExist(buf)) {
				alignment_stars[i].star_ref.dec =
						TelescopeConfiguration::getDouble(buf);
			} else {
				num_alignment_stars = i;
				break;
			}
			sprintf(buf, "cs_%d_m_ra", i);
			if (TelescopeConfiguration::isConfigExist(buf)) {
				alignment_stars[i].star_meas.ra_delta =
						TelescopeConfiguration::getDouble(buf);
			} else {
				num_alignment_stars = i;
				break;
			}
			sprintf(buf, "cs_%d_m_dec", i);
			if (TelescopeConfiguration::isConfigExist(buf)) {
				alignment_stars[i].star_meas.dec_delta =
						TelescopeConfiguration::getDouble(buf);
			} else {
				num_alignment_stars = i;
				break;
			}
			sprintf(buf, "cs_%d_time", i);
			if (TelescopeConfiguration::isConfigExist(buf)) {
				alignment_stars[i].timestamp =
						TelescopeConfiguration::getDouble(buf);
			} else {
				num_alignment_stars = i;
				break;
			}
		}

		recalibrate();

		Logger::log("EqMount calib %d stars loaded.", num_alignment_stars);

	} else if (TelescopeConfiguration::isConfigExist("calib_off_ra")
			&& TelescopeConfiguration::isConfigExist("calib_off_dec")
			&& TelescopeConfiguration::isConfigExist("calib_pa_alt")
			&& TelescopeConfiguration::isConfigExist("calib_pa_azi")
			&& TelescopeConfiguration::isConfigExist("calib_cone")) { // Else try to directly load the calibration
		calibration.offset = IndexOffset(
				TelescopeConfiguration::getDouble("calib_off_dec"),
				TelescopeConfiguration::getDouble("calib_off_ra"));
		calibration.pa = AzimuthalCoordinates(
				TelescopeConfiguration::getDouble("calib_pa_alt"),
				TelescopeConfiguration::getDouble("calib_pa_azi"));
		calibration.cone = TelescopeConfiguration::getDouble("calib_cone");

		Logger::log("EqMount calib info loaded.");
	} else {									// Else use initial calibration
		calibration.pa = AzimuthalCoordinates(loc.getLatitude(), 0);

		Logger::log("EqMount calib is reset.");
	}

	// Start in tracking mode
	startTracking();

	// Start monitor thread
	thd_monitor.start(callback(this, &EquatorialMount::task_monitor));
}

osStatus EquatorialMount::goTo(double ra_dest, double dec_dest) {
	return goTo(EquatorialCoordinates(dec_dest, ra_dest));
}

osStatus EquatorialMount::goTo(EquatorialCoordinates dest) {

	debug_ptg(EM_DEBUG, "dest ra=%.2f, dec=%.2f\r\n", dest.ra, dest.dec);

	updatePosition(); // Get the latest position information

	printPosition();

	Logger::log("EqMount goTo EQ RA=%.2f DEC=%.2f.", dest.ra, dest.dec);

	for (int i = 0; i < 2; i++) {
		// Convert to Mount coordinates. Automatically determine the pier side, then apply offset
		MountCoordinates dest_mount = convertToMountCoordinates(dest);

		osStatus s = goToMount(dest_mount, (i > 0)); // Use correction only for the second time
		if (s != osOK) {

			Logger::log("EqMount goTo EQ aborted.");
			return s;
		}
	}

	printPosition();

	return osOK;
}

osStatus EquatorialMount::goToMount(MountCoordinates dest_mount,
		bool withCorrection) {
	double limit = TelescopeConfiguration::getDouble("ra_limit");
	dest_mount.ra_delta -= getTilt(); // Correct tilt

	// Check RA limit
	if (!(dest_mount.ra_delta < 90 + limit && dest_mount.ra_delta > -90 - limit)) {
		// Out of limit, abort
		debug_ptg(EM_DEBUG,
				"ra limit exceeded at %f (must be in %f to %f).\r\n",
				dest_mount.ra_delta, -90 - limit, 90 + limit);

		Logger::log("EqMount goTo target exceeds RA limit.");
		return osErrorParameter;
	}

	mutex_execution.lock();

	bool was_tracking = false;
	if ((status & MOUNT_TRACKING) && !(status & MOUNT_NUDGING)) {
		// Tracking mode
		was_tracking = true;
		stopSync();
	} else if (status != MOUNT_STOPPED) {
		debug_ptg(EM_DEBUG, "goTo requested while mount is not stopped.\r\n");
		mutex_execution.unlock();

		Logger::logError(
				"Error: EqMount goTo requested while mount is not stopped.");
		return osErrorParameter;
	}
	debug_ptg(EM_DEBUG, "goTo dstmnt ra=%.2f, dec=%.2f\r\n",
			dest_mount.ra_delta, dest_mount.dec_delta);
	debug_ptg(EM_DEBUG, "start slewing\r\n");

	Logger::log("EqMount goTo MNT RA=%.2f DEC=%.2f.", dest_mount.ra_delta,
			dest_mount.dec_delta);

	status = MOUNT_SLEWING;
	ra.startSlewTo(AXIS_ROTATE_CLAMPED, dest_mount.ra_delta, withCorrection);
	dec.startSlewTo(AXIS_ROTATE_CLAMPED, dest_mount.dec_delta, withCorrection);

	int ret = (int) ra.waitForSlew();
	ret |= (int) dec.waitForSlew();

	debug_ptg(EM_DEBUG, "slewing finished\r\n");

	status = MOUNT_STOPPED;

	mutex_execution.unlock();

	if (was_tracking && !(ret & (FINISH_ERROR | FINISH_EMERG_STOPPED))) {
		startTracking();
	}

	updatePosition(); // Update current position

	Logger::log("EqMount goTo finished and returned %d.", ret);
	if (ret) {
		// Stopped during slew
		return ret;
	} else {
		return osOK;
	}
}

osStatus EquatorialMount::startTracking() {
	if (status != MOUNT_STOPPED) {
		debug_ptg(EM_DEBUG,
				"tracking requested while mount is not stopped.\r\n");
		Logger::logError(
				"Error: EqMount tracking requested while mount is not stopped.");
		return osErrorParameter;
	}

	mutex_execution.lock();
	axisrotdir_t ra_dir = AXIS_ROTATE_POSITIVE; // Tracking is always going to positive hour angle direction, which is defined as positive.
	status = MOUNT_TRACKING;
	osStatus sr, sd;
	sr = ra.startTracking(ra_dir);
	sd = dec.startTracking(AXIS_ROTATE_STOP);
	mutex_execution.unlock();

	if (sr != osOK || sd != osOK)
		return osErrorResource;
	else {
		Logger::log("EqMount start tracking.");
		return osOK;
	}
}

osStatus EquatorialMount::startNudge(nudgedir_t newdir) {
	if (status & MOUNT_SLEWING) // Cannot be nudged in slewing mode
			{
		Logger::logError("EqMount nudge requested during slew.");
		return osErrorParameter;
	}
	osStatus s = osOK;
	mutex_execution.lock();
	if (newdir == NUDGE_NONE) { // Stop nudging if being nudged
		Logger::log("EqMount nudge stopped.");
		if (status & MOUNT_NUDGING) {
			mountstatus_t oldstatus = status;
			stopAsync(); // Stop the mount
			if (oldstatus == MOUNT_NUDGING) {
				// Stop the mount
			} else if (oldstatus == MOUNT_NUDGING_TRACKING) {
				// Get to tracking state
				ra.setSlewSpeed(nudgeSpeed); // restore the slew rate of RA
				startTracking();
			}
		}
	} else { // newdir is not NUDGE_NONE
		Logger::log("EqMount nudge %c%c.",
				((newdir & NUDGE_WEST) ? 'W' : (newdir & NUDGE_EAST) ? 'E' : ' '),
				((newdir & NUDGE_NORTH) ? 'N' : (newdir & NUDGE_SOUTH) ? 'S' : ' '));
		updatePosition(); // Update current position, because we need to know the current pier side
		bool ra_changed = false, dec_changed = false;
		axisrotdir_t ra_dir = AXIS_ROTATE_STOP, dec_dir = AXIS_ROTATE_STOP;
		if ((status & MOUNT_NUDGING) == 0) {
			// Initial nudge
			curr_nudge_dir = NUDGE_NONE; //Make sure the current nudging direction is cleared
			nudgeSpeed = getSlewSpeed(); // Get nudge speed and use it for ALL following nudge operations, until the nudge finishes
		}
		// see what has changed in RA
		if ((curr_nudge_dir & (NUDGE_WEST | NUDGE_EAST))
				!= (newdir & (NUDGE_WEST | NUDGE_EAST))) {
			// If something on east/west has changed, we need to stop the RA axis (or maybe enable it later to switch direction)
			ra_changed = true;
			if (newdir & NUDGE_EAST) {
				// Nudge east
				ra_dir = AXIS_ROTATE_NEGATIVE;
			} else if (newdir & NUDGE_WEST) {
				// Nudge west
				ra_dir = AXIS_ROTATE_POSITIVE;
			} else {
				ra_dir = AXIS_ROTATE_STOP;
			}
		}
		// see what has changed in DEC
		if ((curr_nudge_dir & (NUDGE_SOUTH | NUDGE_NORTH))
				!= (newdir & (NUDGE_SOUTH | NUDGE_NORTH))) {
			// If something on east/west has changed, we need to stop the RA axis (or maybe enable it later to switch direction)
			dec_changed = true;
			if (newdir & NUDGE_NORTH) {
				// Nudge north
				dec_dir =
						(curr_pos.side == PIER_SIDE_WEST) ?
								AXIS_ROTATE_NEGATIVE : AXIS_ROTATE_POSITIVE;
			} else if (newdir & NUDGE_SOUTH) {
				// Nudge south
				dec_dir =
						(curr_pos.side == PIER_SIDE_WEST) ?
								AXIS_ROTATE_POSITIVE : AXIS_ROTATE_NEGATIVE;
			} else {
				dec_dir = AXIS_ROTATE_STOP;
			}
		}
		curr_nudge_dir = newdir;

		// Request stop as necessary
		if (ra_changed) {
			ra.flushCommandQueue();
			if (ra.getSlewState() == AXIS_SLEW_DECELERATING
					&& ra_dir == ra.getCurrentDirection()) {
				ra.stopKeepSpeed();
			} else {
				ra.stop();
			}
		}

		if (dec_changed) {
			dec.flushCommandQueue();
			if (dec.getSlewState() == AXIS_SLEW_DECELERATING
					&& dec_dir == dec.getCurrentDirection()) {
				dec.stopKeepSpeed();
			} else {
				dec.stop();
			}
		}

		// Wait for stop together
//		while ((ra_changed && ra.getStatus() != AXIS_STOPPED
//				&& ra.getStatus() != AXIS_INERTIAL)
//				|| (dec_changed && dec.getStatus() != AXIS_STOPPED
//						&& dec.getStatus() != AXIS_INERTIAL))
//		{
//			Thread::yield();
//		}

		// DEC axis is ok to start regardless of tracking state
		if (dec_changed && dec_dir != AXIS_ROTATE_STOP) {
			s = dec.startSlewingIndefinite(dec_dir);
		}

		// Now RA
		if (ra_changed && s == osOK) {
			if (status & MOUNT_TRACKING) { // In tracking mode now, need to calculate differential rate
				if (ra_dir == AXIS_ROTATE_STOP) { // resume tracking
					s = ra.startTracking(AXIS_ROTATE_POSITIVE);
				} else {
					// This is the complicated part
					double trackSpeed = ra.getTrackSpeedSidereal()
							* sidereal_speed;
					debug_ptg(EM_DEBUG, "ra, ns=%f, ts=%f\r\n", nudgeSpeed,
							trackSpeed);
					if (ra_dir == AXIS_ROTATE_POSITIVE) {
						// Same direction as tracking
						ra.setSlewSpeed(nudgeSpeed + trackSpeed);
						s = ra.startSlewingIndefinite(AXIS_ROTATE_POSITIVE);
					} else if (nudgeSpeed < trackSpeed) { // ra_dir == AXIS_ROTATE_NEGATIVE
														  // Partially canceling the tracking speed
						ra.setSlewSpeed(trackSpeed - nudgeSpeed);
						s = ra.startSlewingIndefinite(AXIS_ROTATE_POSITIVE);
					} else if (nudgeSpeed > trackSpeed) {// ra_dir == AXIS_ROTATE_NEGATIVE
														 // Direction inverted
						ra.setSlewSpeed(nudgeSpeed - trackSpeed);
						ra.startSlewingIndefinite(AXIS_ROTATE_NEGATIVE);
					}
					// else would be nudgeSpeed == trackSpeed, and we don't need to start the RA Axis
				}
			} else {
				// In non-tracking mode
				if (ra_dir != AXIS_ROTATE_STOP) {
					s = ra.startSlewingIndefinite(ra_dir);
				}
			}
		}

		// Update status
		if (status & MOUNT_TRACKING)
			status = MOUNT_NUDGING_TRACKING;
		else
			status = MOUNT_NUDGING;

		debug_ptg(EM_DEBUG, "status=%d, ra_dir=%d, dec_dir=%d\r\n",
				(int) status, (int) ra_dir, (int) dec_dir);
	}
	mutex_execution.unlock();

	return s;
}

osStatus EquatorialMount::stopNudge() {
	return startNudge(NUDGE_NONE);
}

double EquatorialMount::getSlewSpeed() {
	return dec.getSlewSpeed(); // Fix: RA speed changes if using NUDGE_TRACKING mode
}

double EquatorialMount::getTrackSpeedSidereal() {
	return ra.getTrackSpeedSidereal();
}

double EquatorialMount::getGuideSpeedSidereal() {
	return ra.getGuideSpeedSidereal();
}

osStatus EquatorialMount::stopTracking() {
	if ((status & MOUNT_TRACKING) == 0) {
		return osErrorParameter;
	}
	mutex_execution.lock();
	if (!(status & MOUNT_NUDGING)) {
		// Tracking
		stopSync();
	} else {
		// Nudging
		status = MOUNT_NUDGING;
	}
	mutex_execution.unlock();

	Logger::log("EqMount stopped tracking.");
	return osOK;
}

void EquatorialMount::printPosition() {
	debug_ptg(EM_DEBUG, "Mount: RA=%7.2f, DEC=%7.2f %c\n", curr_pos.ra_delta,
			curr_pos.dec_delta, (curr_pos.side == PIER_SIDE_WEST) ? 'W' : 'E');
	debug_ptg(EM_DEBUG, "EQ:    RA=%7.2f, DEC=%7.2f\n", curr_pos_eq.ra,
			curr_pos_eq.dec);
}

void EquatorialMount::updatePosition() {
	// Lock the mutex to avoid race condition on the current position values
	mutex_update.lock();
	curr_pos = MountCoordinates(dec.getAngleDeg(),
			ra.getAngleDeg() + getTilt());
	// Update Eq coordinates
	curr_pos_eq = this->convertToEqCoordinates(curr_pos);
	mutex_update.unlock();
}

void EquatorialMount::emergencyStop() {
	ra.emergency_stop();
	dec.emergency_stop();
	status = MOUNT_STOPPED;
	curr_nudge_dir = NUDGE_NONE;
	Logger::log("EqMount emergency stopped.");
}

void EquatorialMount::stopAsync() {
	ra.stop();
	dec.stop();
	status = MOUNT_STOPPED;
	Logger::log("EqMount stopped async.");
}

void EquatorialMount::stopSync() {
	// Wait until they're fully stopped
	while (ra.getStatus() != AXIS_STOPPED || dec.getStatus() != AXIS_STOPPED) {
		ra.stop();
		dec.stop();
		ThisThread::yield();
	}
	status = MOUNT_STOPPED;
	Logger::log("EqMount stopped sync.");
}

osStatus EquatorialMount::recalibrate() {

	if (num_alignment_stars == 0) {
		calibration.error = 0;
		return osOK;
	}
	EqCalibration newcalib = alignAuto(num_alignment_stars, alignment_stars,
			loc.getLocation());
	if (newcalib.error > 100.0) {
		calibration.error = INFINITY;
		return osErrorParameter;
	}

	calibration = newcalib;
	saveCalibration();

	return osOK;
}

void EquatorialMount::saveCalibration() {
	TelescopeConfiguration::setDouble("calib_off_ra",
			calibration.offset.ra_off);
	TelescopeConfiguration::setDouble("calib_off_dec",
			calibration.offset.dec_off);
	TelescopeConfiguration::setDouble("calib_pa_alt", calibration.pa.alt);
	TelescopeConfiguration::setDouble("calib_pa_azi", calibration.pa.azi);
	TelescopeConfiguration::setDouble("calib_cone", calibration.cone);

	// Delete all saved stars
	for (int i = 0; i < MAX_AS_N; i++) {
		char buf[32];
		sprintf(buf, "cs_%d_r_ra", i);
		if (TelescopeConfiguration::isConfigExist(buf)) {
			TelescopeConfiguration::removeConfig(buf);
		}
		sprintf(buf, "cs_%d_r_dec", i);
		if (TelescopeConfiguration::isConfigExist(buf)) {
			TelescopeConfiguration::removeConfig(buf);
		}
		sprintf(buf, "cs_%d_m_ra", i);
		if (TelescopeConfiguration::isConfigExist(buf)) {
			TelescopeConfiguration::removeConfig(buf);
		}
		sprintf(buf, "cs_%d_m_dec", i);
		if (TelescopeConfiguration::isConfigExist(buf)) {
			TelescopeConfiguration::removeConfig(buf);
		}
		sprintf(buf, "cs_%d_time", i);
		if (TelescopeConfiguration::isConfigExist(buf)) {
			TelescopeConfiguration::removeConfig(buf);
		}
	}
	// Write number of stars and all star info
	if (num_alignment_stars > 0)
		TelescopeConfiguration::setInt("calib_num_stars", num_alignment_stars);
	else if (TelescopeConfiguration::isConfigExist("calib_num_stars")) {
		// Remove calib star config if there is no star, so next time it will not be loaded
		TelescopeConfiguration::removeConfig("calib_num_stars");
	}
	for (int i = 0; i < num_alignment_stars; i++) {
		char buf[32];
		sprintf(buf, "cs_%d_r_ra", i);
		TelescopeConfiguration::setDouble(buf, alignment_stars[i].star_ref.ra);
		sprintf(buf, "cs_%d_r_dec", i);
		TelescopeConfiguration::setDouble(buf, alignment_stars[i].star_ref.dec);
		sprintf(buf, "cs_%d_m_ra", i);
		TelescopeConfiguration::setDouble(buf,
				alignment_stars[i].star_meas.ra_delta);
		sprintf(buf, "cs_%d_m_dec", i);
		TelescopeConfiguration::setDouble(buf,
				alignment_stars[i].star_meas.dec_delta);
		sprintf(buf, "cs_%d_time", i);
		TelescopeConfiguration::setDouble(buf, alignment_stars[i].timestamp);
	}

	TelescopeConfiguration::saveConfig();
}

osStatus EquatorialMount::guide(guidedir_t dir, int ms) {
	// Check we are in tracking mode
	if (status != MOUNT_TRACKING) {
		return osErrorResource;
	}
	switch (dir) {
	case GUIDE_EAST:
		return ra.guide(AXIS_ROTATE_NEGATIVE, ms);
	case GUIDE_WEST:
		return ra.guide(AXIS_ROTATE_POSITIVE, ms);
	case GUIDE_NORTH:
		return dec.guide(
				(curr_pos.side == PIER_SIDE_WEST) ?
						AXIS_ROTATE_NEGATIVE : AXIS_ROTATE_POSITIVE, ms);
	case GUIDE_SOUTH:
		return dec.guide(
				(curr_pos.side == PIER_SIDE_WEST) ?
						AXIS_ROTATE_POSITIVE : AXIS_ROTATE_NEGATIVE, ms);
	default:
		return osErrorParameter;
	}
}

void EquatorialMount::setSlewSpeed(double rate) {
	if (status & MOUNT_NUDGING) {
		// Need to lock the mutex if mount is being nudged, to update the nudgeSpeed
		mutex_execution.lock();
		nudgeSpeed = rate; // Update nudgeSpeed
		if ((status == MOUNT_NUDGING_TRACKING)
				&& ((curr_nudge_dir & (NUDGE_EAST | NUDGE_WEST)) != 0)) { // Need to take special care in nudging_tracking mode that affects RA (i.e. indefinite slew w/ tracking)
			axisrotdir_t ra_dir = AXIS_ROTATE_STOP, curr_dir;
			// Get RA rotation direction (on top of tracking rate)
			if (curr_nudge_dir & NUDGE_EAST) {
				// Nudge east
				ra_dir = AXIS_ROTATE_NEGATIVE;
			} else if (curr_nudge_dir & NUDGE_WEST) {
				// Nudge west
				ra_dir = AXIS_ROTATE_POSITIVE;
			}

			curr_dir = ra.getCurrentDirection(); // Current direction

			double trackSpeed = ra.getTrackSpeedSidereal() * sidereal_speed;
			double absSpeed = nudgeSpeed;

			if (ra_dir == AXIS_ROTATE_POSITIVE) {
				// Same direction as tracking
				absSpeed = nudgeSpeed + trackSpeed;
				// keep ra_dir
			} else if (nudgeSpeed < trackSpeed) { // ra_dir == AXIS_ROTATE_NEGATIVE
												  // Partially canceling the tracking speed
				absSpeed = trackSpeed - nudgeSpeed;
				ra_dir = AXIS_ROTATE_POSITIVE; // Rotate in the tracking direction
			} else if (nudgeSpeed > trackSpeed) { // ra_dir == AXIS_ROTATE_NEGATIVE
												  // Direction inverted
				absSpeed = nudgeSpeed - trackSpeed;
				ra_dir = AXIS_ROTATE_NEGATIVE; // Invert the rotation
			} else { // nudgeSpeed == trackSpeed
				absSpeed = 0; // Set speed to zero, although effective the speed would be a tiny value
			}

			if (ra_dir != curr_dir) {
				ra.stop();
			}
			ra.setSlewSpeed(absSpeed);
			if (ra_dir != curr_dir) {
				ra.startSlewingIndefinite(ra_dir);
			}

		} else {
			// Simply update rate
			ra.setSlewSpeed(rate);
		}
		mutex_execution.unlock();
	} else {
		// Simply update rate
		ra.setSlewSpeed(rate);
	}
	// Simply update rate for DEC
	dec.setSlewSpeed(rate);

	Logger::log("EqMount slew speed set to %f.", rate);
}

void EquatorialMount::setTrackSpeedSidereal(double rate) {
//	mutex_execution.lock();
	ra.setTrackSpeedSidereal(rate);
	Logger::log("EqMount track speed set to %f.", rate);
//	mutex_execution.unlock();
}

void EquatorialMount::setGuideSpeedSidereal(double rate) {
//	mutex_execution.lock();
	ra.setGuideSpeedSidereal(rate);
	dec.setGuideSpeedSidereal(rate);

	Logger::log("EqMount guide speed set to %f.", rate);
//	mutex_execution.unlock();
}

mountstatus_t EquatorialMount::getStatus() {
	mountstatus_t s = status;
	if (ra.isGuiding() || dec.isGuiding())
		s = (mountstatus_t) (s | MOUNT_GUIDING);
	return s;
}

void EquatorialMount::forceAlignment() {
	for (int i = 0; i < num_alignment_stars; i++) {
		alignment_stars[i].star_meas = convertToMountCoordinates(
				alignment_stars[i].star_ref);
		alignment_stars[i].timestamp = clock.getTime();
	}
}

void EquatorialMount::setEncoderIndex() {
	double ra_off = ra.getAngleDeg() + getTilt() + ra.getEncoderOffset();
	double dec_off = dec.getAngleDeg() + dec.getEncoderOffset();
	// Set encoder offset to cancel current mount position values
	ra.setEncoderOffset(ra_off);
	dec.setEncoderOffset(dec_off);

	TelescopeConfiguration::setDouble("ra_encoder_offset", ra_off);
	TelescopeConfiguration::setDouble("dec_encoder_offset", dec_off);

	TelescopeConfiguration::saveConfig();
	Logger::log("EqMount encoder index set to current position.");
}

double EquatorialMount::getTilt() {
	if (!inclinometer) {
		return 0;
	} else {
		return inclinometer->getTilt();
	}
}

// Task for monitoring RA limit
void EquatorialMount::task_monitor() {
	bool triggered = false;
	double r, limit;
	while (true) {
		switch (status) {
		case MOUNT_SLEWING:
		case MOUNT_TRACKING:
		case MOUNT_TRACKING_GUIDING:
		case MOUNT_NUDGING:
		case MOUNT_NUDGING_TRACKING:
			r = ra.getAngleDeg();
			limit = TelescopeConfiguration::getDouble("ra_limit");

			if (r > 90 + limit || r < -90 - limit) {
				if (!triggered) { // Avoid repeated triggering
					// Out of limit, abort
					debug_ptg(EM_DEBUG,
							"ra limit exceeded at %f (must be in %f to %f).\r\n",
							r, -90 - limit, 90 + limit);
					// Stop motion
					stopSync();
					triggered = true;

					Logger::log("EqMount RA limit reached.");
				}
			} else {
				// Reset trigger
				triggered = false;
			}
			break;
		default:
			break;
		}

		// Wait
		ThisThread::sleep_for(100ms);
	}
}

void EquatorialMount::linkEncoderOffset() {
	double ra_off = calibration.offset.ra_off + ra.getEncoderOffset();
	double dec_off = calibration.offset.dec_off + dec.getEncoderOffset();
	// Set encoder offset to cancel calibration offset
	ra.setEncoderOffset(ra_off);
	dec.setEncoderOffset(dec_off);

	TelescopeConfiguration::setDouble("ra_encoder_offset", ra_off);
	TelescopeConfiguration::setDouble("dec_encoder_offset", dec_off);
	TelescopeConfiguration::saveConfig();

	calibration.offset = IndexOffset(0, 0);
	forceAlignment();
	recalibrate();

	Logger::log(
			"EqMount calib offset linked into encoder. All stars forced to align.");
}

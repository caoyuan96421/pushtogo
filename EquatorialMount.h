#ifndef _EQUATORIALMOUNT_H_
#define _EQUATORIALMOUNT_H_

#include "mbed.h"
#include "Mount.h"
#include "pushtogo.h"


#define MAX_AS_N 10 // Max number of alignment stars

/**
 * Direction of nudge
 */
typedef enum {
	NUDGE_NONE = 0,
	NUDGE_EAST = 1,
	NUDGE_WEST = 2,
	NUDGE_NORTH = 4,
	NUDGE_SOUTH = 8,

	NUDGE_NORTHWEST = NUDGE_NORTH | NUDGE_WEST,
	NUDGE_SOUTHWEST = NUDGE_SOUTH | NUDGE_WEST,
	NUDGE_NORTHEAST = NUDGE_NORTH | NUDGE_EAST,
	NUDGE_SOUTHEAST = NUDGE_SOUTH | NUDGE_EAST,
} nudgedir_t;

/**
 * Direction of guide
 */
typedef enum {
	GUIDE_EAST = 1, GUIDE_WEST = 2, GUIDE_NORTH = 3, GUIDE_SOUTH = 4,
} guidedir_t;

/**
 * Object that represents an equatorial mount with two perpendicular axis called RA and Dec.
 */
class EquatorialMount: public Mount {

protected:
	Axis &ra;   /// RA Axis
	Axis &dec;  /// DEC Axis

	UTCClock &clock; /// Clock

	Mutex mutex_update; /// Mutex to lock position updating
	Mutex mutex_execution; /// Mutex to lock motion related functions

	LocationProvider loc;   /// Current location (GPS coordinates)
	bool south;	/// If we are in south semisphere
	MountCoordinates curr_pos; /// Current Position in mount coordinates (offset from the index positions)
	EquatorialCoordinates curr_pos_eq; /// Current Position in the equatorial coordinates (absolute pointing direction in the sky)
	nudgedir_t curr_nudge_dir;
	double nudgeSpeed;

	pierside_t pier_side;      /// Side of pier. 1: East
	EqCalibration calibration;
	AlignmentStar alignment_stars[MAX_AS_N];
	int num_alignment_stars;

	PEC pec; /// PEC function

public:

	/**
	 * Create an EquatorialMount object which controls two axis
	 * @param ra RA Axis
	 * @param dec DEC Axis
	 * @note cone_value, ma_alt, ma_azi,off_ra, off_dec will be init to zero, i.e. assuming a perfectly aligned mount pointing at RA=DEC=0
	 * @note This class assumes that the rotating direction of both axis are correct.
	 *       This should be done using the invert option when initializing the RotationAxis objects
	 * @sa RotationAxis
	 */
	EquatorialMount(Axis &ra, Axis &dec, UTCClock &clk, LocationProvider &loc);
	virtual ~EquatorialMount() {
	}

	/**
	 *   Perform a Go-To to specified equatorial coordinates in the sky
	 *   @param  ra_dest RA coordinate in degree.
	 *   @return osOK if no error
	 */
	osStatus goTo(double ra_dest, double dec_dest);
	osStatus goTo(EquatorialCoordinates dest);
	osStatus goToMount(MountCoordinates mc, bool withCorrection = true);
	osStatus goToIndex() {
		return goToMount(MountCoordinates(0, 0));
	}

	osStatus startNudge(nudgedir_t);
	osStatus stopNudge();

	osStatus startTracking();
	osStatus stopTracking();

	mountstatus_t getStatus();
	/**
	 * Guide on specified direction for specified time
	 */
	osStatus guide(guidedir_t dir, int ms);

	/*Calibration related functions*/

	/**
	 * Clear calibration, use current latitude for the polar axis
	 */
	void clearCalibration() {
		num_alignment_stars = 0;
		calibration = EqCalibration();
		calibration.pa.alt = loc.getLatitude();
	}

	/** 
	 * Clear calibration except axis offsets, use current latitude for the polar axis
	 */
	void clearCalibrationExceptOffsets() {
		num_alignment_stars = 0;
		calibration.cone = 0;
		calibration.pa.alt = loc.getLatitude();
		calibration.pa.azi = 0;
	}

	/**
	 * Get calibration
	 * @return Current calibration
	 */
	const EqCalibration &getCalibration() const {
		return calibration;
	}

	/**
	 * Set calibration
	 * @param calib New Calibration
	 */
	void setCalibration(const EqCalibration &calib) {
		calibration = calib;
	}

	/** @return number of alignment stars
	 */
	int getNumAlignmentStar() {
		return num_alignment_stars;
	}

	/** Add an alignment star
	 * @return osStatus error if no more star can be added
	 */
	osStatus addAlignmentStar(const AlignmentStar &as) {
		if (num_alignment_stars < MAX_AS_N) {
			alignment_stars[num_alignment_stars++] = as;
			return recalibrate();
		} else
			return osErrorResource;
	}

	/** Remove an alignment star
	 * @param index # of star to remove, starting from 0
	 * @return osStatus error if no more star can be deleted
	 */
	osStatus removeAlignmentStar(int index) {
		if (index < 0 || index >= num_alignment_stars) {
			return osErrorParameter;
		}
		for (; index < num_alignment_stars - 1; index++) {
			alignment_stars[index] = alignment_stars[index + 1];
		}
		num_alignment_stars--;
		return recalibrate();
	}

	/** Get alignment star
	 * @param index # of star to get
	 * @return pointer to that alignment star if found, NULL if doesn't exist
	 */
	AlignmentStar *getAlignmentStar(int index) {
		if (index < 0 || index >= num_alignment_stars) {
			return NULL;
		}
		return &alignment_stars[index];
	}

	/** Replace an alignment star with another
	 * @param index # of star to replace
	 * @param as new star to add. Will be copied by value
	 * @return error if star not found
	 */
	osStatus replaceAlignmentStar(int index, const AlignmentStar &as) {
		if (index < 0 || index >= num_alignment_stars) {
			return osErrorParameter;
		}
		alignment_stars[index] = as;
		return recalibrate();
	}

	/** Force all alignment stars to be perfected aligned according to the current calibration
	 * Useful during a polar alignment procedure
	 * @note This will throw off all the alignment stars if the error is too big or calibration is wrong
	 */
	void forceAlignment();

	/**
	 * Convert EQ coordinates to mount coodinates using current calibration. Utility function
	 * @param eq EQ coordinate to convert
	 * @return mount coordinates
	 */
	MountCoordinates convertToMountCoordinates(
			const EquatorialCoordinates &eq) {

		LocationCoordinates l = loc.getLocation();
		double timestamp = clock.getTimeHighResolution();
		return eq.toLocalEquatorialCoordinates(timestamp, l).applyPolarMisalignment(
				calibration.pa, l).applyConeError(calibration.cone).toMountCoordinates(
				PIER_SIDE_AUTO) + calibration.offset;
	}

	/**
	 * Convert mount coordinates to EQ coodinates using current calibration. Utility function
	 * @param mc Mount coordinate to convert
	 * @return EQ coordinates
	 */
	EquatorialCoordinates convertToEqCoordinates(const MountCoordinates &mc) {
		LocationCoordinates l = loc.getLocation();
		double timestamp = clock.getTimeHighResolution();
		return (mc - calibration.offset).toLocalEquatorialCoordinates().deapplyConeError(
				calibration.cone).deapplyPolarMisalignment(calibration.pa, l).toEquatorial(
				timestamp, l);
	}

	/**
	 * Use alignment stars to recalculate the calibration.
	 * @return osStatus
	 */
	osStatus recalibrate();

	/**
	 * Call emergency stop of the Axis objects
	 * @note This function can be called from any context (including ISR) to perform a hard stop of the mount
	 */
	void emergencyStop();

	/**
	 * Call stop of the Axis objects
	 * @note This function can be called from any context (including ISR) to perform a soft stop of the mount
	 */
	void stopAsync();

	/** BLOCKING. Cannot be called in ISR.
	 * Call stop of the Axis objects and wait until they are stopped.
	 * @note This function can be called from any context (including ISR) to perform a soft stop of the mount
	 */
	void stopSync();

	/**
	 * Get current equatorial coordinates
	 * @return current equatorial coordinates
	 */
	const EquatorialCoordinates &getEquatorialCoordinates() {
		updatePosition();
		return curr_pos_eq;
	}

	/**
	 * Get current mount coordinates
	 * @return current mount coordinates
	 */
	const MountCoordinates &getMountCoordinates() {
		updatePosition();
		return curr_pos;
	}

	/**
	 * Make an alignment star object using the provided reference star, current mount position, and current time
	 * @param star_ref Reference star position
	 * @return AlignmentStar object representing the alignment star
	 */
	AlignmentStar makeAlignmentStar(const EquatorialCoordinates star_ref) {
		updatePosition();
		return AlignmentStar(star_ref, curr_pos, clock.getTimeHighResolution());
	}

	/**
	 * Align the current mount using an array of alignment stars. Support up to 10 stars.
	 * @note If n=1, will only correct for Index offset
	 * If n=2, will correct for index offset and polar misalignment
	 * If n>=3, will correct for index offset, pa misalignment and cone error
	 * @param n # of alignment stars to use
	 * @param as Array of alignment stars
	 * @return osOK if successfully converged and updated the values
	 */
	osStatus align(int n, const AlignmentStar as[]);

	/**
	 * Set slew rate of both axis
	 * @param rate new speed
	 */
	void setSlewSpeed(double rate);

	/** @return current slew speed
	 */
	double getSlewSpeed();

	/**
	 * Set tracking speed of RA axis
	 * @param rate new speed in sidereal rate
	 */
	void setTrackSpeedSidereal(double rate);

	/** @return current track speed in sidereal units
	 */
	double getTrackSpeedSidereal();

	/**
	 * Set guiding speed of RA axis
	 * @param rate new speed in sidereal rate
	 */
	void setGuideSpeedSidereal(double rate);

	/** @return current guide speed in sidereal units
	 */
	double getGuideSpeedSidereal();

	/**
	 * Set current mount coordinate to be the index position of the encoder, and write to NV Memory if possible
	 * Has effect only if both RA and DEC encoders are enabled
	 */
	void setEncoderIndex();

	/**
	 * Print current position to stream. 
	 */
	void printPosition();

	/**
	 * Update current pointing direction from low-level encoders/counters.
	 * @note should be called before printing the position using printPosition
	 */
	void updatePosition();

	/** Get current time source
	 */
	UTCClock& getClock() const {
		return clock;
	}

	/** @return get current location coodinates.
	 */
	LocationCoordinates getLocation() const {
		return loc.getLocation();
	}

};

#endif /*_EQUATORIALMOUNT_H_*/


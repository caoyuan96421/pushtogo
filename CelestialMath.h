/*
 * CelestialMath.h
 *
 *  Created on: Feb 21, 2018
 *      Author: Yuan
 */

#ifndef _CELESTIALMATH_H_
#define _CELESTIALMATH_H_

#include <time.h>
#include <math.h>

/**
 * This file contains utility classes for doing astronomy related math, rotation of coordinate systems, and alignment
 */

static const double sidereal_speed = 0.00417807462; /* deg / s */

// Forward declaration
struct EquatorialCoordinates;
struct LocalEquatorialCoordinates;
struct AzimuthalCoordinates;
struct LocationCoordinates;
struct Transformation;
struct MountCoordinates;
/**
 * Type definition for side of Pier
 */
typedef enum {
	PIER_SIDE_EAST, PIER_SIDE_WEST, PIER_SIDE_AUTO = 0
} pierside_t;
/**
 * Coordinates on the equatorial sphere in standard DEC and RA coordinates. Expressed in degrees.
 * RA increases towards east (counterclockwise if viewed from north pole
 * @dec -90~90
 * @ra -180~180
 */
struct EquatorialCoordinates {
	double dec;		// Declination
	double ra;		// Right ascension
	EquatorialCoordinates(double d = 0, double r = 0);
	LocalEquatorialCoordinates toLocalEquatorialCoordinates(double timestamp,
			const LocationCoordinates &loc) const; // Convert to local equatorial
};

/**
 * Coordinates at the local observer, expressed in declination and hour angle. Expressed in degrees
 * Hour angle is zero on meridian, and increases towards west (opposite to RA direction)
 * @dec -90~90
 * @ha -180~180
 */
struct LocalEquatorialCoordinates {
	double dec;		// Declination
	double ha;		// Hour angle
	LocalEquatorialCoordinates(double d = 0, double h = 0);
	AzimuthalCoordinates toAzimuthalCoordinates(
			const LocationCoordinates &loc) const; // Convert to azimuthal
	EquatorialCoordinates toEquatorial(double timestamp,
			const LocationCoordinates &loc) const; // Convert to equatorial

	LocalEquatorialCoordinates applyPolarMisalignment(
			const Transformation &) const; // Apply polar misalignment
	LocalEquatorialCoordinates deapplyPolarMisalignment(
			const Transformation &) const; // Deapply polar misalignment
	LocalEquatorialCoordinates applyPolarMisalignment(
			const AzimuthalCoordinates& mpa,
			const LocationCoordinates& loc) const; // Apply polar misalignment
	LocalEquatorialCoordinates deapplyPolarMisalignment(
			const AzimuthalCoordinates& mpa,
			const LocationCoordinates& loc) const; // Deapply polar misalignment

	LocalEquatorialCoordinates applyConeError(const double cone) const; // Apply cone error
	LocalEquatorialCoordinates deapplyConeError(const double cone) const; // Deapply cone error

	MountCoordinates toMountCoordinates(pierside_t ps) const;
};

/**
 * Coordinates at the local observer, expressed in altitude and azimuth angles
 * @alt -90~90
 * @azi -180~180
 */
struct AzimuthalCoordinates {
	double alt;		// Altitude
	double azi;		// Azimuth
	AzimuthalCoordinates(double a1 = 0, double a2 = 0);
	LocalEquatorialCoordinates toLocalEquatorialCoordinates(
			const LocationCoordinates &loc); // Convert to local equatorial
};

/**
 * Geographical location of the local observer, in latitude and longitude (degrees)
 * @lat -90~90
 * @lon -180~180
 */
struct LocationCoordinates {
	double lat;		// Latitude
	double lon;		// Longtitude
	LocationCoordinates(double l1 = 0, double l2 = 0);
};

/**
 * 3D vector in cartesian coordinates, for matrix operations
 */
struct CartesianVector {
	double x, y, z;
	CartesianVector(double x = 0, double y = 0, double z = 0);
	CartesianVector operator*(const Transformation &t);
};

/**
 * General 3x3 transformation that acts on a vector in 3D space
 */
struct Transformation {
	double a11, a12, a13;
	double a21, a22, a23;
	double a31, a32, a33;
	CartesianVector operator*(const CartesianVector &vec);
	void transpose();
	Transformation &getMisalignedPolarAxisTransformation(
			const AzimuthalCoordinates &mpa, const LocationCoordinates &loc); // Get transformation due to misaligned polar axis
};

/// Below are alignment related definitions

/**
 * Offset value of the index, expressed in two angles
 * Ideally, without cone error, when ra=ra_off and dec=dec_off, the telescope will be pointing towards the poles
 * @dec_off -180~180
 * @ra_off -180~180
 */
struct IndexOffset {
	double dec_off; // Offset of the index position in DEC
	double ra_off;  // Offset of the index position in RA/HA axis
	IndexOffset(double d = 0, double r = 0);
};

/**
 * Mechanical coordinates of the mount
 * @dec_delta Displacement from index position in DEC axis
 * @ra_delta Displacement from index position in RA/HA axis
 * @side Side of pier
 */
struct MountCoordinates {
	double dec_delta; // Displacement from index position in DEC axis
	double ra_delta;  // Displacement from index position in RA/HA axis
	pierside_t side;
	MountCoordinates(double dec = 0, double ra = 0, pierside_t s =
			PIER_SIDE_AUTO);
	MountCoordinates operator+(const IndexOffset& offset) const;
	MountCoordinates operator-(const IndexOffset& offset) const;
	LocalEquatorialCoordinates toLocalEquatorialCoordinates() const;
};
/**
 * Alignment star object that stores an alignment star
 * @star_ref Reference position of the star in the sky chart (J2000)
 * @star_meas Measured position of the star mechanically (from motor or encoder)
 * @timestamp Time of the measurement
 */
struct AlignmentStar {
	EquatorialCoordinates star_ref; /// Reference position of the star in the sky (in current epoch)
	MountCoordinates star_meas;	/// Measured position of the star in mount coordinates
	double timestamp;				/// UTC timestamp of the measurement
	AlignmentStar() {
		timestamp = 0;
	}
	AlignmentStar(const EquatorialCoordinates& ref, MountCoordinates meas,
			double t);
	// Returns the LocalEquatorialCoordinates of the reference position at loc
	LocalEquatorialCoordinates star_ref_local(
			const LocationCoordinates &loc) const;
};

/**
 * EqCalibration stores 5 degree of freedom of a typical equatorial mount
 * @offset Index offsets (2 dof)
 * @pa Polar axis (2 dof)
 * @cone Cone error (1 dof)
 * @error RMS error of the alignment (for displaying only)
 */
struct EqCalibration {
	IndexOffset offset;
	AzimuthalCoordinates pa;
	double cone;
	double error;
	EqCalibration();
	EqCalibration(const IndexOffset& off, const AzimuthalCoordinates p,
			double c, double e);
};

/*Time functions*/
double getGreenwichMeanSiderealTime(double timestamp);
double getLocalSiderealTime(double timestamp, const LocationCoordinates &loc);

/*Alignment procedures*/

/**
 * One-star alignment for finding index offset only
 * @param star_ref reference star
 * @param loc Location coordinates
 */
IndexOffset alignOneStarForOffset(const AlignmentStar &star,
		const LocationCoordinates &loc);

/**
 * Two-star alignment for finding PA misalignment as well as offset in both axis
 * @param star_ref Reference stars (array of 2)
 * @param star_meas Measured stars (array of 2)
 * @param loc Location
 * @param pa Initial PA alt-az coordinates. This parameter will be updated with new values
 * @param offset Initial offset values. This parameter will be updated with new values
 * @return residue of alignment, +Inf if diverged
 */
double alignTwoStars(const AlignmentStar stars[],
		const LocationCoordinates &loc, AzimuthalCoordinates &pa,
		LocalEquatorialCoordinates &offset);

/**
 * N-star alignment for finding PA misalignment, offset, and cone error
 * This function will first call alignTwoStars with the first two stars assuming no cone error, then run an optimization algorithm to minimize the residual error by tweaking all 5 parameters.
 * @param N number of alignment stars
 * @param star_ref Reference stars
 * @param star_meas Measured stars
 * @param loc Location
 * @param pa Initial PA alt-az coordinates. This parameter will be updated with new values
 * @param offset Initial offset values. This parameter will be updated with new values
 * @param cone Initial cone error. This parameter will be updated with new values
 * @return residue of alignment, +Inf if diverged
 */
static double alignNStars(const int N,
		const LocalEquatorialCoordinates star_ref[],
		const MountCoordinates star_meas[], const LocationCoordinates &loc,
		AzimuthalCoordinates &pa, IndexOffset &offset, double &cone);

/**
 * Automatically figure out which routine to use based on number of alignment stars
 * @param N number of stars
 * @param stars alignment stars
 * @param loc location coordinates
 * @return EqCalibration containing polar, cone and offset alignment.
 * @note error field in the output value contains alignment RMS error
 */
EqCalibration alignAuto(const int N, const AlignmentStar stars[],
		const LocationCoordinates &loc);

/*Utility functions*/

/*
 * Convert HMS notation such as 21h54m31.6s to degrees (counting from 0h0m0s = 0degree, from -180 ~ 180 degrees)
 * E.g.
 * 1h0m0s -> 15.0
 * 23h59m59s -> -0.0041666667
 * If cannot find a match, NaN is returned
 */
double parseHMSAngle(char *hms);

/*
 * Convert DMS notation such as 21d54m31.6s to degrees (from -180 ~ 180 degrees)
 * E.g.
 * 1d30m0s -> 1.5
 * If cannot find a match, NaN is returned
 */
double parseDMSAngle(char *dms);

/**
 * Calculate King tracking rate based on the star position and location
 */
static double kingRate(EquatorialCoordinates eq, LocationCoordinates loc,
		double time);

#endif /* _CELESTIALMATH_H_ */


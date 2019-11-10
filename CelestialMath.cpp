/*
 * CelestialMath.cpp
 *
 *  Created on: Feb 21, 2018
 *      Author: Yuan
 */

#include "CelestialMath.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "mbed.h"
#include "debug.h"

static inline double clamp(double x) {
	return (x > 1) ? 1 : ((x < -1) ? -1 : x);
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_ITERATION 30
#define MAX_ITERATION_OPTIMIZATION 10

#define MAXN	20

static const double tol = 1e-10;
static const double eps = 1e-13;
static const double delta = 1e-7;

static const double RADIAN = 180.0 / M_PI;
static const double DEGREE = M_PI / 180.0;

LocalEquatorialCoordinates::LocalEquatorialCoordinates(double d, double h) :
		dec(d), ha(h) {
}

EquatorialCoordinates::EquatorialCoordinates(double d, double r) :
		dec(d), ra(r) {
}

LocationCoordinates::LocationCoordinates(double l1, double l2) :
		lat(l1), lon(l2) {
}

AzimuthalCoordinates::AzimuthalCoordinates(double a1, double a2) :
		alt(a1), azi(a2) {
}

CartesianVector::CartesianVector(double x, double y, double z) :
		x(x), y(y), z(z) {
}

void Transformation::transpose() {
	double temp = a12;
	a12 = a21;
	a21 = temp;
	temp = a13;
	a13 = a31;
	a31 = temp;
	temp = a32;
	a32 = a23;
	a23 = temp;
}

IndexOffset::IndexOffset(double d, double r) :
		dec_off(d), ra_off(r) {
}

MountCoordinates::MountCoordinates(double dec, double ra, pierside_t s) :
		dec_delta(dec), ra_delta(ra), side(s) {
}

MountCoordinates MountCoordinates::operator+(const IndexOffset& offset) const {
	return MountCoordinates(remainder(dec_delta + offset.dec_off, 360),
			remainder(ra_delta + offset.ra_off, 360), side);
}

MountCoordinates MountCoordinates::operator-(const IndexOffset& offset) const {
	return MountCoordinates(remainder(dec_delta - offset.dec_off, 360),
			remainder(ra_delta - offset.ra_off, 360), side);
}

AlignmentStar::AlignmentStar(const EquatorialCoordinates& ref,
		MountCoordinates meas, double t) :
		star_ref(ref), star_meas(meas), timestamp(t) {
}

EqCalibration::EqCalibration() :
		cone(0), error(0) {
}

EqCalibration::EqCalibration(const IndexOffset& off,
		const AzimuthalCoordinates p, double c, double e) :
		offset(off), pa(p), cone(c), error(e) {
}

CartesianVector CartesianVector::operator*(const Transformation &t) {
	return CartesianVector(t.a11 * x + t.a21 * y + t.a31 * z,
			t.a12 * x + t.a22 * y + t.a32 * z,
			t.a13 * x + t.a23 * y + t.a33 * z);
}

CartesianVector Transformation::operator*(const CartesianVector &vec) { // Left-product of matrix and vector
	return CartesianVector(a11 * vec.x + a12 * vec.y + a13 * vec.z,
			a21 * vec.x + a22 * vec.y + a23 * vec.z,
			a31 * vec.x + a32 * vec.y + a33 * vec.z);
}

AzimuthalCoordinates LocalEquatorialCoordinates::toAzimuthalCoordinates(
		const LocationCoordinates &loc) const {
	//              cphi             lambda             lambda0
	double c1 = cos(ha * DEGREE), c2 = cos(dec * DEGREE), c3 = cos(
			loc.lat * DEGREE);
	double s1 = sin(ha * DEGREE), s2 = sin(dec * DEGREE), s3 = sin(
			loc.lat * DEGREE);
	double s4 = c1 * c2 * c3 + s2 * s3;
	s4 = clamp(s4);
	double y1 = s1 * c2, x1 = s2 * c3 - c1 * c2 * s3;

	return AzimuthalCoordinates(asin(clamp(s4)) * RADIAN,
			atan2(y1, x1) * RADIAN);
}

LocalEquatorialCoordinates AzimuthalCoordinates::toLocalEquatorialCoordinates(
		const LocationCoordinates &loc) {
	//              mu             eps             lambda0
	double c1 = cos(azi * DEGREE), c2 = cos(alt * DEGREE), c3 = cos(
			loc.lat * DEGREE);
	double s1 = sin(azi * DEGREE), s2 = sin(alt * DEGREE), s3 = sin(
			loc.lat * DEGREE);
	double s4 = c1 * c2 * c3 + s2 * s3;
	double y1 = s1 * c2, x1 = s2 * c3 - c1 * c2 * s3;

	return LocalEquatorialCoordinates(asin(clamp(s4)) * RADIAN,
			atan2(y1, x1) * RADIAN);
}

LocalEquatorialCoordinates AlignmentStar::star_ref_local(
		const LocationCoordinates &loc) const {
	return star_ref.toLocalEquatorialCoordinates(timestamp, loc);
}

double getGreenwichMeanSiderealTime(double timestamp) {
	double jd = (double) timestamp * 1.1574074074074E-5 + 2440587.5 - 2451545.0; // Julian Date since J2000
//	double jd0 = floor(jd - 0.5) + 0.5; // JD of previous midnight
//	double cent = (jd - 2451545.0) / 36525;
//	double gmst = 6.697374558 + 0.06570982441908 * (jd0 - 2451545.0)
//			+ 1.00273790935 * (jd - jd0) * 24 + 0.000026 * cent * cent;
//	gmst *= 15.0;
	double gmst = 280.46061837 + 360.985647366 * jd; // Greenwich mean sidereal time (angle)
	return remainder(gmst, 360.0);
}

double getLocalSiderealTime(double timestamp, const LocationCoordinates &loc) {
	double gmst = getGreenwichMeanSiderealTime(timestamp);
	double lst = gmst + loc.lon * 1.00273790935; // Local sidereal time (angle)
	return remainder(lst, 360.0);
}

LocalEquatorialCoordinates EquatorialCoordinates::toLocalEquatorialCoordinates(
		double timestamp, const LocationCoordinates &loc) const {
// From phi to cphi
	return LocalEquatorialCoordinates(dec,
			remainder(getLocalSiderealTime(timestamp, loc) - ra, 360.0));
}

EquatorialCoordinates LocalEquatorialCoordinates::toEquatorial(double timestamp,
		const LocationCoordinates &loc) const {
// From cphi to phi
	return EquatorialCoordinates(dec,
			remainder(getLocalSiderealTime(timestamp, loc) - ha, 360.0));
}

Transformation &Transformation::getMisalignedPolarAxisTransformation(
		const AzimuthalCoordinates &p, const LocationCoordinates &loc) {
	double c1 = cos(p.azi * DEGREE), c2 = cos(p.alt * DEGREE), c3 = cos(
			loc.lat * DEGREE);
	double s1 = sin(p.azi * DEGREE), s2 = sin(p.alt * DEGREE), s3 = sin(
			loc.lat * DEGREE);
// Matrix to convert from basis vectors in misaligned PA to correct PA
	a11 = c1 * s2 * s3 + c2 * c3;
	a12 = -s1 * s3;
	a13 = -c1 * c2 * s3 + s2 * c3;
	a21 = s1 * s2;
	a22 = c1;
	a23 = -s1 * c2;
	a31 = -c1 * s2 * c3 + c2 * s3;
	a32 = s1 * c3;
	a33 = c1 * c2 * c3 + s2 * s3;
	return *this;
}

LocalEquatorialCoordinates LocalEquatorialCoordinates::applyPolarMisalignment(
		const Transformation& t) const {
	double c1 = cos(dec * DEGREE), c2 = cos(ha * DEGREE);
	double s1 = sin(dec * DEGREE), s2 = sin(ha * DEGREE);
	CartesianVector X = CartesianVector(c1 * c2, -c1 * s2, s1) * t;

	return LocalEquatorialCoordinates(asin(clamp(X.z)) * RADIAN,
			atan2(-X.y, X.x) * RADIAN);
}

LocalEquatorialCoordinates LocalEquatorialCoordinates::deapplyPolarMisalignment(
		const Transformation& t) const {
// the Transformation is ORTHOGONAL, T^-1 = T'
	double c1 = cos(dec * DEGREE), c2 = cos(ha * DEGREE);
	double s1 = sin(dec * DEGREE), s2 = sin(ha * DEGREE);
	Transformation tp = t;
	tp.transpose();
	CartesianVector X = CartesianVector(c1 * c2, -c1 * s2, s1) * t;

	return LocalEquatorialCoordinates(asin(clamp(X.z)) * RADIAN,
			atan2(-X.y, X.x) * RADIAN);
}

LocalEquatorialCoordinates LocalEquatorialCoordinates::applyPolarMisalignment(
		const AzimuthalCoordinates& mpa, const LocationCoordinates& loc) const {
	Transformation tr;
	tr.getMisalignedPolarAxisTransformation(mpa, loc);
	return applyPolarMisalignment(tr);
}

LocalEquatorialCoordinates LocalEquatorialCoordinates::deapplyPolarMisalignment(
		const AzimuthalCoordinates& mpa, const LocationCoordinates& loc) const {
	Transformation tr;
	tr.getMisalignedPolarAxisTransformation(mpa, loc);
	return deapplyPolarMisalignment(tr);
}

LocalEquatorialCoordinates LocalEquatorialCoordinates::applyConeError(
		double cone) const {
	return LocalEquatorialCoordinates(
			asin(clamp(sin(dec * DEGREE) / cos(cone * DEGREE))) * RADIAN,
			ha - asin(clamp(tan(dec * DEGREE) * tan(cone * DEGREE))) * RADIAN);
}

LocalEquatorialCoordinates LocalEquatorialCoordinates::deapplyConeError(
		double cone) const {
	double lmd = asin(clamp(sin(dec * DEGREE) * cos(cone * DEGREE))) * RADIAN;
	if (lmd > 90 - eps || lmd < -90 + eps) // This implies cone=0, so we don't do anything
		return LocalEquatorialCoordinates(*this);
	double phi = ha
			+ asin(clamp(tan(cone * DEGREE) * tan(lmd * DEGREE))) * RADIAN;
	return LocalEquatorialCoordinates(lmd, phi);
}

MountCoordinates LocalEquatorialCoordinates::toMountCoordinates(
		pierside_t side) const {
	MountCoordinates m;
	double ha = remainder(this->ha, 360.0);
	if (side == PIER_SIDE_WEST || (side == PIER_SIDE_AUTO && ha > 0)) {
		m.side = PIER_SIDE_WEST;
		m.dec_delta = 90.0 - dec; // dec_delta > 0
		m.ra_delta = ha - 90.0;
	} else {
		m.side = PIER_SIDE_EAST;
		m.dec_delta = dec - 90; // dec_delta<0
		m.ra_delta = ha + 90.0;
	}
	return m;
}

LocalEquatorialCoordinates MountCoordinates::toLocalEquatorialCoordinates() const {
	LocalEquatorialCoordinates a;
	if (side == PIER_SIDE_WEST || (side == PIER_SIDE_AUTO && (dec_delta > 0))) {
		a.ha = ra_delta + 90.0;
		a.dec = 90.0 - dec_delta;
	} else {
		a.ha = ra_delta - 90;
		a.dec = 90.0 + dec_delta;
	}
	return a;
}

IndexOffset alignOneStarForOffset(const AlignmentStar &star,
		const LocationCoordinates &loc) {
// Convert the reference star to Mount coordinates using the same pier side setting
	MountCoordinates mc = star.star_ref.toLocalEquatorialCoordinates(
			star.timestamp, loc).toMountCoordinates(star.star_meas.side);
	return IndexOffset(star.star_meas.dec_delta - mc.dec_delta,
			star.star_meas.ra_delta - mc.ra_delta);
}

double alignTwoStars(const AlignmentStar stars[],
		const LocationCoordinates& loc, AzimuthalCoordinates& pa,
		IndexOffset& offset) {
	// Initialize the PA and offset
	pa.alt = loc.lat;
	pa.azi = 0;
	offset = IndexOffset(0, 0);

	int i = 0;
	double residue = 1e10;
	Transformation t, t1, t2;
	bool diverge = false;

	while (i++ <= MAX_ITERATION && residue > tol) {
		t.getMisalignedPolarAxisTransformation(pa, loc);
		t1.getMisalignedPolarAxisTransformation(
				AzimuthalCoordinates(pa.alt + delta, pa.azi), loc);
		t2.getMisalignedPolarAxisTransformation(
				AzimuthalCoordinates(pa.alt, pa.azi + delta), loc);

		// Transform both starts and add offset
		MountCoordinates mc[2] =
				{
						stars[0].star_ref_local(loc).applyPolarMisalignment(t).toMountCoordinates(
								stars[0].star_meas.side) + offset,
						stars[1].star_ref_local(loc).applyPolarMisalignment(t).toMountCoordinates(
								stars[1].star_meas.side) + offset };
		MountCoordinates mc1[2] =
				{
						stars[0].star_ref_local(loc).applyPolarMisalignment(t1).toMountCoordinates(
								stars[0].star_meas.side) + offset,
						stars[1].star_ref_local(loc).applyPolarMisalignment(t1).toMountCoordinates(
								stars[1].star_meas.side) + offset };
		MountCoordinates mc2[2] =
				{
						stars[0].star_ref_local(loc).applyPolarMisalignment(t2).toMountCoordinates(
								stars[0].star_meas.side) + offset,
						stars[1].star_ref_local(loc).applyPolarMisalignment(t2).toMountCoordinates(
								stars[1].star_meas.side) + offset };

		// Calculate Jacobian matrix. Everything should be divided by delta
		// The 4x4 matrix has a special structure, it can be blocked as
		// J1  I
		// J2  I
		// Where J1 and J2 are the 2x2 Jacobians as in alignOneStar and I is 2x2 identity matrix
		// Calculate J1=J
		double j11 = mc1[0].dec_delta - mc[0].dec_delta, j12 = mc2[0].dec_delta
				- mc[0].dec_delta;
		double j21 = mc1[0].ra_delta - mc[0].ra_delta, j22 = mc2[0].ra_delta
				- mc[0].ra_delta;
		// Calculate J2=K
		double k11 = mc1[1].dec_delta - mc[1].dec_delta, k12 = mc2[1].dec_delta
				- mc[1].dec_delta;
		double k21 = mc1[1].ra_delta - mc[1].ra_delta, k22 = mc2[1].ra_delta
				- mc[1].ra_delta;
		double det = (j11 - k11) * (j22 - k22) - (j12 - k12) * (j21 - k21); // det(J) = det(J1-J2)
		if (det == 0) {
			diverge = true;
			break;
		}

		// Calculate invert of J1-J2
		double i11 = (j22 - k22) / det, i12 = -(j12 - k12) / det;
		double i21 = -(j21 - k21) / det, i22 = (j11 - k11) / det;
		// Calculate J2(J1-J2)^-1
		double l11 = k11 * i11 + k12 * i21, l12 = k11 * i12 + k12 * i22;
		double l21 = k21 * i11 + k22 * i21, l22 = k21 * i12 + k22 * i22;

		// Calculate F1, F2, F3, F4
		double f1 = mc[0].dec_delta - stars[0].star_meas.dec_delta;
		double f2 = mc[0].ra_delta - stars[0].star_meas.ra_delta;
		double f3 = mc[1].dec_delta - stars[1].star_meas.dec_delta;
		double f4 = mc[1].ra_delta - stars[1].star_meas.ra_delta;

		// Newton's Method - Calculate J^-1 * F
		// dp1,2 should be multiplied by delta
		double dp1 = i11 * f1 + i12 * f2 - i11 * f3 - i12 * f4;
		double dp2 = i21 * f1 + i22 * f2 - i21 * f3 - i22 * f4;
		double dp3 = -l11 * f1 - l12 * f2 + (1 + l11) * f3 + l12 * f4;
		double dp4 = -l21 * f1 - l22 * f2 + l21 * f3 + (1 + l22) * f4;

		// Update the coordinates
		pa.alt += -dp1 * delta;
		pa.azi += -dp2 * delta;
		offset.dec_off += -dp3;
		offset.ra_off += -dp4;

		residue = sqrt(f1 * f1 + f2 * f2 + f3 * f3 + f4 * f4); // calculate the difference
		debug_ptg(CM_DEBUG, "Iteration %i, %f\t%f\t%f\t%f\tdiff=%f\t %e %e\n", i,
				pa.alt, pa.azi, offset.dec_off, offset.ra_off, residue, det,
				det * (i11 * i22 - i12 * i21));
	}
	if (diverge) {
		/// Do something
		debug_ptg(CM_DEBUG, "Diverge\n");
		return INFINITY;
	}
	debug_ptg(CM_DEBUG, "Final delta: %.2e\n", residue);
	return residue;
}

static double jac[20][5]; // can maximally hold 10 stars
static double jacjac[5][5]; // J'J
static double invj[5][5];

static double det33(int a1, int a2, int a3, int b1, int b2, int b3) {
	return jacjac[a1][b1] * jacjac[a2][b2] * jacjac[a3][b3]
			+ jacjac[a1][b2] * jacjac[a2][b3] * jacjac[a3][b1]
			+ jacjac[a1][b3] * jacjac[a2][b1] * jacjac[a3][b2]
			- jacjac[a1][b1] * jacjac[a2][b3] * jacjac[a3][b2]
			- jacjac[a1][b2] * jacjac[a2][b1] * jacjac[a3][b3]
			- jacjac[a1][b3] * jacjac[a2][b2] * jacjac[a3][b1];
}

static double det44(int a1, int a2, int a3, int a4, int b1, int b2, int b3,
		int b4) {
	return jacjac[a1][b1] * det33(a2, a3, a4, b2, b3, b4)
			- jacjac[a1][b2] * det33(a2, a3, a4, b1, b3, b4)
			+ jacjac[a1][b3] * det33(a2, a3, a4, b1, b2, b4)
			- jacjac[a1][b4] * det33(a2, a3, a4, b1, b2, b3);
}

static void invert() {
	invj[0][0] = det44(1, 2, 3, 4, 1, 2, 3, 4);
	invj[1][0] = -det44(1, 2, 3, 4, 0, 2, 3, 4);
	invj[2][0] = det44(1, 2, 3, 4, 0, 1, 3, 4);
	invj[3][0] = -det44(1, 2, 3, 4, 0, 1, 2, 4);
	invj[4][0] = det44(1, 2, 3, 4, 0, 1, 2, 3);

	double det55 = invj[0][0] * jacjac[0][0] + invj[1][0] * jacjac[0][1]
			+ invj[2][0] * jacjac[0][2] + invj[3][0] * jacjac[0][3]
			+ invj[4][0] * jacjac[0][4];
	double idet55 = 1.0 / det55;

	invj[0][0] *= idet55;
	invj[1][0] *= idet55;
	invj[2][0] *= idet55;
	invj[3][0] *= idet55;
	invj[4][0] *= idet55;

	invj[0][1] = -det44(0, 2, 3, 4, 1, 2, 3, 4) * idet55;
	invj[1][1] = det44(0, 2, 3, 4, 0, 2, 3, 4) * idet55;
	invj[2][1] = -det44(0, 2, 3, 4, 0, 1, 3, 4) * idet55;
	invj[3][1] = det44(0, 2, 3, 4, 0, 1, 2, 4) * idet55;
	invj[4][1] = -det44(0, 2, 3, 4, 0, 1, 2, 3) * idet55;

	invj[0][2] = det44(0, 1, 3, 4, 1, 2, 3, 4) * idet55;
	invj[1][2] = -det44(0, 1, 3, 4, 0, 2, 3, 4) * idet55;
	invj[2][2] = det44(0, 1, 3, 4, 0, 1, 3, 4) * idet55;
	invj[3][2] = -det44(0, 1, 3, 4, 0, 1, 2, 4) * idet55;
	invj[4][2] = det44(0, 1, 3, 4, 0, 1, 2, 3) * idet55;

	invj[0][3] = -det44(0, 1, 2, 4, 1, 2, 3, 4) * idet55;
	invj[1][3] = det44(0, 1, 2, 4, 0, 2, 3, 4) * idet55;
	invj[2][3] = -det44(0, 1, 2, 4, 0, 1, 3, 4) * idet55;
	invj[3][3] = det44(0, 1, 2, 4, 0, 1, 2, 4) * idet55;
	invj[4][3] = -det44(0, 1, 2, 4, 0, 1, 2, 3) * idet55;

	invj[0][4] = det44(0, 1, 2, 3, 1, 2, 3, 4) * idet55;
	invj[1][4] = -det44(0, 1, 2, 3, 0, 2, 3, 4) * idet55;
	invj[2][4] = det44(0, 1, 2, 3, 0, 1, 3, 4) * idet55;
	invj[3][4] = -det44(0, 1, 2, 3, 0, 1, 2, 4) * idet55;
	invj[4][4] = det44(0, 1, 2, 3, 0, 1, 2, 3) * idet55;
}

static inline double sqr(double x) {
	return x * x;
}

static void get_corrected_coords(const int N, MountCoordinates mcs[],
		const LocalEquatorialCoordinates star_ref_local[],
		const MountCoordinates star_meas[], const LocationCoordinates& loc,
		const AzimuthalCoordinates& pa, const IndexOffset& offset,
		double cone) {
	static Transformation t;
	t.getMisalignedPolarAxisTransformation(pa, loc);
	for (int i = 0; i < N; i++) {
		mcs[i] = star_ref_local[i].applyPolarMisalignment(t).applyConeError(
				cone).toMountCoordinates(star_meas[i].side) + offset;
	}
}

static void fill_jacobian(const int N, const int j, MountCoordinates stars0[],
		MountCoordinates stars1[], const double &dd) {
	for (int i = 0; i < N; i++) {
		jac[i * 2][j] = (stars1[i].dec_delta - stars0[i].dec_delta) / dd;
		jac[i * 2 + 1][j] = (stars1[i].ra_delta - stars0[i].ra_delta) / dd;
	}
}

double alignNStars(const int N, const AlignmentStar stars[],
		const LocationCoordinates& loc, AzimuthalCoordinates& pa,
		IndexOffset& offset, double& cone) {
	if (N == 2) {
		cone = 0;
		return alignTwoStars(stars, loc, pa, offset);
	}
	if (N <= 1) {
		return INFINITY;
	}

	// Assuming the cone error is not huge, we should be fairly close to the local minimum
	int i = 0;
	double residue = 1e10;
	LocalEquatorialCoordinates star_ref_local[MAXN];
	MountCoordinates star_meas[MAXN];
	MountCoordinates stars0[MAXN], stars1[MAXN];
	double dp[5];
	double f[20];

	for (int i = 0; i < N; i++) {
		star_ref_local[i] = stars[i].star_ref_local(loc);
		star_meas[i] = stars[i].star_meas;
	}

	bool diverge = true;

	while (i++ < MAX_ITERATION_OPTIMIZATION && residue > tol) {
		// Calulate Jacobian
		get_corrected_coords(N, stars0, star_ref_local, star_meas, loc, pa,
				offset, cone);
		/*Vary pa.alt*/
		get_corrected_coords(N, stars1, star_ref_local, star_meas, loc,
				AzimuthalCoordinates(pa.alt + delta, pa.azi), offset, cone);
		fill_jacobian(N, 0, stars0, stars1, delta);
		/*Vary pa.azi*/
		get_corrected_coords(N, stars1, star_ref_local, star_meas, loc,
				AzimuthalCoordinates(pa.alt, pa.azi + delta), offset, cone);
		fill_jacobian(N, 1, stars0, stars1, delta);
		/*Vary offset.dec*/
		get_corrected_coords(N, stars1, star_ref_local, star_meas, loc, pa,
				IndexOffset(offset.dec_off + delta, offset.ra_off), cone);
		fill_jacobian(N, 2, stars0, stars1, delta);
		/*Vary offset.ha*/
		get_corrected_coords(N, stars1, star_ref_local, star_meas, loc, pa,
				IndexOffset(offset.dec_off, offset.ra_off + delta), cone);
		fill_jacobian(N, 3, stars0, stars1, delta);
		/*Vary cone*/
		get_corrected_coords(N, stars1, star_ref_local, star_meas, loc, pa,
				offset, cone + delta);
		fill_jacobian(N, 4, stars0, stars1, delta);

		// The Jacobian is now filled. It is 2*N rows and 5 columns
		// Gauss-Newton method: x_n - x_(n-1) = - (J'J)^-1J' * f_(n-1)
		// 1. Matrix multiplication
		int p, q, r;

		for (p = 0; p < 5; p++) {
			for (q = 0; q < 5; q++) {
				double s = 0;
				for (r = 0; r < 2 * N; r++)
					s += jac[r][p] * jac[r][q];
				jacjac[p][q] = s;
			}
		}

		// 2. Matrix inversion
		invert();

		// 3. Calculate f_(n-1)
		double newresidue = 0;
		for (p = 0; p < N; p++) {
			f[2 * p] = stars0[p].dec_delta - star_meas[i].dec_delta;
			f[2 * p + 1] = stars0[p].ra_delta - star_meas[i].ra_delta;
			newresidue += sqr(f[2 * p]) + sqr(f[2 * p + 1]);
		}
		newresidue = sqrt(newresidue);
		// 4. Matrix multiplication
		for (p = 0; p < 5; p++) {
			double s = 0;
			for (q = 0; q < 5; q++) {
				for (r = 0; r < 2 * N; r++) {
					s += invj[p][q] * jac[r][q] * f[r];
				}
			}
			dp[p] = -s;
		}
		// 5. Apply the correction
		pa.alt += dp[0];
		pa.azi += dp[1];
		offset.dec_off += dp[2];
		offset.ra_off += dp[3];
		cone += dp[4];

		if (newresidue >= residue - tol) {
			debug_ptg(CM_DEBUG, "Converged.\n");
			diverge = false;
			break;
		} else {
			residue = newresidue;
		}
		debug_ptg(CM_DEBUG, "Iteration %i, %f\t%f\t%f\t%f\t%f\tr=%f\n", i,
				pa.alt, pa.azi, offset.dec_off, offset.ra_off, cone, residue);
	}

	if (diverge) {
		debug_ptg(CM_DEBUG, "Diverged.\n");
		return INFINITY;
	}

	debug_ptg(CM_DEBUG, "Final result: %f\t%f\t%f\t%f\t%f\tr=%f\n", pa.alt,
			pa.azi, offset.dec_off, offset.ra_off, cone, residue);
	return residue;

}

EqCalibration alignAuto(const int N, const AlignmentStar stars[],
		const LocationCoordinates &loc) {
	EqCalibration calib; // Zero everything
	calib.pa.alt = loc.lat;
	if (N == 1) {
		calib.offset = alignOneStarForOffset(stars[0], loc);
		return calib; // Always success
	} else {
		if (N == 2) {
			alignTwoStars(stars, loc, calib.pa, calib.offset);
		} else if (N <= MAXN) {
			alignNStars(N, stars, loc, calib.pa, calib.offset, calib.cone);
		}
		else{
			// Only use the first MAXN stars
			alignNStars(MAXN, stars, loc, calib.pa, calib.offset, calib.cone);
		}
	}
	// Calculate RMS error
	static Transformation t;
	t.getMisalignedPolarAxisTransformation(calib.pa, loc);
	double r = 0;
	for (int i = 0; i < N; i++) {
		// Misalign, apply cone error, and transform to mount coordinates using the same pier side as in the measured stars
		MountCoordinates mc =
				stars[i].star_ref_local(loc).applyPolarMisalignment(t).applyConeError(
						calib.cone).toMountCoordinates(stars[i].star_meas.side)
						+ calib.offset;
		// Square sum
		r += sqr(mc.ra_delta - stars[i].star_meas.ra_delta)
				+ sqr(mc.dec_delta - stars[i].star_meas.dec_delta);
	}
	calib.error = sqrt(r);
	return calib;
}

double parseHMSAngle(char* hms) {
	char *h = strchr(hms, 'h');
	char *m = strchr(hms, 'm');
	char *s = strchr(hms, 's');
	if (h == NULL || m == NULL || s == NULL || !(h < m && m < s)) {
		return NAN;
	}

	*h = '\0';
	*m = '\0';
	*s = '\0';

	char *tp;
	int hour = strtol(hms, &tp, 10);
	if (tp == hms) {
		return NAN;
	}
	int minute = strtol(h + 1, &tp, 10);
	if (tp == h + 1) {
		return NAN;
	}
	double second = strtod(m + 1, &tp);
	if (tp == m + 1) {
		return NAN;
	}

	if (!(hour >= 0 && hour <= 23 && minute >= 0 && minute <= 59 && second >= 0
			&& second <= 60)) {
		return NAN;
	}

	return remainder((hour + minute / 60.0 + second / 3600.0) * 15, 360);
}

double parseDMSAngle(char* dms) {
	char *d = strchr(dms, 'd');
	char *m = strchr(dms, 'm');
	char *s = strchr(dms, 's');
	if (d == NULL || m == NULL || s == NULL || !(d < m && m < s)) {
		return NAN;
	}

	*d = '\0';
	*m = '\0';
	*s = '\0';

	char *tp;
	int degree = strtol(dms, &tp, 10);
	if (tp == dms) {
		return NAN;
	}
	int arcminute = strtol(d + 1, &tp, 10);
	if (tp == d + 1) {
		return NAN;
	}
	double arcsecond = strtod(m + 1, &tp);
	if (tp == m + 1) {
		return NAN;
	}

	if (!(degree >= -180.0 && degree <= 180.0 && arcminute >= 0
			&& arcminute <= 59 && arcsecond >= 0 && arcsecond <= 60)) {
		return NAN;
	}

	return remainder((degree + arcminute / 60.0 + arcsecond / 3600.0), 360);
}

double kingRate(EquatorialCoordinates eq, LocationCoordinates loc,
		double time) {
	LocalEquatorialCoordinates leq = eq.toLocalEquatorialCoordinates(time, loc);
//	AzimuthalCoordinates ac = CelestialMath::localEquatorialToAzimuthal(leq,
//			loc);
	double cosLat = cos(loc.lat * DEGREE);
	double sinLat = sin(loc.lat * DEGREE);
	double cotLat = cosLat / sinLat;
	double cosDec = cos(eq.dec * DEGREE);
	double sinDec = sin(eq.dec * DEGREE);
	double tanDec = sinDec / cosDec;
	double cosHA = cos(leq.ha * DEGREE);
	double kingMpD = (1436.46
			+ 0.4
					* (cosLat / cosDec
							* (cosLat * cosDec + sinLat * sinDec * cosHA)
							/ pow(sinLat * sinDec + cosLat * cosDec * cosHA,
									2.0) - cotLat * tanDec * cosHA));
	return 6.0 / kingMpD;
}

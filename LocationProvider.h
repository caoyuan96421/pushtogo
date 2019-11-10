#ifndef _LOCATION_PROVIDER_H
#define _LOCATION_PROVIDER_H

#include "CelestialMath.h"
/**
 * Provides location information. Can be overriden if a GPS is installed for example.
 */
class LocationProvider {
public:
	LocationProvider() {
	}
	virtual ~LocationProvider() {
	}

	virtual double getLongtitude() const;

	virtual double getLatitude() const;

	LocationCoordinates getLocation() const {
		return LocationCoordinates(getLatitude(), getLongtitude());
	}
};


#endif

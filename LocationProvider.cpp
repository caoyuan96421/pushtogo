/*
 * LocationCoordinates.cpp
 *
 *  Created on: Sep 14, 2018
 *      Author: caoyu
 */

#include "LocationProvider.h"
#include "TelescopeConfiguration.h"

double LocationProvider::getLongtitude() const {
	return TelescopeConfiguration::getDouble("longitude");
}

double LocationProvider::getLatitude() const {
	return TelescopeConfiguration::getDouble("latitude");
}

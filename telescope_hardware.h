/*
 * telescope_hardware.h
 *
 *  Created on: 2018Äê3ÔÂ1ÈÕ
 *      Author: caoyuan9642
 */

#ifndef TELESCOPE_HARDWARE_H_
#define TELESCOPE_HARDWARE_H_

#include "mbed.h"
#include "EquatorialMount.h"

/**
 * These functions must be implemented in a separate telescope_hardware.cpp, adapted to your specific hardware configuration
 */

EquatorialMount &telescopeHardwareInit();

osStatus telescopeServerInit();

osStatus telescopeConfigurationWriteback();



#endif /* TELESCOPE_HARDWARE_H_ */


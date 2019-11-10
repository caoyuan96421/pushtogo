/*
 * debug.h
 *
 *  Created on: Oct 6, 2019
 *      Author: caoyu
 */

#ifndef PUSHTOGO_DEBUG_H_
#define PUSHTOGO_DEBUG_H_

#include <stdarg.h>

typedef enum {
	DEFAULT_DEBUG = 0,
	AXIS_DEBUG,		// Axis debug
	EM_DEBUG,		// EquatorialMount debug
	SVR_DEBUG,		// Server debug
	CM_DEBUG,		// CelestialMath debug
	HW_DEBUG		// Hardware debug
} ptgDbg;

void debug_ptg(ptgDbg dbg, const char *fmt, ...);
void debug_ptg(const char *fmt, ...);

// Force debug to generate error
//#define debug_if(...) error("No debug allowed")
//#define debug(...) error("No debug allowed")


#endif /* PUSHTOGO_DEBUG_H_ */

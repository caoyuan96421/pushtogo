/*
 * debug.c
 *
 *  Created on: Oct 6, 2019
 *      Author: caoyu
 */

#include "debug.h"
#include "mbed.h"
#include "printf.h"

void debug_ptg(ptgDbg dbg, const char *fmt, ...) {
	char buf[512];
	int len = 0;
	switch (dbg) {
	case AXIS_DEBUG:
		len = sprintf(buf, "AXIS: ");
		break;
	case EM_DEBUG:
		len = sprintf(buf, "EM: ");
		break;
	case SVR_DEBUG:
		return;
//		len = sprintf(buf, "SVR: ");
//		break;
	case CM_DEBUG:
		len = sprintf(buf, "MATH: ");
		break;
	case HW_DEBUG:
		len = sprintf(buf, "HW: ");
		break;
	default:
		break;
	}
	va_list args;
	va_start(args, fmt);
	len += vsnprintf(buf + len, sizeof(buf) - len, fmt, args);
	va_end(args);

	buf[sizeof(buf) - 1] = '\0'; // Force termination
	fputs(buf, stdout); // Output to STDOUT
}

void debug_ptg(const char *fmt, ...) {
	char buf[512];
	int len = 0;
	va_list args;
	va_start(args, fmt);
	len = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	buf[sizeof(buf) - 1] = '\0'; // Force termination
	fputs(buf, stdout); // Output to STDOUT
}

/*
 * PEC.cpp
 *
 *  Created on: Sep 9, 2018
 *      Author: caoyu
 */

#include "PEC.h"
#include "Axis.h"
#include "config/TelescopeConfiguration.h"
#include <math.h>

PEC::PEC(Axis &a) :
		axis(a), enabled(false), indexOffset(0) {
	thread = new Thread(osPriorityAboveNormal,
	OS_STACK_SIZE / 4, NULL, "PEC Thread");
	thread->start(callback(this, &PEC::task));

	granularity = TelescopeConfiguration::getInt("pec_granularity");
	pecData = new float[granularity];
	for (int i = 0; i < granularity; i++) {
		pecData[i] = 0.0f;
	}
}

void PEC::task() {

	Timer t;
	t.start();
	while (true) {
		if (enabled) {
			double degTeeth = 360.0
					/ TelescopeConfiguration::getInt("worm_teeth"); // Degree per teeth
			double period = degTeeth
					/ (axis.getTrackSpeedSidereal() * sidereal_speed); // Worm rotation period

			int currentIndex = (int) floor(
					remainder((axis.getAngleDeg() - indexOffset), degTeeth)
							* granularity);
			if (currentIndex < 0)
				currentIndex += granularity;

			double interval = period / granularity; // Time to wait between executing each step

			uint64_t startTimeUs = t.read_high_resolution_us();
			uint64_t count = 0;
			double intervalUs = interval * 1000000.0;

			while (enabled) {
				float correction = pecData[currentIndex]; // Correction in arcseconds
				if (++currentIndex >= granularity)
					currentIndex -= granularity;

				if (correction != 0.0f) {
					// Calculate guide time in milliseconds
					// 3.6 = 3600 (arcsec/degree) * 1000 (ms/s)
					int guidetime = correction / float(axis.guideSpeed) / 3.6f;

					// Positive guidetime means needs to correct towards east
					// Negative guidetime means needs to correct towards west
					axis.guide(AXIS_ROTATE_NEGATIVE, guidetime);
				}

				// Make sure mean interval is very accurate
				count++;
				while ((t.read_high_resolution_us() - startTimeUs)
						< intervalUs * count) {
					ThisThread::sleep_for(1ms);
				}
			}
		}
		ThisThread::sleep_for(10ms);
	}
}

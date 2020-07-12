/*
 * EqMountServer.cpp
 *
 *  Created on: 2018��3��1��
 *      Author: caoyuan9642
 */

#include "EqMountServer.h"
#include "mbed_events.h"
#include <ctype.h>
#include <time.h>
#include "printf.h"

extern ServerCommand commandlist[MAX_COMMAND];

void svprintf(EqMountServer *server, const char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	server->vfprint(fmt, args);
	va_end(args);
}

void EqMountServer::vfprint(const char *fmt, va_list arg) {
	char buf[1024];
	int len = vsnprintf(buf, sizeof(buf), fmt, arg);
	stream_mutex.lock();
	stream.write(buf, len);
	stream_mutex.unlock();
}

EqMountServer::EqMountServer(FileHandle &stream, bool echo) :
		eq_mount(NULL), stream(stream), thread(osPriorityBelowNormal,
		OS_STACK_SIZE, NULL, "EqMountServer"), echo(echo) {
	thread.start(callback(this, &EqMountServer::task_thread));
}

EqMountServer::~EqMountServer() {
	thread.terminate();
}

void EqMountServer::task_thread() {
	EventQueue queue(16 * EVENTS_EVENT_SIZE);
	Thread evq_thd(osThreadGetPriority(ThisThread::get_id()), OS_STACK_SIZE,
			NULL, "EqMountServer dispatcher");
	evq_thd.start(callback(&queue, &EventQueue::dispatch_forever));

	while (true) {
		const int size = 256;
		char *buffer = new char[size]; // text buffer
		if (!buffer) {
			error("EqMountServer: out of memory\r\n");
			break;
		}
		char x = 0;
		int i = 0;
		while (i < size - 2) {
			int s = stream.read(&x, 1);
			if (s <= 0) { // Device not ready yet
				ThisThread::sleep_for(100); // Wait for device to come up
				continue;
			} else if (x == '\r' || x == '\n') { // End of line
				break;
			} else if (x == '\b' || x == '\x7F') { // Backspace
				if (i > 0) {
					svprintf(this, "\b \b"); // blank the current character properly
					i--;
				}
				continue;
			} else if (isspace(x)) { // Convert to white space
				x = ' ';
			} else if (!isprint(x)) { // Ignore everything else
				continue;
			}
			// Echo
			if (echo) {
				stream.write(&x, 1);
			}
			buffer[i++] = (char) x;
		}
		if (echo) {
			// Echo new line character after command
			svprintf(this, "\r\n");
		}
		if (i == 0) { // Empty command
			delete[] buffer;
			continue;
		}
		buffer[i] = '\0'; // insert null character

		if (eq_mount == NULL) {
			svprintf(this, "Error: EqMount not binded.\r\n");
			delete[] buffer;
			continue;
		}

		char delim[] = " "; // Delimiter, can be any white character in the actual input
		char *saveptr;

		char *command = strtok_r(buffer, delim, &saveptr); // Get the first token

		if (command == NULL || strlen(command) == 0) { // Empty command
			delete[] buffer;
			continue;
		}

		for (char *p = command; *p; ++p)
			*p = tolower(*p); // Convert to lowercase

		char **args = new char*[16];

		// Extract parameters
		i = 0;
		do {
			args[i] = strtok_r(NULL, delim, &saveptr);
			if (args[i] == NULL || ++i == 16)
				break;
		} while (true);

		int argn = i;

		debug_ptg(SVR_DEBUG, "command: |%s|\r\n", command);
		for (i = 0; i < argn; i++) {
			debug_ptg(SVR_DEBUG, "  arg[%i]=|%s|\r\n", i, args[i]);
			for (char *p = args[i]; *p; ++p)
				*p = tolower(*p); // Convert to lowercase
		}

		int cind = -1;

		for (i = 0; i < MAX_COMMAND && commandlist[i].fptr != NULL; i++) {
			if (strcmp(commandlist[i].cmd, command) == 0) {
				cind = i;
				break;
			}
		}

		if (cind == -1) {
			debug_ptg(SVR_DEBUG, "Error: command %s not found.\r\n", command);
			svprintf(this, "Unknown command %s\r\n", command);
			delete[] buffer;
			delete[] args;
			continue;
		}

		// Commands that can return immediately, directly run them
		if (cind < 8 || strcmp(command, "config") == 0) {
			int ret = commandlist[cind].fptr(this, command, argn, args);
			// Send the return status back
			svprintf(this, "%d %s\r\n", ret, command);
			delete[] buffer;
			delete[] args;
			continue;
		}

		// Queue the command
		Callback<void(ServerCommand&, int, char**, char*)> cb = callback(this,
				&EqMountServer::command_execute);
		while (queue.call(cb, commandlist[cind], argn, args, buffer) == 0) { // Use the event dispatching thread to run this
			debug_ptg(SVR_DEBUG, "Event queue full. Wait...\r\n");
			ThisThread::sleep_for(100);
		}

		// The buffer and argument list will be deleted when the command finishes execution in the event dispatch thread
	}
}

void EqMountServer::command_execute(ServerCommand &cmd, int argn, char *args[],
		char *buffer) {
	int ret = cmd.fptr(this, cmd.cmd, argn, args);

	if (ret == ERR_WRONG_NUM_PARAM) {
		// Wrong number of arguments
		debug_ptg(SVR_DEBUG, "Error: %s wrong number of args.\r\n", cmd.cmd);
	} else if (ret == 2) {
		// Wrong number of arguments
		debug_ptg(SVR_DEBUG, "Error: %s parameters out of range.\r\n", cmd.cmd);
	} else if (ret) {
		debug_ptg(SVR_DEBUG, "Error: %s returned code %d.\r\n", cmd.cmd, ret);
	}

	// Send the return status back
	svprintf(this, "%d %s\r\n", ret, cmd.cmd);

	delete[] buffer;
	delete[] args;
}

static int eqmount_stop(EqMountServer *server, const char *cmd, int argn,
		char *argv[]) {
	if (argn == 0) {
		// Stop nudging or slewing
		if (server->getEqMount()->getStatus() & MOUNT_NUDGING) {
			server->getEqMount()->stopNudge();
		} else if (server->getEqMount()->getStatus() == MOUNT_SLEWING) {
			server->getEqMount()->stopAsync();
		}
	} else if (argn == 1 && strcmp(argv[0], "track") == 0) {
		server->getEqMount()->stopTracking();
	} else {
		return ERR_WRONG_NUM_PARAM;
	}
	return 0;
}

static int eqmount_estop(EqMountServer *server, const char *cmd, int argn,
		char *argv[]) {
	server->getEqMount()->emergencyStop();
	return 0;
}

static int eqmount_goto(EqMountServer *server, const char *cmd, int argn,
		char *argv[]) {
	if (argn == 1) {
		if (strcmp(argv[0], "index") == 0) {
			return server->getEqMount()->goToIndex();
		}
	} else if (argn == 2 || (argn == 3 && strcmp(argv[0], "eq") == 0)) {
		if (argn == 3 && strcmp(argv[0], "eq") == 0) {
			argv[0] = argv[1];
			argv[1] = argv[2];
		}
		char *tp;
		// First try HMS format
		double ra = parseHMSAngle(argv[0]);
		if (isnan(ra)) {
			// If doesn't work, then we use as a double
			ra = strtod(argv[0], &tp);
			if (tp == argv[0]) {
				return ERR_PARAM_OUT_OF_RANGE;
			}
		}
		// First try DMS format
		double dec = parseDMSAngle(argv[1]);
		if (isnan(dec)) {
			// If doesn't work, then we use as a double
			dec = strtod(argv[1], &tp);
			if (tp == argv[1]) {
				return ERR_PARAM_OUT_OF_RANGE;
			}
		}

		if (!((ra <= 180.0) && (ra >= -180.0) && (dec <= 90.0) && (dec >= -90.0)))
			return ERR_PARAM_OUT_OF_RANGE;

		osStatus s;
		if ((s = server->getEqMount()->goTo(ra, dec)) != osOK)
			return s;
	} else if (argn == 3) {
		if (strcmp(argv[0], "mount") == 0) {
			char *tp;
			double ra = strtod(argv[1], &tp);
			if (tp == argv[1]) {
				return ERR_PARAM_OUT_OF_RANGE;
			}
			double dec = strtod(argv[2], &tp);
			if (tp == argv[2]) {
				return ERR_PARAM_OUT_OF_RANGE;
			}

			if (!((ra <= 180.0) && (ra >= -180.0) && (dec <= 180.0)
					&& (dec >= -180.0)))
				return ERR_PARAM_OUT_OF_RANGE;

			osStatus s;
			if ((s = server->getEqMount()->goToMount(MountCoordinates(dec, ra)))
					!= osOK)
				return s;
		}
	} else {
		return ERR_WRONG_NUM_PARAM;
	}

	return 0;
}

static int eqmount_speed(EqMountServer *server, const char *cmd, int argn,
		char *argv[]) {
	if (argn == 1) {
		// Print speed
		double speed = 0;
		if (strcmp(argv[0], "slew") == 0) {
			speed = server->getEqMount()->getSlewSpeed();
		} else if (strcmp(argv[0], "track") == 0) {
			speed = server->getEqMount()->getTrackSpeedSidereal();
		} else if (strcmp(argv[0], "guide") == 0) {
			speed = server->getEqMount()->getGuideSpeedSidereal();
		} else {
			return ERR_PARAM_OUT_OF_RANGE;
		}
		svprintf(server, "%s %f\r\n", cmd, speed);
	} else if (argn == 2) {
		char *tp;
		double speed = strtod(argv[1], &tp);
		if (tp == argv[1]) {
			return ERR_PARAM_OUT_OF_RANGE;
		}
		if (strcmp(argv[0], "slew") == 0) {
			if (speed <= 0
					|| speed > TelescopeConfiguration::getDouble("max_speed")) {
				return ERR_PARAM_OUT_OF_RANGE;
			}
			server->getEqMount()->setSlewSpeed(speed);
		} else if (strcmp(argv[0], "track") == 0) {
			if (speed <= 0 || speed > 64) {
				return ERR_PARAM_OUT_OF_RANGE;
			}
			server->getEqMount()->setTrackSpeedSidereal(speed);
		} else if (strcmp(argv[0], "guide") == 0) {
			if (speed <= 0 || speed > 64) {
				return ERR_PARAM_OUT_OF_RANGE;
			}
			server->getEqMount()->setGuideSpeedSidereal(speed);
		}
	} else {
		return ERR_WRONG_NUM_PARAM;
	}

	return 0;
}

static int eqmount_align(EqMountServer *server, const char *cmd, int argn,
		char *argv[]) {
	if (argn == 0) {
		svprintf(server,
				"%s usage: align add [star]\r\nalign replace [n] [star]\r\nalign delete [n]\r\nalign show\r\nalign show [n]\r\nalign clear\r\nalign set\r\nalign force\r\nalign update\r\n",
				cmd);
		return ERR_WRONG_NUM_PARAM;
	}
	if (strcmp(argv[0], "add") == 0) {
		AlignmentStar as;
		if (argn != 3 && argn != 5) {
			svprintf(server,
					"%s usage: align add {ref_ra} {ref_dec}\r\n%s usage: align add {ref_ra} {ref_dec} {meas_ra} {meas_dec}\r\n",
					cmd, cmd);
			return ERR_WRONG_NUM_PARAM;
		}
		char *tp;
		double ref_ra = strtod(argv[1], &tp);
		if (tp == argv[1]) {
			return ERR_PARAM_OUT_OF_RANGE;
		}
		double ref_dec = strtod(argv[2], &tp);
		if (tp == argv[2]) {
			return ERR_PARAM_OUT_OF_RANGE;
		}
		as.star_ref = EquatorialCoordinates(ref_dec, ref_ra);
		if (argn == 5) {
			double meas_ra = strtod(argv[3], &tp);
			if (tp == argv[3]) {
				return ERR_PARAM_OUT_OF_RANGE;
			}
			double meas_dec = strtod(argv[4], &tp);
			if (tp == argv[4]) {
				return ERR_PARAM_OUT_OF_RANGE;
			}
			as.star_meas = MountCoordinates(meas_dec, meas_ra);
		} else
			as.star_meas = server->getEqMount()->getMountCoordinates();
		as.timestamp = server->getEqMount()->getClock().getTime();
		return server->getEqMount()->addAlignmentStar(as);
	} else if (strcmp(argv[0], "replace") == 0) {
		AlignmentStar as;
		if (argn != 4 && argn != 6) {
			svprintf(server,
					"%s usage: align replace [index] {ref_ra} {ref_dec}\r\n%s usage: align replace [index] {ref_ra} {ref_dec} {meas_ra} {meas_dec}\r\n",
					cmd, cmd);
			return ERR_WRONG_NUM_PARAM;
		}
		char *tp;

		int index = strtol(argv[1], &tp, 10);
		if (tp == argv[1]) {
			return ERR_PARAM_OUT_OF_RANGE;
		}

		double ref_ra = strtod(argv[2], &tp);
		if (tp == argv[2]) {
			return ERR_PARAM_OUT_OF_RANGE;
		}
		double ref_dec = strtod(argv[3], &tp);
		if (tp == argv[3]) {
			return ERR_PARAM_OUT_OF_RANGE;
		}
		as.star_ref = EquatorialCoordinates(ref_dec, ref_ra);
		if (argn == 6) {
			double meas_ra = strtod(argv[4], &tp);
			if (tp == argv[4]) {
				return ERR_PARAM_OUT_OF_RANGE;
			}
			double meas_dec = strtod(argv[5], &tp);
			if (tp == argv[5]) {
				return ERR_PARAM_OUT_OF_RANGE;
			}
			as.star_meas = MountCoordinates(meas_dec, meas_ra);
		} else
			as.star_meas = server->getEqMount()->getMountCoordinates();
		as.timestamp = server->getEqMount()->getClock().getTime();
		return server->getEqMount()->replaceAlignmentStar(index, as);
	} else if (strcmp(argv[0], "delete") == 0) {
		char *tp;
		int n = strtol(argv[1], &tp, 10);
		if (tp == argv[1] || n >= server->getEqMount()->getNumAlignmentStar()
				|| n < 0) {
			return ERR_PARAM_OUT_OF_RANGE;
		}
		int ret = server->getEqMount()->removeAlignmentStar(n);
		if (server->getEqMount()->getNumAlignmentStar() == 0) {
			server->getEqMount()->clearCalibrationExceptOffsets();
		}
		return ret;
	} else if (strcmp(argv[0], "show") == 0) {
		if (argn == 1) {
			svprintf(server, "%s offset %.8f %.8f\r\n", cmd,
					server->getEqMount()->getCalibration().offset.ra_off,
					server->getEqMount()->getCalibration().offset.dec_off);
			svprintf(server, "%s pa %.8f %.8f\r\n", cmd,
					server->getEqMount()->getCalibration().pa.alt,
					server->getEqMount()->getCalibration().pa.azi);
			svprintf(server, "%s cone %.8f\r\n", cmd,
					server->getEqMount()->getCalibration().cone);
			svprintf(server, "%s error %g\r\n", cmd,
					server->getEqMount()->getCalibration().error);
		} else if (argn == 2) {
			if (strcmp(argv[1], "num") == 0) {
				svprintf(server, "%s %d\r\n", cmd,
						server->getEqMount()->getNumAlignmentStar());
			} else {
				char *tp;
				int index = strtol(argv[1], &tp, 0);
				if (tp == argv[1]
						|| index >= server->getEqMount()->getNumAlignmentStar()
						|| index < 0) {
					return ERR_PARAM_OUT_OF_RANGE;
				}
				AlignmentStar *as = server->getEqMount()->getAlignmentStar(
						index);
				if (!as) {
					return ERR_PARAM_OUT_OF_RANGE;
				}
				svprintf(server, "%s %.8f %.8f %.8f %.8f %d\r\n", cmd,
						as->star_ref.ra, as->star_ref.dec,
						as->star_meas.ra_delta, as->star_meas.dec_delta,
						as->timestamp);
			}
		} else {
			return ERR_WRONG_NUM_PARAM;
		}
	} else if (strcmp(argv[0], "clear") == 0) {
		if (argn != 1) {
			return ERR_WRONG_NUM_PARAM;
		}
		server->getEqMount()->clearCalibration();
	} else if (strcmp(argv[0], "convert") == 0) {
		if (argn != 4) {
			return ERR_WRONG_NUM_PARAM;
		}
		char *tp;
		double ra = strtod(argv[2], &tp);
		if (tp == argv[2]) {
			return ERR_PARAM_OUT_OF_RANGE;
		}
		double dec = strtod(argv[3], &tp);
		if (tp == argv[3]) {
			return ERR_PARAM_OUT_OF_RANGE;
		}
		if (strcmp(argv[1], "mount") == 0) {
			// Convert to eq
			EquatorialCoordinates eq =
					server->getEqMount()->convertToEqCoordinates(
							MountCoordinates(dec, ra));
			svprintf(server, "%s %.8f %.8f\r\n", cmd, eq.ra, eq.dec);
		} else if (strcmp(argv[1], "eq") == 0) {
			// Convert to eq
			MountCoordinates mc =
					server->getEqMount()->convertToMountCoordinates(
							EquatorialCoordinates(dec, ra));
			svprintf(server, "%s %.8f %.8f\r\n", cmd, mc.ra_delta,
					mc.dec_delta);
		} else
			return ERR_PARAM_OUT_OF_RANGE;
	} else if (strcmp(argv[0], "set") == 0) {
		if (argn != 6) {
			svprintf(server,
					"%s usage: align set [offset_ra] [offset_dec] [pa_alt] [pa_az] [cone]\r\n",
					cmd);
			return ERR_WRONG_NUM_PARAM;
		}
		char *tp;
		EqCalibration calib;
		calib.offset.ra_off = strtod(argv[1], &tp);
		if (tp == argv[1]) {
			return ERR_PARAM_OUT_OF_RANGE;
		}
		calib.offset.dec_off = strtod(argv[2], &tp);
		if (tp == argv[2]) {
			return ERR_PARAM_OUT_OF_RANGE;
		}
		calib.pa.alt = strtod(argv[3], &tp);
		if (tp == argv[3]) {
			return ERR_PARAM_OUT_OF_RANGE;
		}
		calib.pa.azi = strtod(argv[4], &tp);
		if (tp == argv[4]) {
			return ERR_PARAM_OUT_OF_RANGE;
		}
		calib.cone = strtod(argv[5], &tp);
		if (tp == argv[5]) {
			return ERR_PARAM_OUT_OF_RANGE;
		}

		server->getEqMount()->setCalibration(calib);
	} else if (strcmp(argv[0], "force") == 0) {
		if (argn != 1) {
			return ERR_WRONG_NUM_PARAM;
		}
		server->getEqMount()->forceAlignment();
	} else if (strcmp(argv[0], "update") == 0) {
		if (argn != 1) {
			return ERR_WRONG_NUM_PARAM;
		}
		server->getEqMount()->recalibrate();
	} else {
		return ERR_PARAM_OUT_OF_RANGE;
	}
	return 0;
}

static int eqmount_nudge(EqMountServer *server, const char *cmd, int argn,
		char *argv[]) {
	if (argn != 1 && argn != 2) {
		return ERR_WRONG_NUM_PARAM;
	}
	int dir = 0;
	for (int i = 0; i < argn; i++) {
		if (strcmp(argv[i], "south") == 0)
			dir |= NUDGE_SOUTH;
		else if (strcmp(argv[i], "north") == 0)
			dir |= NUDGE_NORTH;
		else if (strcmp(argv[i], "east") == 0)
			dir |= NUDGE_EAST;
		else if (strcmp(argv[i], "west") == 0)
			dir |= NUDGE_WEST;
		else if (strcmp(argv[i], "stop") == 0) {
			// STOP nudging
			dir = NUDGE_NONE;
			break;
		} else {
			return ERR_PARAM_OUT_OF_RANGE;
		}
	}

	osStatus s;
	if ((s = server->getEqMount()->startNudge((nudgedir_t) dir)) != osOK)
		return s;

	return 0;
}

static int eqmount_track(EqMountServer *server, const char *cmd, int argn,
		char *argv[]) {
	if (argn != 0) {
		return ERR_WRONG_NUM_PARAM;
	}
	osStatus s;
	if ((s = server->getEqMount()->startTracking()) != osOK)
		return s;
	return 0;
}

static int eqmount_read(EqMountServer *server, const char *cmd, int argn,
		char *argv[]) {

	if (argn == 0) {
		EquatorialCoordinates eq =
				server->getEqMount()->getEquatorialCoordinates();
		svprintf(server, "%s %.8f %.8f\r\n", cmd, eq.ra, eq.dec);
	} else if (argn == 1) {
		if (strcmp(argv[0], "eq") == 0) {
			EquatorialCoordinates eq =
					server->getEqMount()->getEquatorialCoordinates();
			svprintf(server, "%s %.8f %.8f\r\n", cmd, eq.ra, eq.dec);
		} else if (strcmp(argv[0], "mount") == 0) {
			MountCoordinates mc = server->getEqMount()->getMountCoordinates();
			svprintf(server, "%s %.8f %.8f %c\r\n", cmd, mc.ra_delta,
					mc.dec_delta, (mc.side == PIER_SIDE_WEST) ? 'W' : 'E');
		} else {
			return ERR_PARAM_OUT_OF_RANGE;
		}
	} else {
		return ERR_WRONG_NUM_PARAM;
	}

	return 0;
}

static int eqmount_state(EqMountServer *server, const char *cmd, int argn,
		char *argv[]) {

	if (argn == 0) {
		const char *s;
		switch (server->getEqMount()->getStatus()) {
		case MOUNT_STOPPED:
			s = "stopped";
			break;
		case MOUNT_SLEWING:
			s = "slewing";
			break;
		case MOUNT_TRACKING:
			s = "tracking";
			break;
		case MOUNT_TRACKING_GUIDING:
			s = "tracking_guiding";
			break;
		case MOUNT_NUDGING:
			s = "nudging";
			break;
		case MOUNT_NUDGING_TRACKING:
			s = "nudging_tracking";
			break;
		default:
			s = "undefined";
		}
		svprintf(server, "%s %s\r\n", cmd, s);
	} else {
		return ERR_WRONG_NUM_PARAM;
	}

	return 0;
}

static int eqmount_help(EqMountServer *server, const char *cmd, int argn,
		char *argv[]) {
	svprintf(server, "%s Available commands: \r\n", cmd);
	for (int i = 0; i < MAX_COMMAND; i++) {
		if (commandlist[i].fptr == NULL)
			break;
		svprintf(server, "%s - %s : %s\r\n", cmd, commandlist[i].cmd,
				commandlist[i].desc);
	}
	return 0;
}

static int eqmount_guide(EqMountServer *server, const char *cmd, int argn,
		char *argv[]) {

	if (argn != 2) {
		svprintf(server,
				"%s Usage: guide {north|west|south|east} milliseconds\r\n",
				cmd);
		return ERR_WRONG_NUM_PARAM;
	}

	char *tp;
	int ms = strtod(argv[1], &tp);
	if (tp == argv[1] || ms < 1 || ms > MBED_CONF_PUSHTOGO_MAX_GUIDE_TIME) {
		return ERR_PARAM_OUT_OF_RANGE;
	}

	if (strcmp("north", argv[0]) == 0) {
		return server->getEqMount()->guide(GUIDE_NORTH, ms);
	} else if (strcmp("south", argv[0]) == 0) {
		return server->getEqMount()->guide(GUIDE_SOUTH, ms);
	} else if (strcmp("west", argv[0]) == 0) {
		return server->getEqMount()->guide(GUIDE_WEST, ms);
	} else if (strcmp("east", argv[0]) == 0) {
		return server->getEqMount()->guide(GUIDE_EAST, ms);
	} else {
		return ERR_PARAM_OUT_OF_RANGE;
	}

	return 0;
}

static int eqmount_time(EqMountServer *server, const char *cmd, int argn,
		char *argv[]) {
	char buf[32];
//Get time
	time_t t = server->getEqMount()->getClock().getTime();

	if (argn >= 2) {
		return ERR_WRONG_NUM_PARAM;
	} else if (argn == 1) {
		if (strcmp(argv[0], "stamp") == 0) { // Read timestamp
			// Print timestamp value
			svprintf(server, "%s %d\r\n", cmd, (uint32_t) t);
			return 0;
		} else if (strcmp(argv[0], "sidereal") == 0) { // Read sidereal time (-180~180)
			// Print sidereal time at current location
			// 0.0 is sidereal midnight, 180/-180 is sidereal noon
			double st = getLocalSiderealTime(t,
					server->getEqMount()->getLocation());
//			int hh = ((int) floor(st / 15) + 24) % 24;
//			int mm = (int) floor((st + 360.0 - hh * 15) * 4) % 60;
//			int ss = (int) floor((st + 360.0 - hh * 15 - mm * 0.25) * 240) % 60;
			svprintf(server, "%s %f\r\n", cmd, st);
//			stprintf(server, "%d:%d:%d LST\r\n", hh, mm, ss);
			return 0;
		} else if (strcmp(argv[0], "local") == 0) { // Read local time (at longitude)
			t += (int) (remainder(server->getEqMount()->getLocation().lon, 360)
					* 240);

#if !( defined(__ARMCC_VERSION) || defined(__CC_ARM) )
			ctime_r(&t, buf);
#else
			core_util_critical_section_enter();
			char *ibuf = ctime(&t);
			strncpy(buf, ibuf, sizeof(buf));
			core_util_critical_section_exit();
#endif

			svprintf(server, "%s %s\r\n", cmd, buf);
			return 0;
		} else if (strcmp(argv[0], "zone") == 0) { // Read time zone
			t += (int) (TelescopeConfiguration::getInt("timezone") * 3600);

#if !( defined(__ARMCC_VERSION) || defined(__CC_ARM) )
			ctime_r(&t, buf);
#else
			core_util_critical_section_enter();
			char *ibuf = ctime(&t);
			strncpy(buf, ibuf, sizeof(buf));
			core_util_critical_section_exit();
#endif

			svprintf(server, "%s %s\r\n", cmd, buf);
			return 0;
		} else if (strcmp(argv[0], "hr") == 0) { // Read high resolution timestamp
			double hr =
					server->getEqMount()->getClock().getTimeHighResolution();
			svprintf(server, "%s %d.%03d\r\n", cmd, int(hr),
					int((hr - int(hr)) * 1000));
			return 0;
		}

	}

// Print of formatted string of current time

#if !( defined(__ARMCC_VERSION) || defined(__CC_ARM) )
	ctime_r(&t, buf);
#else
	core_util_critical_section_enter();
	char *ibuf = ctime(&t);
	strncpy(buf, ibuf, sizeof(buf));
	core_util_critical_section_exit();
#endif

	svprintf(server, "%s %s\r\n", cmd, buf);

	return 0;
}

static int eqmount_settime(EqMountServer *server, const char *cmd, int argn,
		char *argv[]) {
	if (argn == 1) {
		//Use the first argument as UTC timestamp
		time_t t = strtol(argv[0], NULL, 10);
		server->getEqMount()->getClock().setTime(t);
	} else if (argn == 6) {
		int year = strtol(argv[0], NULL, 10);
		int month = strtol(argv[1], NULL, 10);
		int day = strtol(argv[2], NULL, 10);
		int hour = strtol(argv[3], NULL, 10);
		int min = strtol(argv[4], NULL, 10);
		int sec = strtol(argv[5], NULL, 10);
		struct tm ts;
		ts.tm_sec = sec;
		ts.tm_min = min;
		ts.tm_hour = hour;
		ts.tm_mday = day;
		ts.tm_mon = month - 1;
		ts.tm_year = year - 1900;
		ts.tm_isdst = 0;

		time_t t = mktime(&ts);
		if (t == -1) {
			// Parameter out of range
			return 2;
		}

		server->getEqMount()->getClock().setTime(t);
	} else {
		svprintf(server,
				"%s usage: settime <timestamp>, or, settime <year> <month> <day> <hour> <minute> <second> (UTC time should be used)\r\n",
				cmd);
		return 1;
	}

	return 0;
}

void EqMountServer::addCommand(const ServerCommand &cmd) {
	int i = 0;
	while (i < MAX_COMMAND && commandlist[i].fptr != NULL)
		i++;
	if (i >= MAX_COMMAND - 1) {
		error("EqMountServer: max command reached.\r\n");
		return;
	}

	commandlist[i] = cmd;
	commandlist[++i] = ServerCommand("", "", NULL);
}

ServerCommand commandlist[MAX_COMMAND] = { /// List of all commands
		ServerCommand("stop", "Stop mount motion", eqmount_stop), 	/// Stop
		ServerCommand("estop", "Emergency stop", eqmount_estop), /// Emergency Stop
		ServerCommand("read", "Read current RA/DEC position", eqmount_read), /// Read Position
		ServerCommand("time", "Get and set system time", eqmount_time), /// System time
		ServerCommand("status", "Get the mount state", eqmount_state), /// System state
		ServerCommand("help", "Print this help menu", eqmount_help), /// Help menu
		ServerCommand("speed", "Set slew and tracking speed", eqmount_speed), /// Set speed
		ServerCommand("align", "Star alignment", eqmount_align), /// Alignment
		/// Above are allowed commands when another command is running

				ServerCommand("goto",
						"Perform go to operation to specified ra, dec coordinates",
						eqmount_goto), 		/// Go to
				ServerCommand("nudge", "Perform nudging on specified direction",
						eqmount_nudge), 		/// Nudge
				ServerCommand("track", "Start tracking in specified direction",
						eqmount_track), 		/// Track
				ServerCommand("guide", "Guide on specified direction",
						eqmount_guide), /// Guide
				ServerCommand("settime", "Set system time", eqmount_settime), /// System time
				ServerCommand("", "", NULL) };


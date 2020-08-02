/*
 * Logger.h
 *
 *  Created on: Aug 1, 2020
 *      Author: caoyu
 */

#ifndef _LOGGER_H_
#define _LOGGER_H_

#include "mbed.h"
#include "pushtogo.h"
#include "FATFileSystem.h"
#include <stdarg.h>

class Logger {
public:

	/**
	 * Initialize the Logger to log to specific filesystem on bd
	 */
	static bool init(BlockDevice *bd){
		return getInstance()._init(bd);
	}

	/**
	 * Should be called when the media (e.g. SD card) is removed, so that further logs will be sent to STDOUT instead of saved
	 */
	static void deinit(){
		getInstance()._deinit();
	}

	static void format(){
		getInstance()._format();
	}

	/**
	 * Logs a string. Data and time info are added automatically. String doesn't need to have newline after them.
	 * @warning Cannot be called from ISR
	 */
	static void log(const char * fmt, ...){
		va_list args;
		va_start(args, fmt);
		getInstance()._vlog(fmt, args, false);
		va_end(args);
	}

	static void logError(const char * fmt, ...){
		va_list args;
		va_start(args, fmt);
		getInstance()._vlog(fmt, args, true);
		va_end(args);
	}

private:
	static Logger &getInstance(){
		static Logger inst;
		return inst;
	}
	Logger();
	~Logger();

	bool _init(BlockDevice *bd);
	void _deinit();
	bool _format();
	void _vlog(const char *fmt, va_list args, bool error);

	bool is_initialized;
	BlockDevice *bd;
	FATFileSystem *fs;
	Mutex mutex;

};

#endif /* _LOGGER_H_ */

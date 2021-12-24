/*
 * Logger.cpp
 *
 *  Created on: Aug 1, 2020
 *      Author: caoyu
 */

#include <Logger.h>

Logger::Logger():is_initialized(false), bd(NULL), fs(NULL) {
}

Logger::~Logger() {
	if (fs) {
		delete fs;
	}
}

bool Logger::_init(BlockDevice *bd) {
	if (fs)
		delete fs;
	this->bd = bd;
	fs = new FATFileSystem("", NULL);
	if (bd->init() == 0 && fs->mount(bd) == 0){
		is_initialized = true;
		return true;
	}
	else {
		delete fs;
		fs = NULL;
		return false;
	}
}

void Logger::_deinit() {
	if (fs) delete fs;
	fs = NULL;

	if (bd)
		bd->deinit();
	bd = NULL;

	is_initialized = false;
}

bool Logger::_format() {
	if (bd) {
		if (fs){
			delete fs;
			fs = NULL;
		}
		FATFileSystem::format(bd, 0);

		fs = new FATFileSystem("", NULL);
		fs->mount(bd);
		return true;
	}
	else{
		return false;
	}
}


static struct tm *getLocalTime(time_t timestamp, struct tm *tts) {
	if (!timestamp)
		timestamp = time(NULL);
	time_t ts = timestamp + TelescopeConfiguration::getInt("timezone") * 3600;
#if !( defined(__ARMCC_VERSION) || defined(__CC_ARM) )
	gmtime_r(&ts, tts);
#else
	core_util_critical_section_enter();
	memcpy(tts, gmtime(&ts), sizeof(struct tm));
	core_util_critical_section_exit();
#endif
	return tts;
}

static void get_log_filename(char *buf, int len, struct tm *tt) {
	snprintf(buf, len, "ptg_%4d_%02d_%02d.log", tt->tm_year+1900, tt->tm_mon+1, tt->tm_mday);
}

static char *trim(char *s)
{
    char* back = s + strlen(s);
    while(back>=s && isspace(*back))
    	back--;
    *(back+1) = '\0';
    return s;
}

static char _buf[128];

void Logger::_vlog(const char *fmt, va_list args, bool error) {

	mutex.lock();
	File *fp;
	int len;

	time_t tnow = RealTimeClock::to_time_t(RealTimeClock::now());
	struct tm tt;
	getLocalTime(tnow, &tt);
	get_log_filename(_buf, sizeof(_buf), &tt);


	if (fs && is_initialized){
		fp = new File();
		if (fp->open(fs, _buf, O_WRONLY | O_APPEND) != 0) {
			delete fp;
			fp = NULL;
		}
	} else {
		fp = NULL;
	}

	len=snprintf(_buf, sizeof(_buf),"[%02d:%02d:%02d] ", tt.tm_hour, tt.tm_min, tt.tm_sec);

	len += vsnprintf(_buf+len, sizeof(_buf)-len, fmt, args);

	_buf[sizeof(_buf)-1] = '\0'; // Make sure termination

	trim(_buf); // Remove tailing white spaces

	if (fp) {
		fp->write(_buf, len);
		fp->write("\n", 1);
		fp->sync();
		fp->close();
		delete fp;
		bd->sync();
	}
	else {
		// Print to screen
		puts(_buf);
	}
	mutex.unlock();
}

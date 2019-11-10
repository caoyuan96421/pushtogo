/*
 * EqMountServer.h
 *
 *  Created on: 2018��3��1��
 *      Author: caoyuan9642
 */

#ifndef _EQMOUNTSERVER_H_
#define _EQMOUNTSERVER_H_

#include "pushtogo.h"
#include "MountServer.h"

class EquatorialMount;
class EqMountServer;

/** Command structure for the mount server
 */
struct ServerCommand {
	const char *cmd; /// Name of the command
	const char *desc; /// Description of the command
	int (*fptr)(EqMountServer*, const char*, int, char**); /// Function pointer to the command
	ServerCommand(const char *n = "", const char *d = "",
			int (*fp)(EqMountServer*, const char*, int, char**) = NULL) :
			cmd(n), desc(d), fptr(fp) {
	}
};

#define MAX_COMMAND 128

#define ERR_WRONG_NUM_PARAM 1
#define ERR_PARAM_OUT_OF_RANGE 2

/** EqMount server class. Receives commands from a stream and execute on the binded EqMount.
 */
class EqMountServer: public MountServer {
protected:

	EquatorialMount *eq_mount; /// EqMount to be binded
	FileHandle &stream; /// Input stream
	Thread thread;
	bool echo; /// Echo
	Mutex stream_mutex;

	void task_thread(); /// Main task entrance
	void command_execute(ServerCommand&, int argn, char *argv[], char *buffer); /// To execute a server command
	void vfprint(const char *, va_list);
	friend void svprintf(EqMountServer *, const char *fmt, ...);

public:
	/** Creates a server with input stream and optionally echoing to the stream.
	 */
	EqMountServer(FileHandle &stream, bool echo = false);
	virtual ~EqMountServer();

	/** Bind to a EqMount. Commands will be ignored if no mount is binded.
	 */
	void bind(EquatorialMount &eq) {
		eq_mount = &eq;
	}

	/** @return binded EqMount
	 */
	EquatorialMount* getEqMount() const {
		return eq_mount;
	}

	/** @return associated stream
	 */
	FileHandle& getStream() const {
		return stream;
	}

	/** Add a command to the available commands. Must have unique names.
	 */
	static void addCommand(const ServerCommand &cmd);
};

/**
 * Print to stream
 */
void svprintf(EqMountServer *, const char *fmt, ...);

#endif /* _EQMOUNTSERVER_H_ */


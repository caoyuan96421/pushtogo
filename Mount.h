#ifndef _MOUNT_H_
#define _MOUNT_H_

typedef enum
{
	MOUNT_STOPPED = 0,
	MOUNT_SLEWING = 1,
	MOUNT_TRACKING = 2,
	MOUNT_NUDGING = 4,
	MOUNT_NUDGING_TRACKING = MOUNT_TRACKING | MOUNT_NUDGING,
	
	// Modifiers, will only appear in getStatus() but not in status itself
	MOUNT_GUIDING = 8,
	MOUNT_TRACKING_GUIDING = MOUNT_TRACKING | MOUNT_GUIDING
} mountstatus_t;

class Mount
{
protected:
	mountstatus_t status;

public:
	Mount() :
			status(MOUNT_STOPPED)
	{
	}

	virtual ~Mount()
	{
	}

	virtual mountstatus_t getStatus()
	{
		return status;
	}
};

#endif /*_MOUNT_H_*/


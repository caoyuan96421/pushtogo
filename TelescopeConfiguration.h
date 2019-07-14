/*
 * TelescopeConfiguration.h
 *
 *  Created on: 2018Äê3ÔÂ1ÈÕ
 *      Author: caoyuan9642
 */

#ifndef PUSHTOGO_TELESCOPECONFIGURATION_H_
#define PUSHTOGO_TELESCOPECONFIGURATION_H_

#include <stdio.h>
#include <string.h>
#include "CelestialMath.h"
#include "EqMountServer.h"

typedef enum
{
	DATATYPE_INT, DATATYPE_DOUBLE, DATATYPE_STRING, DATATYPE_BOOL
} DataType;

typedef union
{
	int idata;
	double ddata;bool bdata;
	char strdata[32];
} DataValue;

struct ConfigItem
{
	char *config;
	char *name;
	const char *help;
	DataType type;
	DataValue value;
	DataValue min;
	DataValue max;bool extra;
};

class EqMountServer;

/**
 * Global telescope configuration storage class
 */
class TelescopeConfiguration
{
public:
	static void readFromFile(FILE *fp);
	static void writeToFile(FILE *fp);

	static int getInt(const char *name)
	{
		return getIntFromConfig(getInstance().getConfigItemCheck(name));
	}
	static double getDouble(const char *name)
	{
		return getDoubleFromConfig(getInstance().getConfigItemCheck(name));
	}
	static bool getBool(const char *name)
	{
		return getBoolFromConfig(getInstance().getConfigItemCheck(name));
	}
	static char *getString(const char *name, char buf[], int len)
	{
		return getStringFromConfig(getInstance().getConfigItemCheck(name), buf,
				len);
	}

private:
	TelescopeConfiguration();
	~TelescopeConfiguration();

	static TelescopeConfiguration instance;

	struct ConfigNode
	{
		ConfigItem *config;
		const ConfigItem *default_config;
		ConfigNode *next;
	}*head;

	static TelescopeConfiguration &getInstance()
	{
		return instance;
	}

	ConfigItem *getConfigItem(const char *name);
	ConfigItem *getConfigItemCheck(const char *name);

	void setConfig(const char *name, char *value);

	static int getIntFromConfig(ConfigItem *);
	static double getDoubleFromConfig(ConfigItem *);
	static bool getBoolFromConfig(ConfigItem *);
	static char *getStringFromConfig(ConfigItem *, char buf[], int len);

	static bool setIntToConfig(ConfigItem *, int value);
	static bool setDoubleToConfig(ConfigItem *, double value);
	static bool setBoolToConfig(ConfigItem *, bool value);
	static bool setStringToConfig(ConfigItem *, char *value);

	static int eqmount_config(EqMountServer *server, const char *cmd, int argn,
			char *argv[]);
};

#endif /* PUSHTOGO_TELESCOPECONFIGURATION_H_ */


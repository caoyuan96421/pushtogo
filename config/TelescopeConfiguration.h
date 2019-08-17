/*
 * TelescopeConfiguration.h
 *
 *  Created on: 2018Äê3ÔÂ1ÈÕ
 *      Author: caoyuan9642
 */

#ifndef _TELESCOPECONFIGURATION_H_
#define _TELESCOPECONFIGURATION_H_

#include <stdio.h>
#include <string.h>
#include "CelestialMath.h"
#include "EqMountServer.h"

#ifdef NVSTORE_ENABLED
#include "nvstore.h"
#endif

typedef enum {
	DATATYPE_INT, DATATYPE_DOUBLE, DATATYPE_STRING, DATATYPE_BOOL
} DataType;

typedef union {
	int idata;
	double ddata;
	bool bdata;
	char strdata[32];
} DataValue;

struct ConfigItem {
	char *config;
	char *name;
	const char *help;
	DataType type;
	DataValue value;
	DataValue min;
	DataValue max;
	bool extra;
};

class EqMountServer;

/**
 * Global telescope configuration storage class
 */
class TelescopeConfiguration {
public:
	static void readFromFile(FILE *fp);
	static void writeToFile(FILE *fp);

#ifdef NVSTORE_ENABLED
	static int saveConfig_NV();
	static int readConfig_NV();
#endif

	static int getInt(const char *name) {
		return getIntFromConfig(getInstance().getConfigItemCheck(name));
	}
	static double getDouble(const char *name) {
		return getDoubleFromConfig(getInstance().getConfigItemCheck(name));
	}
	static bool getBool(const char *name) {
		return getBoolFromConfig(getInstance().getConfigItemCheck(name));
	}
	static char* getString(const char *name, char buf[], int len) {
		return getStringFromConfig(getInstance().getConfigItemCheck(name), buf,
				len);
	}
	static void setInt(const char *name, int value) {
		setIntToConfig(getInstance().getConfigItemCheck(name), value);
	}
	static void setDouble(const char *name, double value) {
		setDoubleToConfig(getInstance().getConfigItemCheck(name), value);
	}
	static void setBool(const char *name, bool value) {
		setBoolToConfig(getInstance().getConfigItemCheck(name), value);
	}
	static void setString(const char *name, char *value) {
		setStringToConfig(getInstance().getConfigItemCheck(name), value);
	}

	static bool isConfigExist(const char *name){
		return getInstance().getConfigItem(name) == NULL;
	}

private:
	TelescopeConfiguration();
	~TelescopeConfiguration();

	struct ConfigNode {
		int key;
		ConfigItem *config;
		const ConfigItem *default_config;
		ConfigNode *next;
	} *head;

	static TelescopeConfiguration& getInstance() {
		static TelescopeConfiguration instance;
		return instance;
	}

	ConfigItem* getConfigItem(const char *name);
	ConfigItem* getConfigItemCheck(const char *name);

	void setConfigByName(const char *name, const char *value);

	static int getIntFromConfig(ConfigItem*);
	static double getDoubleFromConfig(ConfigItem*);
	static bool getBoolFromConfig(ConfigItem*);
	static char* getStringFromConfig(ConfigItem*, char buf[], int len);

	static bool setIntToConfig(ConfigItem*, int value);
	static bool setDoubleToConfig(ConfigItem*, double value);
	static bool setBoolToConfig(ConfigItem*, bool value);
	static bool setStringToConfig(ConfigItem*, char *value);

	static int eqmount_config(EqMountServer *server, const char *cmd, int argn,
			char *argv[]);
};

#endif /* PUSHTOGO_TELESCOPECONFIGURATION_H_ */


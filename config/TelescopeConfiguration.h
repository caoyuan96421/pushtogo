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
#include "pushtogo.h"

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

struct ConfigNode {
	int key;
	ConfigItem *config;
	const ConfigItem *default_config;
	ConfigNode *next;
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

	static int getInt(const char *name);
	static double getDouble(const char *name);
	static bool getBool(const char *name);
	static char* getString(const char *name, char buf[], int len);

	static int getInt(const ConfigItem *config);
	static double getDouble(const ConfigItem *config);
	static bool getBool(const ConfigItem *config);
	static char* getString(const ConfigItem *config, char buf[], int len);

	static bool getIntLimit(const char *name, int &min, int &max);
	static bool getDoubleLimit(const char *name, double &min, double &max);

	static void setInt(const char *name, int value);
	static void setDouble(const char *name, double value);
	static void setBool(const char *name, bool value);
	static void setString(const char *name, const char *value);
	static void setConfigAutoType(const char *name, const char *value);

	static void setInt(ConfigItem *config, int value);
	static void setDouble(ConfigItem *config, double value);
	static void setBool(ConfigItem *config, bool value);
	static void setString(ConfigItem *config, const char *value);
	static void setConfigAutoType(ConfigItem *config, const char *value);

	static bool setIntLimit(const char *name, int min, int max);
	static bool setDoubleLimit(const char *name, double min, double max);

	static bool isConfigExist(const char *name);
//		return getInstance().getConfigItem(name) == NULL;
	static void removeConfig(const char *name);

	static ConfigNode *&getHead() {
		static TelescopeConfiguration instance;
		return instance.head;
	}

private:
	static ConfigNode *head;

	TelescopeConfiguration();
	~TelescopeConfiguration();

	static ConfigItem* getConfigItem(const char *name, bool check = false);
	static ConfigItem* addConfigItem(const char *name, DataType config_type);
	static void deleteConfigItem(const char *name);

	static int eqmount_config(EqMountServer *server, const char *cmd, int argn,
			char *argv[]);
};

#endif /* PUSHTOGO_TELESCOPECONFIGURATION_H_ */


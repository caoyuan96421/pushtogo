/*
 * TelescopeConfiguration.cpp
 *
 *  Created on: 2018Äê3ÔÂ1ÈÕ
 *      Author: caoyuan9642
 */

#include "TelescopeConfiguration.h"
#include "mbed.h"
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <float.h>
#include "printf.h"

ConfigNode *TelescopeConfiguration::head;

static const char* typeName(DataType type) {
	switch (type) {
	case DATATYPE_INT:
		return "INT";
	case DATATYPE_DOUBLE:
		return "DOUBLE";
	case DATATYPE_BOOL:
		return "BOOL";
	case DATATYPE_STRING:
		return "STRING";
	default:
		return "UNKNOWN";
	}
}

// @formatter:off
static const ConfigItem default_config[] =
		{
			{.config = const_cast<char *>("latitude"), .name = "Your Latitude", .help =	"Latitude of observer, in degrees north of equator.",
			.type =	DATATYPE_DOUBLE, .value = { .ddata =0 }, .min = { .ddata = -90 }, .max = { .ddata = 90 } },

			{.config =	const_cast<char *>("longitude"), .name = "Your Longitude", .help =	"Longitude of observer, in degrees east of Greenwich.",
			.type =	DATATYPE_DOUBLE, .value = { .ddata = 0 }, .min = { .ddata = -180 }, .max = { .ddata = 180 } },

			{.config = const_cast<char *>("timezone"),	.name = "Your Timezone", .help = "Timezone in hours ahead of UTC time.",
			.type =	DATATYPE_INT, .value = { .idata = 0 }, .min = {.idata = -12 }, .max = { .idata = 12 } },

			{.config = const_cast<char *>("motor_steps"), .name = "Steps per Revolution",	.help =	"Motor steps/revolution.\nIf you hard-programed the microstepping, here should use total microstep resolution.",
			.type = DATATYPE_INT, .value = { .idata = 400 }, .min =	{ .idata = 1 }, .max = { .idata = 1000000 } },

			{.config = const_cast<char *>("gear_reduction"), .name = "Gear Ratio",	.help = "Gearbox reduction ratio. ",
			.type = DATATYPE_DOUBLE, .value = { .ddata = 1 }, .min = { .ddata = 0 }, .max = { .ddata = 10000 } },

			{.config = const_cast<char *>("worm_teeth"), .name = "Worm Teeth", .help =	"Number of teeth on the ring gear.",
			.type =	DATATYPE_INT, .value = { .idata = 180 }, .min =	{ .idata = 1 }, .max = { .idata = 10000 } },

			{ .config = const_cast<char *>("ra_use_encoder"), .name = "Use Encoder on RA", .help = "Use encoder to determine RA position",
			.type = DATATYPE_BOOL, .value = { .bdata = false } },

			{ .config = const_cast<char *>("dec_use_encoder"), .name = "Use Encoder on DEC", .help = "Use encoder to determine DEC position",
			.type = DATATYPE_BOOL, .value = { .bdata = false } },

			{.config = const_cast<char *>("ra_invert"), .name = "Invert RA Direction", .help =	"Invert RA driving direction?\n Save and restart to take effect",
			.type = DATATYPE_BOOL, .value = { .bdata = false } },

			{ .config = const_cast<char *>("dec_invert"), .name = "Invert DEC Direction", .help = "Invert DEC driving direction?\n Save and restart to take effect",
			.type = DATATYPE_BOOL, .value = { .bdata = false } },

			{.config = const_cast<char *>("ra_enc_invert"), .name = "Reverse RA Encoder", .help =	"Reverse counting direction of RA encoder",
			.type = DATATYPE_BOOL, .value = { .bdata = false } },

			{ .config = const_cast<char *>("dec_enc_invert"), .name = "Reverse DEC Encoder", .help = "Reverse counting direction of DEC encoder",
			.type = DATATYPE_BOOL, .value = { .bdata = false } },

			{.config = const_cast<char *>("goto_slew_speed"), .name = "Goto Slew Speed", .help = "Slewing speed used for goto, in deg/s",
			.type =	DATATYPE_DOUBLE, .value = { .ddata = 2 }, .min = { .ddata = 1 }, .max = { .ddata = 10 } },

			{ .config = const_cast<char *>("acceleration"), .name = "Acceleration",	.help = "Acceleration in deg/s^2.",
			.type =	DATATYPE_DOUBLE, .value = { .ddata = 5 }, .min = { .ddata = 0.01 }, .max = { .ddata = 1000 } },

			{ .config = const_cast<char *>("max_speed"), .name = "Max Slewing Speed", .help = "Max slewing speed. Reduce this value if losing steps.",
			.type = DATATYPE_DOUBLE, .value = { .ddata = 4 }, .min = { .ddata = 1 }, .max = { .ddata = 100 } },

			{ .config = const_cast<char *>("pec_granularity"), .name = "PEC Granularity", .help = "Number of PEC slots per revolution of the worm",
			.type = DATATYPE_INT, .value = { .idata = 512 }, .min = {.idata = 32 }, .max = { .idata = 16384} },

			{ .config = const_cast<char *>("serial_baud"), .name = "Baud Rate", .help = "Baud rate of communication",
			.type = DATATYPE_INT, .value = { .idata = 115200 }, .min = {.idata = 9600 }, .max = { .idata = 2000000} },

			{ .config = const_cast<char *>("ra_limit"), .name = "RA Limit", .help = "Limit of RA axis below horizontal position in deg",
			.type = DATATYPE_DOUBLE, .value = { .ddata = 20 }, .min = {.ddata = -90 }, .max = { .ddata = 90} },

			{ .config = const_cast<char *>("side_pref"), .name = "Side of Pier (W=-1, E=1)", .help = "Preference of side-of-pier near meridian",
			.type = DATATYPE_INT, .value = { .idata = 0 }, .min = {.idata = -1 }, .max = { .idata = 1} },

			{ .config = const_cast<char *>("side_thd"), .name = "Side of Pier Threshold", .help = "Threshold of side-of-pier choice. Outside of this value from meridian, side-of-pier is fixed.",
			.type = DATATYPE_DOUBLE, .value = { .ddata = 10 }, .min = {.ddata = 0 }, .max = { .ddata = 20} },

			{.config = const_cast<char *>("") } };
// @formatter:on
int TelescopeConfiguration::eqmount_config(EqMountServer *server,
		const char *cmd, int argn, char *argv[]) {
	char buf[256];
	if (argn == 0) {
		// Print all config names
		ConfigNode *p = getHead();
		svprintf(server, "%s", cmd);
		for (; p; p = p->next) {
			svprintf(server, " %s", p->config->config);
			// Get string representing the value
//			getStringFromConfig(p->config, buf, sizeof(buf));
//			stprintf(server, "%s %s,%s,%s,%s\r\n", cmd,
//					p->config->config, p->config->name,
//					typeName(p->config->type), buf);
		}
		svprintf(server, "\r\n");
	} else {
		char *config_name = argv[0]; // Name of the config in question
		ConfigItem *config = getConfigItem(config_name);
		if (!config) {
			svprintf(server, "%s Error: config %s not found\r\n", cmd,
					config_name);
			return ERR_PARAM_OUT_OF_RANGE;
		}
		if (argn == 1) {
			// Print value of the config
			getString(config, buf, sizeof(buf));
			svprintf(server, "%s %s\r\n", cmd, buf);
		} else if (argn == 2) {
			if (strcmp(argv[1], "default") == 0) {
				// Print default value of the config
				// TODO
			} else if (strcmp(argv[1], "name") == 0) {
				// Print name
				svprintf(server, "%s %s\r\n", cmd, config->name);
			} else if (strcmp(argv[1], "help") == 0) {
				// Print help
				svprintf(server, "%s %s\r\n", cmd, config->help);
			} else if (strcmp(argv[1], "type") == 0) {
				// Print type
				svprintf(server, "%s %s\r\n", cmd, typeName(config->type));
			} else if (strcmp(argv[1], "info") == 0) {
				// Print type, value, name and help
				getString(config, buf, sizeof(buf));
				svprintf(server, "%s %s,%s,%s,%s\r\n", cmd,
						typeName(config->type), buf, config->name,
						config->help);
			} else if (strcmp(argv[1], "limit") == 0) {
				// Print min/max
				if (config->type != DATATYPE_STRING
						&& config->type != DATATYPE_BOOL && !config->extra) {
					switch (config->type) {
					case DATATYPE_INT:
						svprintf(server, "%s %d %d\r\n", cmd, config->min.idata,
								config->max.idata);
						break;
					case DATATYPE_DOUBLE:
						svprintf(server, "%s %.8f %.8f\r\n", cmd,
								config->min.ddata, config->max.ddata);
						break;
					default:
						break;
					}
				} else {
					svprintf(server, "%s limit not supported for %s.\r\n", cmd,
							config->config);
				}
			} else {
				// Set value
				char *value = argv[1];
				setConfigAutoType(config, value);
			}
		}
	}
	return 0;
}

TelescopeConfiguration::TelescopeConfiguration() {
	ConfigNode *q = NULL, *r;
	int k = 0;
	for (const ConfigItem *p = default_config; *(p->config) != '\0'; p++) {
		r = new ConfigNode;
		if (!q)
			head = r;
		else
			q->next = r;
		r->next = NULL;
		r->config = new ConfigItem(*p);
		r->default_config = p;
		r->key = k++;
		q = r;
	}

#ifdef NVSTORE_ENABLED
	NVStore::get_instance().set_max_keys(128);
#endif

	EqMountServer::addCommand(
			ServerCommand("config", "Configuration subsystem",
					TelescopeConfiguration::eqmount_config));
}

int TelescopeConfiguration::getInt(const ConfigItem *config) {
	if (config->type != DATATYPE_INT) {
		error("Data type mismatch: wanted %s, actual %s",
				typeName(DATATYPE_INT), typeName(config->type));
	}
	return config->value.idata;
}

double TelescopeConfiguration::getDouble(const ConfigItem *config) {
	if (config->type != DATATYPE_DOUBLE && config->type != DATATYPE_INT) {
		error("Data type mismatch: wanted %s, actual %s",
				typeName(DATATYPE_DOUBLE), typeName(config->type));
	}
	return (config->type == DATATYPE_DOUBLE) ?
			config->value.ddata : config->value.idata;
}

bool TelescopeConfiguration::getBool(const ConfigItem *config) {
	if (config->type != DATATYPE_BOOL) {
		error("Data type mismatch: wanted %s, actual %s",
				typeName(DATATYPE_BOOL), typeName(config->type));
	}
	return config->value.bdata;
}

char* TelescopeConfiguration::getString(const ConfigItem *config, char buf[],
		int len) {
	if (!buf || len <= 0)
		return NULL;
	switch (config->type) {
	case DATATYPE_DOUBLE:
		snprintf(buf, len, "%.8f", config->value.ddata);
		break;
	case DATATYPE_INT:
		snprintf(buf, len, "%d", config->value.idata);
		break;
	case DATATYPE_BOOL:
		snprintf(buf, len, "%s", config->value.bdata ? "true" : "false");
		break;
	case DATATYPE_STRING:
		strncpy(buf, config->value.strdata, len);
		break;
	}
	buf[len - 1] = 0; // Make sure it's terminated
	return buf;
}

void TelescopeConfiguration::setInt(ConfigItem *config, int value) {
	if (config == NULL) {
		debug_ptg(DEFAULT_DEBUG, "TC: Null config");
		return;
	}
	if (config->type != DATATYPE_DOUBLE && config->type != DATATYPE_INT) {
		error("Data type mismatch %s: wanted %s, actual %s", config->config,
				typeName(DATATYPE_INT), typeName(config->type));
	}
	if (config->type == DATATYPE_INT) {
		if (value > config->max.idata)
			value = config->max.idata;
		else if (value < config->min.idata)
			value = config->min.idata;
		config->value.idata = value;
	} else {
		if (value > config->max.ddata)
			value = floor(config->max.ddata);
		else if (value < config->min.ddata)
			value = ceil(config->min.ddata);
		config->value.ddata = value;
	}
}

void TelescopeConfiguration::setDouble(ConfigItem *config, double value) {
	if (config == NULL) {
		debug_ptg(DEFAULT_DEBUG, "TC: Null config");
		return;
	}
	if (config->type != DATATYPE_DOUBLE) {
		error("Data type mismatch %s: wanted %s, actual %s", config->config,
				typeName(DATATYPE_INT), typeName(config->type));
		return;
	}

	if (value > config->max.ddata)
		value = config->max.ddata;
	else if (value < config->min.ddata)
		value = config->min.ddata;
	config->value.ddata = value;
}

void TelescopeConfiguration::setBool(ConfigItem *config, bool value) {
	if (config == NULL) {
		debug_ptg(DEFAULT_DEBUG, "TC: Null config");
		return;
	}
	if (config->type != DATATYPE_BOOL) {
		error("Data type mismatch %s: wanted %s, actual %s", config->config,
				typeName(DATATYPE_INT), typeName(config->type));
		return;
	}
	config->value.bdata = value;
}

void TelescopeConfiguration::setString(ConfigItem *config, const char *value) {
	if (config == NULL) {
		debug_ptg(DEFAULT_DEBUG, "TC: Null config");
		return;
	}
	if (config->type != DATATYPE_STRING) {
		error("Data type mismatch %s: wanted %s, actual %s", config->config,
				typeName(DATATYPE_INT), typeName(config->type));
		return;
	}
	strncpy(config->value.strdata, value, sizeof(config->value.strdata));
}

ConfigItem* TelescopeConfiguration::addConfigItem(const char *name,
		DataType type) {
	ConfigItem *config;
	// Create new node
	config = new ConfigItem;
	config->config = new char[strlen(name) + 1];
	strcpy(config->config, name);
	config->help = "";
	config->name = config->config;
	config->extra = true;
	config->type = type;
	if (config->type == DATATYPE_INT) {
		config->max.idata = 0x7FFFFFFF; // 2^31-1
		config->min.idata = 0x80000000; // -2^31
	} else if (config->type == DATATYPE_DOUBLE) {
		config->max.ddata = DBL_MAX;
		config->min.ddata = -DBL_MAX;
	}
	ConfigNode *n = new ConfigNode, *m = getHead();
	while (m->next)
		m = m->next;
	n->config = config;
	n->default_config = NULL;
	m->next = n;
	n->key = m->key + 1;
	n->next = NULL;

	return config;
}

ConfigItem* TelescopeConfiguration::getConfigItem(const char *name,
		bool check) {
	ConfigNode *p;
	for (p = getHead(); p && (strcmp(p->config->config, name) != 0); p =
			p->next)
		;
	if (!p) {
		if (check)
			error("Config not found: %s", name);
		return NULL;
	} else
		return p->config;
}

void TelescopeConfiguration::deleteConfigItem(const char *name) {
	ConfigNode *p;
	for (p = getHead(); p && (strcmp(p->config->config, name) != 0); p =
			p->next)
		;
	if (p && p->default_config == NULL) { // Default configs cannot be deleted
		if (p == getHead()) {
			// Remove head, just point to the second element
			getHead() = p->next;
		} else {
			// Find previous node
			ConfigNode *q;
			for (q = getHead(); q && q->next != p; q = q->next)
				;
			// Point to next element
			q->next = p->next;
		}
		// Delete p properly
		if (p->config->config) // non-default ones have deletable q->config->config string
			delete p->config->config;
		if (p->config)
			delete p->config;
		delete p;
	}
}

TelescopeConfiguration::~TelescopeConfiguration() {
	for (ConfigNode *q = getHead(); q;) {
		ConfigNode *p = q->next;
		if (q->config->config && q->default_config) // Only non-default ones have deletable q->config->config string
			delete q->config->config;
		if (q->config)
			delete q->config;
		delete q;
		q = p;
	}
}

void TelescopeConfiguration::readFromFile(FILE *fp) {
	char line[256];

	int lineno = 0;

	while (true) {
		if (fgets(line, sizeof(line), fp) == NULL)
			break;
		char *p = line;
		lineno++;
		// Skip any white characters in the front
		while (*p && isspace(*p))
			p++;
		if (*p == '\0') {
			/*Empty line*/
			continue;
		}
		// Skip commented lines
		if (*p == '#')
			continue;
		// Find the '=' sign
		char *q = strchr(p, '=');
		if (q == NULL) {
			/*Syntax error*/
			error("TC: Syntax error in line %d\n", lineno);
			continue;
		}

		/*strip the parameter name*/
		char *r = q - 1;
		while (r >= p && isspace(*r))
			r--;

		q = q + 1;
		while (*q && isspace(*q))
			q++;
		if (*q == '\0') {
			/*Empty value, just keep the default*/
			continue;
		}

		char *s = line + strlen(line) - 1; // Last character of the string
		while (s >= q && isspace(*s))
			s--;

		char parameter[64], value[64];
		strncpy(parameter, p, r - p + 1);
		parameter[r - p + 1] = '\0';
		strncpy(value, q, s - q + 1);
		value[s - q + 1] = '\0';

		setConfigAutoType(parameter, value);
	}
}

void TelescopeConfiguration::writeToFile(FILE *fp) {
	char buf[256];
	for (ConfigNode *p = getHead(); p; p = p->next) {
		getString(p->config, buf, sizeof(buf));
		fprintf(fp, "%s = %s\n", p->config->config, buf);
	}
}

#ifdef NVSTORE_ENABLED

int TelescopeConfiguration::saveConfig_NV() {
	char buf[256];
	bool success = true;
	NVStore &nv = NVStore::get_instance();

	for (ConfigNode *p = getHead(); p; p = p->next) {
		// Format of saving: [name] [value]
		size_t len = snprintf(buf, sizeof(buf), "%s ", p->config->config);
		getString(p->config, buf + len, sizeof(buf) - len);

		if (nv.set(p->key, strlen(buf), buf) != NVSTORE_SUCCESS) {
			success = false;
		}
	}

	return success ? 0 : -1;
}

int TelescopeConfiguration::readConfig_NV() {
	char buf[256];
	NVStore &nv = NVStore::get_instance();
	unsigned int key;
	// Iterate over all saved items
	for (key = 0; key < nv.get_max_keys(); key++) {
		unsigned short actual_size;
		if (nv.get(key, sizeof(buf), buf, actual_size) != NVSTORE_SUCCESS) {
			break;
		}
		char *sp = strchr(buf, ' ');
		char *name = buf;
		char *value = sp + 1;
		*sp = '\0'; // Terminate name string
		*(buf + actual_size) = '\0'; // Terminate value string
		setConfigAutoType(name, value);
	}

	return key >= sizeof(default_config) / sizeof(ConfigItem); // Success if at least all default items are set
}

#endif

int TelescopeConfiguration::getInt(const char *name) {
	ConfigItem *config = getConfigItem(name, true);
	return getInt(config);
}

double TelescopeConfiguration::getDouble(const char *name) {
	ConfigItem *config = getConfigItem(name, true);
	return getDouble(config);
}

bool TelescopeConfiguration::getBool(const char *name) {
	ConfigItem *config = getConfigItem(name, true);
	return getBool(config);
}

char* TelescopeConfiguration::getString(const char *name, char buf[], int len) {
	ConfigItem *config = getConfigItem(name, true);
	return getString(config, buf, len);
}

void TelescopeConfiguration::setInt(const char *name, int value) {
	ConfigItem *config = getConfigItem(name);
	if (!config) {
		// Create new config
		config = addConfigItem(name, DATATYPE_INT);
		if (!config)
			return;
	}
	setInt(config, value);
}

void TelescopeConfiguration::setDouble(const char *name, double value) {
	ConfigItem *config = getConfigItem(name);
	if (!config) {
		// Create new config
		config = addConfigItem(name, DATATYPE_DOUBLE);
		if (!config)
			return;
	}
	setDouble(config, value);
}

void TelescopeConfiguration::setBool(const char *name, bool value) {
	ConfigItem *config = getConfigItem(name);
	if (!config) {
		// Create new config
		config = addConfigItem(name, DATATYPE_BOOL);
		if (!config)
			return;
	}
	setBool(config, value);
}

void TelescopeConfiguration::setString(const char *name, const char *value) {
	ConfigItem *config = getConfigItem(name);
	if (!config) {
		// Create new config
		config = addConfigItem(name, DATATYPE_STRING);
		if (!config)
			return;
	}
	setString(config, value);
}

void TelescopeConfiguration::setConfigAutoType(const char *name,
		const char *value) {
	ConfigItem *config = getConfigItem(name);
	if (!config) {
		DataType type;
		if (strcmp(value, "true") == 0 || strcmp(value, "false") == 0) {
			type = DATATYPE_BOOL;
		} else {
			char *tail;
			strtol(value, &tail, 10);
			if (*tail == '\0') { // Successfully converted to int
				type = DATATYPE_INT;
			} else {
				strtod(value, &tail);
				if (*tail == '\0') { // Successfully converted to double
					type = DATATYPE_DOUBLE;
				} else {
					type = DATATYPE_STRING;
				}
			}
		}
		config = addConfigItem(name, type);
		if (!config)
			return;
	}
	setConfigAutoType(config, value);
}

void TelescopeConfiguration::setConfigAutoType(ConfigItem *config,
		const char *value) {
	switch (config->type) {
	case DATATYPE_INT:
		setInt(config, strtol(value, NULL, 10));
		break;
	case DATATYPE_DOUBLE:
		setDouble(config, strtod(value, NULL));
		break;
	case DATATYPE_BOOL:
		setBool(config, (strcmp(value, "true") == 0));
		break;
	case DATATYPE_STRING:
		setString(config, value);
		break;
	}
}

bool TelescopeConfiguration::isConfigExist(const char *name) {
	return getConfigItem(name) != NULL;
}

bool TelescopeConfiguration::getIntLimit(const char *name, int &min, int &max) {
	ConfigItem *config = getConfigItem(name);
	if (!config || config->type != DATATYPE_INT)
		return false;
	min = config->min.idata;
	max = config->max.idata;
	return true;
}

bool TelescopeConfiguration::getDoubleLimit(const char *name, double &min,
		double &max) {
	ConfigItem *config = getConfigItem(name);
	if (!config || config->type != DATATYPE_DOUBLE)
		return false;
	min = config->min.ddata;
	max = config->max.ddata;
	return true;
}

bool TelescopeConfiguration::setIntLimit(const char *name, int min, int max) {
	ConfigItem *config = getConfigItem(name);
	if (!config || config->type != DATATYPE_INT)
		return false;
	config->min.idata = min;
	config->max.idata = max;
	return true;
}

bool TelescopeConfiguration::setDoubleLimit(const char *name, double min,
		double max) {
	ConfigItem *config = getConfigItem(name);
	if (!config || config->type != DATATYPE_DOUBLE)
		return false;
	config->min.ddata = min;
	config->max.ddata = max;
	return true;
}

void TelescopeConfiguration::removeConfig(const char *name) {
	deleteConfigItem(name);
}


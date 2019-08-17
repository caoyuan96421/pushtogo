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

#define TC_DEBUG 1

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
			{.config = const_cast<char *>("latitude"), .name = "Your latitude", .help =	"Latitude of observer, in degrees north of equator.",
			.type =	DATATYPE_DOUBLE, .value = { .ddata =0 }, .min = { .ddata = -90 }, .max = { .ddata = 90 } },

			{.config =	const_cast<char *>("longitude"), .name = "Your longitude", .help =	"Longitude of observer, in degrees east of Greenwich.",
			.type =	DATATYPE_DOUBLE, .value = { .ddata = 0 }, .min = { .ddata = -180 }, .max = { .ddata = 180 } },

			{.config = const_cast<char *>("timezone"),	.name = "Your timezone", .help = "Timezone in hours ahead of UTC time.",
			.type =	DATATYPE_INT, .value = { .idata = 0 }, .min = {.idata = -12 }, .max = { .idata = 12 } },

			{.config = const_cast<char *>("motor_steps"), .name = "Steps per Revolution",	.help =	"Motor steps/revolution.\nIf you hard-programed the microstepping, here should use total microstep resolution.",
			.type = DATATYPE_INT, .value = { .idata = 400 }, .min =	{ .idata = 1 }, .max = { .idata = 1000000 } },

			{.config = const_cast<char *>("gear_reduction"), .name = "Gear Ratio",	.help = "Gearbox reduction ratio. ",
			.type = DATATYPE_DOUBLE, .value = { .ddata = 1 }, .min = { .ddata = 0 }, .max = { .ddata = 10000 } },

			{.config = const_cast<char *>("worm_teeth"), .name = "Worm Teeth", .help =	"Number of teeth on the ring gear.",
			.type =	DATATYPE_INT, .value = { .idata = 180 }, .min =	{ .idata = 1 }, .max = { .idata = 10000 } },

			{ .config = const_cast<char *>("ra_use_encoder"), .name = "Use encoder on RA", .help = "Use encoder to determine RA position",
			.type = DATATYPE_BOOL, .value = { .bdata = false } },

			{ .config = const_cast<char *>("dec_use_encoder"), .name = "Use encoder on DEC", .help = "Use encoder to determine DEC position",
			.type = DATATYPE_BOOL, .value = { .bdata = false } },

			{.config = const_cast<char *>("ra_invert"), .name = "Invert RA direction", .help =	"Invert RA driving direction?\n Save and restart to take effect",
			.type = DATATYPE_BOOL, .value = { .bdata = false } },

			{ .config = const_cast<char *>("dec_invert"), .name = "Invert DEC direction", .help = "Invert DEC driving direction?\n Save and restart to take effect",
			.type = DATATYPE_BOOL, .value = { .bdata = false } },

			{.config = const_cast<char *>("ra_enc_invert"), .name = "Reverse RA encoder", .help =	"Reverse counting direction of RA encoder",
			.type = DATATYPE_BOOL, .value = { .bdata = false } },

			{ .config = const_cast<char *>("dec_enc_invert"), .name = "Reverse DEC encoder", .help = "Reverse counting direction of DEC encoder",
			.type = DATATYPE_BOOL, .value = { .bdata = false } },

			{.config = const_cast<char *>("goto_slew_speed"), .name = "Goto slew speed", .help = "Slewing speed used for goto, in deg/s",
			.type =	DATATYPE_DOUBLE, .value = { .ddata = 2 }, .min = { .ddata = 1 }, .max = { .ddata = 10 } },

			{ .config = const_cast<char *>("acceleration"), .name = "Acceleration",	.help = "Acceleration in deg/s^2.",
			.type =	DATATYPE_DOUBLE, .value = { .ddata = 5 }, .min = { .ddata = 0.01 }, .max = { .ddata = 1000 } },

			{ .config = const_cast<char *>("max_speed"), .name = "Max slewing speed", .help = "Max slewing speed. Reduce this value if losing steps.",
			.type = DATATYPE_DOUBLE, .value = { .ddata = 4 }, .min = { .ddata = 1 }, .max = { .ddata = 100 } },

			{ .config = const_cast<char *>("pec_granularity"), .name = "PEC Granularity", .help = "Number of PEC slots per revolution of the worm",
			.type = DATATYPE_INT, .value = { .idata = 512 }, .min = {.idata = 32 }, .max = { .idata = 16384} },

			{ .config = const_cast<char *>("serial_baud"), .name = "Baud rate", .help = "Baud rate of communication",
			.type = DATATYPE_INT, .value = { .idata = 115200 }, .min = {.idata = 9600 }, .max = { .idata = 560800} },

			{.config = const_cast<char *>("") } };
// @formatter:on
int TelescopeConfiguration::eqmount_config(EqMountServer *server,
		const char *cmd, int argn, char *argv[]) {
	char buf[256];
	if (argn == 0) {
		// Print all config names
		ConfigNode *p = getInstance().head;
		stprintf(server->getStream(), "%s", cmd);
		for (; p; p = p->next) {
			stprintf(server->getStream(), " %s", p->config->config);
			// Get string representing the value
//			getStringFromConfig(p->config, buf, sizeof(buf));
//			stprintf(server->getStream(), "%s %s,%s,%s,%s\r\n", cmd,
//					p->config->config, p->config->name,
//					typeName(p->config->type), buf);
		}
		stprintf(server->getStream(), "\r\n");
	} else {
		char *config_name = argv[0]; // Name of the config in question
		ConfigItem *config = getInstance().getConfigItem(config_name);
		if (!config) {
			stprintf(server->getStream(), "%s Error: config %s not found\r\n",
					cmd, config_name);
			return ERR_PARAM_OUT_OF_RANGE;
		}
		if (argn == 1) {
			// Print value of the config
			getStringFromConfig(config, buf, sizeof(buf));
			stprintf(server->getStream(), "%s %s\r\n", cmd, buf);
		} else if (argn == 2) {
			if (strcmp(argv[1], "default") == 0) {
				// Print default value of the config
				// TODO
			} else if (strcmp(argv[1], "name") == 0) {
				// Print name
				stprintf(server->getStream(), "%s %s\r\n", cmd, config->name);
			} else if (strcmp(argv[1], "help") == 0) {
				// Print help
				stprintf(server->getStream(), "%s %s\r\n", cmd, config->help);
			} else if (strcmp(argv[1], "type") == 0) {
				// Print type
				stprintf(server->getStream(), "%s %s\r\n", cmd,
						typeName(config->type));
			} else if (strcmp(argv[1], "info") == 0) {
				// Print type, value, name and help
				getStringFromConfig(config, buf, sizeof(buf));
				stprintf(server->getStream(), "%s %s,%s,%s,%s\r\n", cmd,
						typeName(config->type), buf, config->name,
						config->help);
			} else if (strcmp(argv[1], "limit") == 0) {
				// Print min/max
				if (config->type != DATATYPE_STRING
						&& config->type != DATATYPE_BOOL && !config->extra) {
					switch (config->type) {
					case DATATYPE_INT:
						stprintf(server->getStream(), "%s %d %d\r\n", cmd,
								config->min.idata, config->max.idata);
						break;
					case DATATYPE_DOUBLE:
						stprintf(server->getStream(), "%s %.8f %.8f\r\n", cmd,
								config->min.ddata, config->max.ddata);
						break;
					default:
						break;
					}
				} else {
					stprintf(server->getStream(),
							"%s limit not supported for %s.\r\n", cmd,
							config->config);
				}
			} else {
				// Set value
				char *value = argv[1];
				char *s;
				int i;
				double d;
				bool b;
				switch (config->type) {
				case DATATYPE_INT:
					i = strtol(value, &s, 10);
					if (s == value || setIntToConfig(config, i)) {
						return ERR_PARAM_OUT_OF_RANGE;
					}
					break;
				case DATATYPE_DOUBLE:
					d = strtod(value, &s);
					if (s == value || setDoubleToConfig(config, d)) {
						return ERR_PARAM_OUT_OF_RANGE;
					}
					break;
				case DATATYPE_BOOL:
					if (strcmp(value, "true") == 0) {
						b = true;
					} else if (strcmp(value, "false") == 0) {
						b = false;
					} else {
						return ERR_PARAM_OUT_OF_RANGE;
					}
					if (setBoolToConfig(config, b)) {
						return ERR_PARAM_OUT_OF_RANGE;
					}
					break;
				case DATATYPE_STRING:
					if (setStringToConfig(config, value)) {
						return ERR_PARAM_OUT_OF_RANGE;
					}
					break;
				}
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
		r->next = q;
		r->config = new ConfigItem(*p);
		r->default_config = p;
		r->key = k++;
		q = r;
	}
	head = r;

#ifdef NVSTORE_ENABLED
	NVStore::get_instance().set_max_keys(64);
#endif

	EqMountServer::addCommand(
			ServerCommand("config", "Configuration subsystem",
					TelescopeConfiguration::eqmount_config));
}

int TelescopeConfiguration::getIntFromConfig(ConfigItem *config) {
	if (config->type != DATATYPE_INT) {
		error("Data type mismatch: wanted %s, actual %s",
				typeName(DATATYPE_INT), typeName(config->type));
	}
	return config->value.idata;
}

double TelescopeConfiguration::getDoubleFromConfig(ConfigItem *config) {
	if (config->type != DATATYPE_DOUBLE && config->type != DATATYPE_INT) {
		error("Data type mismatch: wanted %s, actual %s",
				typeName(DATATYPE_DOUBLE), typeName(config->type));
	}
	return (config->type == DATATYPE_DOUBLE) ?
			config->value.ddata : config->value.idata;
}

bool TelescopeConfiguration::getBoolFromConfig(ConfigItem *config) {
	if (config->type != DATATYPE_BOOL) {
		error("Data type mismatch: wanted %s, actual %s",
				typeName(DATATYPE_BOOL), typeName(config->type));
	}
	return config->value.bdata;
}

bool TelescopeConfiguration::setIntToConfig(ConfigItem *config, int value) {
	if (config == NULL) {
		debug_if(TC_DEBUG, "Null config");
		return true;
	}
	if (config->type != DATATYPE_DOUBLE && config->type != DATATYPE_INT) {
		debug_if(TC_DEBUG, "Data type mismatch: wanted %s, actual %s",
				typeName(DATATYPE_INT), typeName(config->type));
		return true;
	}
	if (config->type == DATATYPE_INT)
		config->value.idata = value;
	else
		config->value.ddata = value;
	return false;
}

bool TelescopeConfiguration::setDoubleToConfig(ConfigItem *config,
		double value) {
	if (config == NULL) {
		debug_if(TC_DEBUG, "Null config");
		return true;
	}
	if (config->type != DATATYPE_DOUBLE) {
		error("Data type mismatch: wanted %s, actual %s",
				typeName(DATATYPE_DOUBLE), typeName(config->type));
		return true;
	}
	config->value.ddata = value;
	return false;
}

bool TelescopeConfiguration::setBoolToConfig(ConfigItem *config, bool value) {
	if (config == NULL) {
		debug_if(TC_DEBUG, "Null config");
		return true;
	}
	if (config->type != DATATYPE_BOOL) {
		debug_if(TC_DEBUG, "Data type mismatch: wanted %s, actual %s",
				typeName(DATATYPE_BOOL), typeName(config->type));
		return true;
	}
	config->value.bdata = value;
	return false;
}

bool TelescopeConfiguration::setStringToConfig(ConfigItem *config,
		char *value) {
	if (config == NULL) {
		debug_if(TC_DEBUG, "Null config");
		return true;
	}
	if (config->type != DATATYPE_STRING) {
		debug_if(TC_DEBUG, "Data type mismatch: wanted %s, actual %s",
				typeName(DATATYPE_STRING), typeName(config->type));
		return true;
	}
	strncpy(config->value.strdata, value, sizeof(config->value.strdata));
	return false;
}

ConfigItem* TelescopeConfiguration::getConfigItemCheck(const char *name) {
	ConfigItem *config = getConfigItem(name);
	if (!config) {
		error("Config not found: %s", name);
		return NULL;
	}
	return config;
}

char* TelescopeConfiguration::getStringFromConfig(ConfigItem *config,
		char buf[], int len) {
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

static void setConfigValue(ConfigItem *config, const char *value) {
	switch (config->type) {
	case DATATYPE_INT:
		config->value.idata = strtol(value, NULL, 10);
		if (!config->extra
				&& (config->value.idata > config->max.idata
						|| config->value.idata < config->min.idata)) {
			error("'%s' value out of range: must be > %d and < %d",
					config->config, config->max.idata, config->min.idata);
		}
		break;
	case DATATYPE_DOUBLE:
		config->value.ddata = strtod(value, NULL);
		if (!config->extra
				&& (config->value.ddata > config->max.ddata
						|| config->value.ddata < config->min.ddata)) {
			error("'%s' value out of range: must be > %f and < %f",
					config->config, config->max.ddata, config->min.ddata);
		}
		break;
	case DATATYPE_BOOL:
		config->value.bdata = (strcmp(value, "true") == 0);
		break;
	case DATATYPE_STRING:
		strncpy(config->value.strdata, value, sizeof(config->value.strdata));
		break;
	}
}

void TelescopeConfiguration::setConfigByName(const char *name,
		const char *value) {
	// Find the config node if it already exists
	ConfigItem *config = getInstance().getConfigItem(name);
	if (config == NULL) {
		if (*value == '\0') {
			// Empty value string, don't add
			return;
		}
		// Create new node
		config = new ConfigItem;
		config->config = new char[strlen(name) + 1];
		strcpy(config->config, name);
		config->help = "";
		config->name = config->config;
		config->extra = true;
		ConfigNode *n = new ConfigNode, *m = getInstance().head;
		n->config = config;
		n->default_config = NULL;
		// Find tail
		while (m->next != NULL)
			m = m->next;
		m->next = n;
		n->key = m->key + 1;
		n->next = NULL;
		if (strcmp(value, "true") == 0 || strcmp(value, "false") == 0) {
			config->type = DATATYPE_BOOL;
		} else if (!isalpha(value[0])) {
			if (strchr(value, '.') == NULL) { // Look for decimal point
				config->type = DATATYPE_INT;
			} else {
				config->type = DATATYPE_DOUBLE;
			}
		} else {
			config->type = DATATYPE_STRING;
		}
	}
	setConfigValue(config, value);
}

ConfigItem* TelescopeConfiguration::getConfigItem(const char *name) {
	ConfigNode *p;
	for (p = head; p && (strcmp(p->config->config, name) != 0); p = p->next)
		;
	if (!p)
		return NULL;
	else
		return p->config;
}

TelescopeConfiguration::~TelescopeConfiguration() {
	for (ConfigNode *q = head; q;) {
		ConfigNode *p = q->next;
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
			debug("Syntax error in line %d\n", lineno);
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

		getInstance().setConfigByName(parameter, value);
	}
}

void TelescopeConfiguration::writeToFile(FILE *fp) {
	char buf[256];
	for (ConfigNode *p = getInstance().head; p; p = p->next) {
		getStringFromConfig(p->config, buf, sizeof(buf));
		fprintf(fp, "%s = %s\n", p->config->config, buf);
	}
}

#ifdef NVSTORE_ENABLED

int TelescopeConfiguration::saveConfig_NV() {
	char buf[256];
	bool success = true;
	NVStore &nv = NVStore::get_instance();

	for (ConfigNode *p = getInstance().head; p; p = p->next) {
		// Format of saving: [name] [value]
		size_t len = snprintf(buf, sizeof(buf), "%s ", p->config->config);
		getStringFromConfig(p->config, buf + len, sizeof(buf) - len);

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
		getInstance().setConfigByName(name, value);
	}

	return key >= sizeof(default_config) / sizeof(ConfigItem); // Success if at least all default items are set
}
#endif

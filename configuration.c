

#include "common.h"
#include "configuration.h"

#include <stdlib.h>
#include <confuse.h>

/**
 * The global instance
 */
static cfg_t * cfg = NULL;

/**
 * Definition of all options and their default values
 */
static cfg_opt_t opts[] = 
{
	CFG_STR("device", "/dev/video0", CFGF_NONE),
	CFG_INT("fps", 30, CFGF_NONE),

	CFG_FLOAT("k_p", 0.0, 0),
	CFG_FLOAT("k_i", 0.0, 0),
	CFG_FLOAT("k_d", 0.0, 0),
	CFG_FLOAT("k_error", 0.0, 0),
	CFG_FLOAT("k_error_diff", 0.0, 0),
	CFG_INT("speed", 0, 0),

	CFG_INT("slice_upper_start", 0, 0),
	CFG_INT("slice_upper_end", 0, 0),
	CFG_INT("slice_lower_start", 0, 0),
	CFG_INT("slice_lower_end", 0, 0),

	CFG_FLOAT("k_brightness", 0.0, 0),
	CFG_FLOAT("k_constrast", 0.0, 0),

	CFG_INT("thr_enable", 0, 0),
	CFG_INT("thr_upper", 0, 0),
	CFG_INT("thr_lower", 0, 0),

	CFG_END()
};


void config_init()
{
	cfg = cfg_init(opts, CFGF_NONE);
	if (config_reload() < 0)
	{
		printf("[config] Failed loading configuration file, exiting...\n");
		exit(-1);
	}
}

int config_reload()
{
	if (cfg_parse(cfg, CONFIG_FILE) == CFG_PARSE_ERROR)
	{
		return -1;
	}
	return 0;
}

char * config_get_str(const char * name)
{
	return cfg_getstr(cfg, name);
}

float config_get_float(const char * name)
{
	return (float) cfg_getfloat(cfg, name);
}

int config_get_int(const char * name)
{
	return (int) cfg_getint(cfg, name);
}




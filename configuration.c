

#include "common.h"
#include "configuration.h"

#include <stdlib.h>
#include <confuse.h>

/**
 * Definition of the global configuration
 */
conf_t conf;

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

	CFG_FLOAT("k_p", 0, 0),
	CFG_FLOAT("k_i", 0, 0),
	CFG_FLOAT("k_d", 0, 0),

	CFG_FLOAT("k_p_fast", 0, 0),
	CFG_FLOAT("k_i_fast", 0, 0),
	CFG_FLOAT("k_d_fast", 0, 0),

	CFG_FLOAT("k_error", 0, 0),
	CFG_FLOAT("k_error_diff", 0, 0),

	CFG_SIMPLE_INT("speed_straight", &conf.speed_straight),
	CFG_SIMPLE_INT("speed_normal", &conf.speed_normal),
	CFG_SIMPLE_INT("speed_slow", &conf.speed_slow),
	CFG_SIMPLE_INT("speed_fast", &conf.speed_fast),

	CFG_SIMPLE_INT("slice_upper_start", &conf.slice_upper_start),
	CFG_SIMPLE_INT("slice_upper_end", &conf.slice_upper_end),
	CFG_SIMPLE_INT("slice_lower_start", &conf.slice_lower_start),
	CFG_SIMPLE_INT("slice_lower_end", &conf.slice_lower_end),

	CFG_SIMPLE_INT("mass_horizontal_lower", &conf.mass_horizontal_lower),
	CFG_SIMPLE_INT("mass_horizontal_upper", &conf.mass_horizontal_upper),
	CFG_SIMPLE_INT("mass_cross_lower", &conf.mass_cross_lower),
	CFG_SIMPLE_INT("mass_cross_upper", &conf.mass_cross_upper),
	CFG_SIMPLE_INT("mass_bypath_lower", &conf.mass_bypath_lower),
	CFG_SIMPLE_INT("mass_bypath_upper", &conf.mass_bypath_upper),
	CFG_SIMPLE_INT("mass_end_lower", &conf.mass_end_lower),
	CFG_SIMPLE_INT("mass_end_upper", &conf.mass_end_upper),

	CFG_FLOAT("w_k_p", 0, 0),
	CFG_FLOAT("w_k_i", 0, 0),
	CFG_FLOAT("w_k_d", 0, 0),
	CFG_FLOAT("w_diff_p", 0, 0),

	CFG_SIMPLE_INT("w_max_sum_error", &conf.w_max_sum_error),
	CFG_SIMPLE_INT("w_max_error", &conf.w_max_error),
	CFG_SIMPLE_INT("w_setpoint", &conf.w_setpoint),
	CFG_SIMPLE_INT("w_speed", &conf.w_speed),
	

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

	conf.k_p = cfg_getfloat(cfg, "k_p");
	conf.k_i = cfg_getfloat(cfg, "k_i");
	conf.k_d = cfg_getfloat(cfg, "k_d");
	conf.k_p_fast = cfg_getfloat(cfg, "k_p_fast");
	conf.k_i_fast = cfg_getfloat(cfg, "k_i_fast");
	conf.k_d_fast = cfg_getfloat(cfg, "k_d_fast");

	conf.k_error = cfg_getfloat(cfg, "k_error");
	conf.k_error_diff = cfg_getfloat(cfg, "k_error_diff");

	conf.w_k_p = cfg_getfloat(cfg, "w_k_p");
	conf.w_k_i = cfg_getfloat(cfg, "w_k_i");
	conf.w_k_d = cfg_getfloat(cfg, "w_k_d");
	conf.w_diff_p = cfg_getfloat(cfg, "w_diff_p");

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






#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

typedef struct conf {

	// Speeds
	int speed_straight, speed_slow, speed_normal, speed_fast;

	// Line PID
	float k_p, k_i, k_d;
	float k_error, k_error_diff;

	// Wall PID
	float w_k_p, w_k_i, w_k_d;
	float w_diff_p;
	int w_speed, w_setpoint, w_max_sum_error, w_max_error;

	// Line mass limits
	int mass_horizontal_lower, mass_horizontal_upper;
	int mass_cross_lower, mass_cross_upper;
	int mass_bypath_lower, mass_bypath_upper;
	int mass_end_lower, mass_end_upper;

	// Slices
	int slice_upper_start, slice_upper_end;
	int slice_lower_start, slice_lower_end;

} conf_t;


/**
 * Declaration of the global config object
 */
extern conf_t conf;


void config_init();
int config_reload();
char * config_get_str(const char * name);
float config_get_float(const char * name);
int config_get_int(const char * name);

#endif


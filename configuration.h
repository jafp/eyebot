

#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

void config_init();
int config_reload();
char * config_get_str(const char * name);
float config_get_float(const char * name);
int config_get_int(const char * name);

#endif




#ifndef _LOG_H_
#define _LOG_H_

typedef struct log_fields {
	unsigned long time;
	unsigned long frame;
	
	int error_lower_x;
	int error_upper_x;

	int mass;

	float P;
	float I;
	float D;
	
	float correction;

	int speed_left;
	int speed_right;

	int speed_ref_left;
	int speed_ref_right;

	int tacho_left;
	int tacho_right;

} log_fields_t;

typedef struct log_entry {
	struct log_fields fields;
	struct log_entry * next;
} log_entry_t;

typedef struct log_list {
	unsigned long entries;
	struct log_entry * first;
	struct log_entry * last;
} log_list_t;

log_list_t * log_create();
log_entry_t * log_entry_create();
void log_add(log_list_t * log, log_entry_t * entry);
int log_dump(log_list_t * log, const char * filename);
void log_free(log_list_t * log);

#endif 

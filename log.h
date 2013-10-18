

#ifndef _LOG_H_
#define _LOG_H_

typedef struct log_fields {
	unsigned long time;
	unsigned long frame;
	
	int mass;
	int center_x;
	int center_y;

	int tacho_left;
	int tacho_right;

	int error;
	int diff;

	int speed_left;
	int speed_right;

} log_fields_t;

typedef struct log_entry {
	struct log_fields fields;
	struct log_entry * next;
} log_entry_t;

typedef struct log_entry log_t;

log_t * log_create();
void log_add(log_t * log, log_entry_t * entry);
void log_dump(log_t * log, const char filename);
void log_free(log_t * log);

#endif 

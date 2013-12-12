
#ifndef _PID_H_
#define _PID_H_

typedef struct pid_data {
	// The set point
	int set_point;
	// The P, I and D factors
	float P, I, D;
	// Last process value
	float last_process_value;
	// Sum of errors (process values)
	int sum_error;
	// Max error 
	int max_error;
	// Max sum error
	int max_sum_error;
	// Callback for setting the new control variable
	void (*set_cv)(struct pid_data *, float);
} pid_data_t;


void pid_ctrl(pid_data_t * pid, int process_value);
void pid_reset(pid_data_t * pid);

#endif

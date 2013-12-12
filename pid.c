
#include <stdlib.h>
#include "pid.h"

void pid_ctrl(pid_data_t * pid, int process_value)
{
	float p, i, d;
	float temp, cv;
	int error;

	//error = pid->set_point - process_value;
	error = process_value;
	//pid->last_set_point = pid->set_point;

	// Check limits (pid->max_error)
	if (abs(error) > pid->max_error)
	{
		// TODO
	}

	// Calculate P-term
	p = pid->P * error;

	// Calculate I-term
	pid->sum_error = pid->sum_error + error;
	if (pid->sum_error > pid->max_sum_error || pid->sum_error < -pid->max_sum_error)
	{
		pid->sum_error = 0;
	}

	i = pid->I * pid->sum_error;
	// TODO Check for integral wind-up

	// Calculate D-term
	d = pid->D * (pid->last_process_value - process_value);
	pid->last_process_value = process_value;

	// Total correction (control variable)
	cv = p + i + d;

	if (pid->set_cv != NULL)
	{
		pid->set_cv(pid, cv);
	}
}

void pid_reset(pid_data_t * pid)
{
	pid->sum_error = 0;
}

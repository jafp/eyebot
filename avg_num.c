
#include <stdlib.h>
#include "avg_num.h"

/**
 * Calculate the average of `num`.
 */
static int calculate_average(avg_num_t * num)
{
	int i, sum = 0;
	for (i = 0; i < num->length; i++)
	{
		sum += num->values[i];
	}
	return sum / num->length;
}

/** 
 * Create and allocate a new average number structure.
 */
void avg_num_create(avg_num_t * num, int length)
{
	if (num == 0)
	{
		num = (avg_num_t *) malloc(sizeof(avg_num_t));
	}
	num->values = calloc(length, sizeof(int));
	num->length = length;

	avg_num_clear(num);
}

/**
 * Add a number to the average number and return the 
 * newly calculated average.
 */
int avg_num_add(avg_num_t * num, int n)
{
	num->values[num->idx++] = n;
	if (num->idx >= num->length)
	{
		num->idx = 0;
	}
	num->avg = calculate_average(num);
	return num->avg;
}

/**
 * Clear all values and reset the average and index values
 * to zero.
 */
void avg_num_clear(avg_num_t * num)
{
	int i;
	for (i = 0; i < num->length; i++)
	{
		num->values[i] = 0;
	}
	num->avg = num->idx = 0;
}

#ifndef _AVG_NUM_H_
#define _AVG_NUM_H_

typedef struct avg_num {
	int length;
	int avg;
	int idx;
	int * values;
} avg_num_t;

void avg_num_create(avg_num_t * num, int length);
int avg_num_add(avg_num_t * num, int n);
void avg_num_clear(avg_num_t * num);

#endif 


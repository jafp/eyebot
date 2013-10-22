

#ifndef _BROADCAST_H_
#define _BROADCAST_H_

int broadcast_init();
int broadcast_start();
void broadcast_release();
void broadcast_send(int l_x, int l_y, int u_x, int u_y, int error_lower, 
	int error_upper, int mass, unsigned char * buffer);

#endif

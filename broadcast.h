

#ifndef _BROADCAST_H_
#define _BROADCAST_H_

int broadcast_init();
int broadcast_start();
void broadcast_send(int x, int y, int error, unsigned char * buffer);

#endif

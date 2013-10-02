
#ifndef _CAM_H_
#define _CAM_H_

#include <unistd.h>
#include <sys/time.h>

// Forward declaration of context structure
struct cam_ctx;

struct __buffer {
        void   *start;
        size_t  length;
};

struct cam_config {
	unsigned int width;
	unsigned int height;
	unsigned int fps;	
	void (*frame_cb)(struct cam_ctx *, void *, int length);
};

struct cam_ctx {
	struct cam_config config;

	int run;
	int fd;
	char * dev;
	struct __buffer * buffers;
	unsigned int n_buffers;

	struct timeval start, end;
	unsigned long frame_count;
};


void cam_init(struct cam_ctx *);
void cam_uninit(struct cam_ctx *);

void cam_start_capturing(struct cam_ctx *);
void cam_stop_capturing(struct cam_ctx *);

void cam_loop(struct cam_ctx *);
void cam_end_loop(struct cam_ctx *);

double cam_get_measured_fps(struct cam_ctx * ctx);

#endif

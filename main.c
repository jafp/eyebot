
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <netdb.h>
#include <assert.h>	
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <pthread.h>

#include "common.h"
#include "i2c.h"
#include "broadcast.h"
#include "motor_ctrl.h"
#include "cam.h"


#define SERVER_HOSTNAME		"10.42.0.1"
#define SERVER_PORT			"23000"

#define DEV					"/dev/video0"
//#define FPS 				60
#define FPS 				125

//#define THRESHOLD			100
#define THRESHOLD 			90

#define WIDTH 				320
#define HEIGHT 				240
#define IMG_SIZE 			(WIDTH * HEIGHT)

#define FLOOR				255
#define LINE				0

static struct cam_ctx * ctx;
static unsigned long frame_counter;
static unsigned int image_size;
static unsigned char * buffer;
static int socket_fd;

static int speed_ref = INITIAL_SPEED;

static int speed_l = INITIAL_SPEED, 
			speed_r = INITIAL_SPEED;

static int 	last_error = 0,
			cur_error = 0;

/**
 * Function prototypes
 */
static int update_loop(int error, int x, int y, int mass);


/**
 * Callback fired when a frame is ready.
 * 
 * \param ctx Pointer to the current camera context
 * \param frame Pointer to the frame data (planar image data)
 * \param length The of the frame data (in bytes, not pixels)
 */
static void frame_callback(struct cam_ctx * ctx, void * frame, int length)
{
	int i, p = 0;
	unsigned char v;
	unsigned char * ptr;
	unsigned int x = 0;
	unsigned int y = 0;
	unsigned int count = 0, error = 0;
	
	ptr = (unsigned char *) frame;

	//
	// Image processing part of the code!
	//
	// The following steps are done:
	//  - Every pixel below a curtain gray scale value are "marked" as the line,
	//    and the rest of the pixels are marked as the floor. The line pixels
	//	  are colored black, and the rest white.
	//
	//  - The X and Y values of each pixel is added together, and the total
	// 	  number of "line pixels" are counted.
	//
	//  - ...

	for (i = 0; i < IMG_SIZE; i++)
	{
		v = (*ptr);

		// Do thresholding
		v = buffer[i] = v < THRESHOLD ? LINE : FLOOR;

		if (v == 0)
		{
			// Calculate the X,Y coordinate from the current array position.
			// TODO: Remove hardcoded image size values
			x += i % 320;
			y += i / 320;
			count++;
		}

		ptr += 2;
	}

	// Calculate the average X,Y position, and then the "error", which is the
	// difference of the X position and the center of the image.
	if (count > 0)
	{
		x = x / count;
		y = y / count;
		error = 160 - x;
	}

	// Transmit every 4rd frame over sockets.
	// This is 15 frames per second when we are capturing
	// 60 frames per second from the camera.
	if (frame_counter % 4 == 0)
	{
		broadcast_send(x, y, error, buffer);
	}

	frame_counter++;

	// Dispatch the updating to another function
	update_loop(error, x, y, count);
}

static unsigned char limit_speed(int speed)
{	
	if (speed > 255)
	{
		return 255;
	}
	else if (speed < 0)
	{
		return 0;
	}
	return speed;
}

/**
 * Update loop callback. Called whenever a new image has been processed.
 * From this point, it's all about calculating new speeds for the motors,
 * and transmitting the updates to them.
 * 
 * \param error The error calculated from the image (diff. in X coordinate)
 * \param x The calculated X position of the center mass of the line
 * \param x The calculated Y position of the center mass of the line
 * \param mass The number of pixels identified as the line
 */
static int update_loop(int error, int x, int y, int mass)
{
	// Experiment - stop motor if we are way off the line
	if (mass < 50)
	{
		//motor_ctrl_set_speed(0, 0);
		//return;
	}

	float 	kp = 1.0,	
			kd = 5.0,

			k_err = 0.1;

	int diff,
		enc_diff;

	unsigned char enc_left, enc_right;
	motor_ctrl_get_speed(&enc_left, &enc_right);

	enc_diff = enc_left - enc_right;

	// Scale error
	error =  (k_err * error) + enc_diff;

	// Differentiate
	last_error = cur_error;
	cur_error = error;
	diff = cur_error - last_error;

	// Calculate compensation
	speed_l = speed_ref - (kp * error) + (kd * diff);
	speed_r = speed_ref + (kp * error) + (kd * diff);

	// Print a lot of useful stuff!
	printf("[%10d] speed: %4d %4d - error (image): %4d  - encoder: %3d, %3d - mass: %4d \n", 
		frame_counter, speed_l, speed_r, error, enc_left, enc_right, mass);
	
	// Each speed is limited to the interval 0-255 (unsigned 8-bit number)
	motor_ctrl_set_speed(limit_speed(speed_l), limit_speed(speed_r));
}

/**
 * SIGINT signal handler.
 * 
 * \param signal Signal number
 */
static void sigint_handler(int signal)
{
	cam_end_loop(ctx);
}

static void * processing_thread_fn(void * ptr)
{
	cam_loop(ctx);
	pthread_exit(0);
}

/**
 * Main entry point.
 */
int main(int argc, char ** argv)
{
	pthread_t processing_thread, shell_thread;
	struct addrinfo hints, *res;

	ctx = (struct cam_ctx *) malloc(sizeof(struct cam_ctx));
	if (ctx == NULL)
	{
		// We are out of memory...this never happens!
	}

	// Camera configuration
	ctx->config.frame_cb = frame_callback;
	ctx->config.width = WIDTH;
	ctx->config.height = HEIGHT;
	ctx->config.fps = FPS;
	ctx->dev = DEV;

	frame_counter = 0;
	buffer = malloc(IMG_SIZE);

	printf("\n=== Eyecam ===\n");
	printf("Config: w: %d, h: %d, fps (expected): %d\n", ctx->config.width, ctx->config.height, ctx->config.fps);


	// Open i2c bus (by internally opening the i2c device driver)
	if (i2c_bus_open() < 0)
	{
		printf("Failed opening I2C bus, exiting...\n");
		exit(-1);
	}

	// Catch CTRL-C signal and end looping
	signal(SIGINT, sigint_handler);

	// Open camera and start capturing
	cam_init(ctx);
	cam_start_capturing(ctx);
	
	// Open TCP server socket, and start listening for connections	
	broadcast_init();
	broadcast_start();

	// Start motor at the initial speed
	motor_ctrl_set_speed(speed_l, speed_r);

	// Create the main thread
	if (pthread_create(&processing_thread, NULL, processing_thread_fn, ctx) < 0)
	{
		perror("pthread_create");
		exit(-1);
	}

	// Wait for the main thread and shell thread to finish
	pthread_join(processing_thread, NULL);
	// TODO wait for shell thread
	
	cam_stop_capturing(ctx);
	cam_uninit(ctx);

	// Stop robot
	motor_ctrl_set_speed(0, 0);

	broadcast_release();

	// Print some statistics.
	// TODO: Calculate and show some statistics while running?
	printf("\nActual fps: %f\n", cam_get_measured_fps(ctx));
	printf("Done.\n\n");

	return 0;
}


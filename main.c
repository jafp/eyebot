
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

#include "cam.h"

#define SERVER_HOSTNAME		"10.42.0.1"
#define SERVER_PORT			"23000"

#define DEV					"/dev/video0"
#define FPS 				60

#define THRESHOLD			100
#define WIDTH 				320
#define HEIGHT 				240
#define IMG_SIZE 			(WIDTH * HEIGHT)

#define FLOOR				255
#define LINE				0

static struct cam_ctx ctx;
static unsigned long frame_counter;
static unsigned int image_size;
static unsigned char * buffer;
static int socket_fd;

/**
 * Function prototypes
 */
static int update_loop(int error, int x, int y, int mass);


/**
 * Send a huge buffer over several send() system calls. 
 */
static int sendall(int s, char *buf, int *len)
{
    int total = 0;        // how many bytes we've sent
    int bytesleft = *len; // how many we have left to send
    int n;

    while(total < *len) 
    {
        n = send(s, buf + total, bytesleft, 0);
        if (n == -1) 
        { 
        	break; 
        }
        total += n;
        bytesleft -= n;
    }

    *len = total; // return number actually sent here
    return n == -1 ? -1 : 0; // return -1 on failure, 0 on success
}

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
		i = send(socket_fd, &x, sizeof(x), 0);
		if (i == -1)
		{
			perror("Could not send x");
			return;
		}

		i = send(socket_fd, &y, sizeof(y), 0);
		if (i == -1)
		{
			perror("Could not send y");
			return;
		}

		int length = IMG_SIZE;
		if (sendall(socket_fd, buffer, &length) == -1)
		{
			perror("Could not send frame buffer");
			return;
		}

		// This should teoretically never fails, but...
		assert(length == IMG_SIZE);
	}

	frame_counter++;

	// Dispatch the updating to another function
	update_loop(error, x, y, count);
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
	// TODO! 
	// Read tacho from motor
	// Do magic
	// Tell motors new speed
}

/**
 * SIGINT signal handler.
 * 
 * \param signal Signal number
 */
static void sigint_handler(int signal)
{
	cam_end_loop(&ctx);
}

/**
 * Main entry point.
 */
int main(int argc, char ** argv)
{
	struct addrinfo hints, *res;

	// Configuration
	ctx.config.frame_cb = frame_callback;
	ctx.config.width = WIDTH;
	ctx.config.height = HEIGHT;
	ctx.config.fps = FPS;
	ctx.dev = DEV;

	frame_counter = 0;
	buffer = malloc(IMG_SIZE);

	printf("\n=== Eyecam ===\n");
	printf("Config: w: %d, h: %d, fps (expected): %d\n", ctx.config.width, ctx.config.height, ctx.config.fps);

	// ----------------------------------------
	// Sockets
	// ----------------------------------------

	// TODO: Refactor all the networking/socket stuff out of the main file.
	// This is mostly relevant during debugging and testing.

	socket_fd = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    if(socket_fd == -1)
    {
        printf("Could not make a socket\n");
        return 1;
    }

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	getaddrinfo(SERVER_HOSTNAME, SERVER_PORT, &hints, &res);

	if (connect(socket_fd, res->ai_addr, res->ai_addrlen) == -1)
	{
		printf("Failed to connect\n");
		return 1;
	}

	// ----------------------------------------
	// End Sockets
	// ----------------------------------------

	// Catch CTRL-C signal and end looping
	signal(SIGINT, sigint_handler);

	// Open camera and start capturing
	cam_init(&ctx);
	cam_start_capturing(&ctx);

	// Loop forever - or till the loop is stopped by calling `cam_end_loop`.
	// `cam_end_loop`is called from a SIGINT signal handler, which means
	// the image capture can by stopped by send CTRL-C in the terminal.
	cam_loop(&ctx);

	cam_stop_capturing(&ctx);
	cam_uninit(&ctx);

	// Print some statistics.
	// TODO: Calculate and show some statistics while running?
	printf("\nActual fps: %f\n", cam_get_measured_fps(&ctx));
	printf("Done.\n\n");

	return 0;
}

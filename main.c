#include <stdio.h>
#include <math.h>
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
#include "configuration.h"
#include "i2c.h"
#include "broadcast.h"
#include "motor_ctrl.h"
#include "camera.h"
#include "log.h"


/**
 * Image size
 */
#define WIDTH 					320
#define HEIGHT 					240
#define IMG_SIZE 				(WIDTH * HEIGHT)

/** 
 * Image processing
 */
#define FLOOR					255
#define LINE					0

#define INDEX(y)				( y * WIDTH )

#define T 						0.03333
#define T_90_DEG				1300000
#define T_180_DEG				2600000

#define PI 						3.14159265

#define SPEED_LIMIT				100

/**
 * Overall states for the state machine.
 */
typedef enum {
	CALIBRATE,
	WAITING,
	GOTO_LINE,
	GOTO_LINE_TURN_LEFT,
	FOLLOW_LINE,
	BRAKE_DOWN,
	FOLLOW_LINE_SPEEDY,
	FOLLOW_LINE_AFTER_WALL,
	GOTO_WALL_AND_BACK,
	FOLLOW_WALL,
	TRACK_COMPLETE
} state_t;

/**
 * Information about a `slice` of the image, 
 * including mass of the line, error and so on.
 */
typedef struct slice {
	int x, y, mass, error;
} slice_t;

typedef struct img_part {
	int length;
	unsigned char * data;
} img_part_t;


int dilation_mask[4][4] = {
	{ 0, 1, 1, 0 },
	{ 1, 1, 1, 1 },
	{ 1, 1, 1, 1 },
	{ 0, 1, 1, 0 }
};

int dilation_mask_n = 4;


/**
 * List of log entries
 */
static log_list_t * logs;

/**
 * Camera interface (see cam.h)
 */
static camera_t * cam;

/**
 * Buffer for temporary image data
 */ 
static unsigned char * buffer;

/**
 * Frame counter
 */
static unsigned long frame_counter;

/**
 * Current state.
 */
static state_t current_state;


/** 
 * Variables used in the PID controller.
 */
static int speed_ref, speed_ref_slow, speed_ref_fast;


static int speed_l;
static int speed_r;

static float last_error = 0;
static float last_error_2 = 0;

static float last_correction = 0;
static int I_sum = 0;
static float k_p, k_i, k_d, k_error, k_error_diff;
static float k_constrast, k_brightness;
static int slice_upper_start, slice_upper_end, slice_lower_start, slice_lower_end;
static int thr_enable, thr_lower, thr_upper;

static int cnt;

/**
 * Variables containing masses for different types of line intersections
 */
static int mass_horizontal_lower, mass_horizontal_upper;
static int mass_cross_lower, mass_cross_upper;
static int mass_bypath_lower, mass_bypath_upper;

#define MASS_AVG_CNT	2

static int latest_masses[MASS_AVG_CNT];
static int latest_mass_idx = 0;

/**
 * Function prototypes
 */
static int update_loop(int mass, slice_t * upper, slice_t * lower);

static void reset()
{
	int i;

	latest_mass_idx = 0;
	for (i = 0; i < MASS_AVG_CNT; i++)
	{
		latest_masses[i] = 0;
	}
	
	motor_ctrl_forward();
	I_sum = 0;
}

/**
 * 
 */
static int add_mass(int mass)
{
	int i, sum = 0;
	latest_masses[latest_mass_idx++] = mass;
	if (latest_mass_idx == MASS_AVG_CNT)
	{
		latest_mass_idx = 0;
	}
	for (i = 0; i < MASS_AVG_CNT; i++)
	{
		sum += latest_masses[i];
	}
	return sum / MASS_AVG_CNT;
}

/**
 * Calculate the "center of mass" of the given portion of the image,
 * given as an Y-offset and Y-length.
 */
static void calculate_center_of_mass(slice_t * pt, int y_offset_start, int y_offset_end)
{
	int offset_start, offset_end, sum = 0, x = 0, y = 0, i;
	pt->x = pt->y = pt->error = 0;

	offset_start = y_offset_start * WIDTH;
	offset_end = y_offset_end * WIDTH;

	for (i = offset_start; i < offset_end; i++)
	{
		if (buffer[i] == LINE)
		{
			x += i % WIDTH;
			y += i / WIDTH;
			sum++;
		}
	}

	if (sum > 0)
	{
		pt->x = x / sum;
		pt->y = y / sum;
		pt->error = (WIDTH / 2) - pt->x;
	}

	pt->mass = sum;
}

/**
 * Calculates histogram.
 *
 * \param hist Pointer to float array where to put the histogram
 * \param start Start row
 * \param end End row
 */
static void histogram(float * hist, int start, int end)
{
	int i;
	int ihist[256] = {0};
	
	start = INDEX(start);
	end = INDEX(end);

	for (i = start; i < end; i++)
	{
		ihist[buffer[i]]++;
	}
	for (i = 0; i < 256; i++)
	{
		hist[i] = (float)ihist[i] / (float)(end - start);
	}
}

/**
 * Optimum Thresholding algorithm from `The Pocket Handbook of Image 
 * Processing Algorithms in C`.
 *
 * \param start Start row (0 - HEIGHT)
 * \param end End row (0 - HEIGHT)
 */
static void optimum_thresholding(int start, int end, int nice)
{
	int y, x, j, flag, thr;

	float sum;
	float hist[256];
	
	
	histogram(hist, start, end);

	for (y = 0; y < 256; y++)
	{
		j = 0;
		sum = 0;
		for (x = -15; x <= 15; x++)
		{
			j++;
			if ((y-x) >= 0)
			{
				sum = sum + hist[y-x];
			}
		}
		hist[y] = sum / (float) j;
	}

	y = 50; //50 for normal track
	thr = 0;
	flag = 0;

	while (flag == 0 && y < 254)
	{
		if (hist[y-1] >= hist[y] && hist[y] < hist[y+1]) 
		{
			flag = 1;
			thr = y;
		}
		y++;
	}
	
	start = INDEX(start);
	end = INDEX(end);
	thr += nice;
	//thr = thr_lower;
	for (j = start; j < end; j++)
	{
		buffer[j] = buffer[j] < thr ? LINE : FLOOR;
	}
}

static void constrast()
{
	int i, tmp;
	for (i = 0; i < IMG_SIZE; i++)
	{
		tmp = buffer[i] * k_constrast;
		if (tmp > 255)
		{
			buffer[i] = 255;
		}
		else if (tmp < 0)
		{
			buffer[i] = 0;
		}
		else
		{
			buffer[i] = (unsigned char)tmp;
		}
	}
}

static void dilation(int N, int mask[N][N])
{
	int y, x, i, j, smax, off;
	int n = N/2;
	for (y = n; y < HEIGHT - n; y++)
	{
		for (x = n; x < WIDTH - n; x++)
		{
			off = x + y * WIDTH;
			smax = FLOOR;
			for (j = -n; j < n; j++)
			{
				for (i = -n; i < n; i++)
				{
					if (mask[i+n][j+n] == 1)
					{
						if (buffer[off] == LINE)
						{
							smax = LINE;
							break;
						}
					}
				}
				if (smax == LINE)
				{
					break;
				}
			}
			buffer[off] = smax;
		}
	}
}

/**
 * Calculate the robot's angle relative to the line.
 *
 * \param upper
 * \param lower
 */
static double angle_to_line(slice_t * upper, slice_t * lower)
{
	int x1, y1, x2, y2;

	// Return zero in case no line at all is found
	if (upper->x == 0 || upper->y == 0 || lower->x == 0 || lower->y == 0)
	{
		return 0;
	}

	x1 = upper->x - lower->x;
	y1 = upper->y - lower->y;

	x2 = lower->x;
	y2 = 0;

	// Calculate and return angle in degrees
	return ( (x1*x2+y1*y2) / (sqrt(x1*x1+y1*y1) * sqrt(x2*x2+y2*y2)) ) * (180/PI);
}


static void extract_slice(int start, int end, int peak_split, int nice)
{
	optimum_thresholding(start, end, nice);
	//dilation(dilation_mask_n, dilation_mask);
}

/** 
 * 
 */
static void extract_line()
{
	extract_slice(0, 44, 100, 0);
	extract_slice(44, 88, 100, 0);
	extract_slice(88, 144, 100, 0);
	extract_slice(144, 192, 100, 0);
	extract_slice(192, 240, 100, 0);
}


/**
 * Callback fired when a frame is ready.
 * 
 * \param cam Pointer to the current camera context
 * \param frame Pointer to the frame data (planar image data)
 * \param length The of the frame data (in bytes, not pixels)
 */
static void frame_callback(struct camera * cam, void * frame, int length)
{
	int i;
	unsigned char * ptr;
	unsigned int count = 0;
	int avg_mass;
	slice_t lower, upper;

	ptr = (unsigned char *) frame;

	// TODO UPDATE!!
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

	int tmp;
	// Copy to working buffer
	for (i = 0; i < IMG_SIZE; i++)
	{
		//tmp = (int)(*ptr) *  k_constrast + k_brightness;
		buffer[i] = (*ptr);//tmp > 255 ? 255 : tmp;
		ptr += 2;
	}

	if (current_state != CALIBRATE)
	{
		// Extract line
		extract_line();

		// 
		// Calculate center of mass at the upper half of the image.
		// (this is where the line is farest away)
		//
		calculate_center_of_mass(&upper, slice_upper_start, slice_upper_end);

		//
		// Calculate center of mass at the lower half of the image
		//
		calculate_center_of_mass(&lower, slice_lower_start, slice_lower_end);

		// Aggregated mass of line
		count = upper.mass + lower.mass;
	}

	avg_mass = add_mass(count);

	// Transmit every 4rd frame over sockets.
	// This is 15 frames per second when we are capturing
	// 60 frames per second from the camera.
	if (frame_counter % 3 == 0)
	{
		//printf("%d %d -- %d %d\n", lower.x, lower.y, upper.x, upper.y);
		broadcast_send(lower.x, lower.y, upper.x, upper.y, lower.error, 
			upper.error, avg_mass, buffer);
	}

	frame_counter++;

	// Dispatch the updating to another function
	update_loop(avg_mass, &upper, &lower);
}

/**
 * Limits the speed according to the limit specified in the
 * SPEED_LIMIT constant.
 *
 * \param speed The speed to limit
 * \return The limited speed
 */
static unsigned char limit_speed(int speed)
{	
	if (speed > SPEED_LIMIT)
	{
		return SPEED_LIMIT;
	}
	else if (speed < 0)
	{
		return 0;
	}
	return speed;
}

/**
 * Function that implements the actual discrete PID controller.
 *
 * \param mass
 * \param upper
 * \param lower
 * \param speed
 * \param Kerr
 * \param Kp
 * \param Ki
 * \param Kd
 */
static void pid_controller(int mass, slice_t * upper, slice_t * lower, int speed,
	float Kerr, float Kp, float Ki, float Kd)
{
	float P, I, D;
	float err;
	float correction;
	float err_diff;
	float mass_pct;
	int mass_limiter = 0;
	unsigned char tacho_left = 0, tacho_right = 0;

	// Scale error down
	err = lower->error * Kerr;

	//
	// Calculate PID
	//
	P = err * Kp;

	I_sum = (0.5 * I_sum) + err;

	// Avoid integral wind-up
	if (abs(I_sum) > 10)
	{
		I_sum = 0;
	}

	I = (0.5 * last_correction) + T * Ki * err;
	////I = I_sum * Ki;

	D = (Kd/T) * (err - last_error);
	////D = Kd * (err - last_error);
	
	// Total correction
	correction = P + I + D;
	//correction = last_correction + (Kp + Ki*T + Kd/T)*err + (-Kp - 2*Kd/T)*last_error + (Kd/T)*last_error_2;

	last_correction = correction;

	//last_error_2 = last_error;
	last_error = err;


	// Limit speed if the line has big changes in direction 
	// in the future
	err_diff = abs(lower->error - upper->error) * k_error_diff;

	// Calculate new speed
	speed_l = (int) round(speed - err_diff - correction);
	speed_r = (int) round(speed - err_diff + correction);

	//printf("> %4f %4f %4d %4d\n", correction, err, speed_l, speed_r);

	// Send new speeds to motor controller
	// (Each speed is limited to the interval 0-255 (unsigned 8-bit number))
	motor_ctrl_set_speed(limit_speed(speed_l), limit_speed(speed_r));

	//
	// Add log entry
	//

	log_entry_t * entry = log_entry_create();
	log_fields_t * f = &entry->fields;

	f->time = 0;
	f->frame = frame_counter;

	f->error_lower_x = lower->error;
	f->error_upper_x = upper->error;
	f->mass = mass;

	f->P = P;
	f->I = I;
	f->D = D;
	f->correction = correction;

	f->speed_left = speed_l;
	f->speed_right = speed_r;

	f->speed_ref_left = speed;
	f->speed_ref_right = speed;

	f->tacho_left = (int) tacho_left;
	f->tacho_right = (int) tacho_right;

	log_add(logs, entry);	

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
static int update_loop(int mass, slice_t * upper, slice_t * lower)
{
	switch (current_state)
	{	
		case CALIBRATE:
		{
			break;
		}
		/*
		 *
		 * Go straight till we see the line and then make a left-turn.
		 *
		 */
		case GOTO_LINE:
		{
			// Set speed - motor controller takes care of correcting the 
			// actual speed of the motors.
			motor_ctrl_set_speed(speed_ref_slow, speed_ref_slow);

			if (mass > mass_horizontal_lower && mass < mass_horizontal_upper)
			{
				printf("FOUND IT! (mass: %d)\n", mass);
				//current_state = FOLLOW_LINE;
				current_state = WAITING;	
			}
			break;
		}
		/*
		 *
		 *
		 */
		case GOTO_LINE_TURN_LEFT:
		{
			cnt--;
			if (cnt == 0)
			{
				current_state = FOLLOW_LINE;
				//motor_ctrl_set_speed(speed_ref, speed_ref);
				pid_controller(mass, upper, lower, speed_ref, k_error, k_p, k_i, k_d);
			}
			break;
		}
		/*
		 *
		 * Follow the line
		 *
		 */
		case FOLLOW_LINE:
		{
			pid_controller(mass, upper, lower, speed_ref, k_error, k_p, k_i, k_d);
			// TODO Detect crossing tape - goto wall and back!
			if (mass > mass_cross_lower && mass < mass_cross_upper)
			{

			}

			break;
		}
		/*
		 *
		 *
		 *
		 */
		case GOTO_WALL_AND_BACK:
		{
			// Sequential code for driving to the wall and back

			double distance;
			double position;
			double wheelbase = 250; // TODO
			double pos_per_millimeter = 1.72; // TODO pulses per rev / wheel circumfence
			double angle = angle_to_line(upper, lower);

			
			distance = ((90 + angle) * PI * wheelbase)/180;
			position = distance * pos_per_millimeter; 

			printf("%4.2f %4.2f %4.2f\n", angle, distance, position);

			break;
		}
		/*
		 *
		 * Follow line after wall
		 *
		 */
		case FOLLOW_LINE_AFTER_WALL:
		{
			if (mass > 20000)
			{
				current_state = FOLLOW_LINE_SPEEDY;
			}
			pid_controller(mass, upper, lower, speed_ref, k_error, k_p, k_i, k_d);
			break;
		}
		/*
 		 *
		 * Follow the line speedy
		 *
		 */
		case FOLLOW_LINE_SPEEDY:
		{
			pid_controller(mass, upper, lower, speed_ref_fast, k_error, k_p, k_i, k_d);
		}
		default: 
		{

		}
	}
}

/**
 * Load the configuration variables into memory.
 */
static void load_config()
{
	config_reload();

	speed_ref = config_get_int("speed");
	speed_ref_slow = config_get_int("speed_slow");
	speed_ref_fast = config_get_int("speed_fast");

	k_p = config_get_float("k_p");
	k_i = config_get_float("k_i");
	k_d = config_get_float("k_d");
	k_error = config_get_float("k_error");
	k_error_diff = config_get_float("k_error_diff");

	k_constrast = config_get_float("k_constrast");
	k_brightness = config_get_float("k_brightness");

	slice_upper_start = config_get_int("slice_upper_start");
	slice_upper_end = config_get_int("slice_upper_end");

	slice_lower_start = config_get_int("slice_lower_start");
	slice_lower_end = config_get_int("slice_lower_end");

	thr_enable = config_get_int("thr_enable");
	thr_upper = config_get_int("thr_upper");
	thr_lower = config_get_int("thr_lower");

	mass_horizontal_lower = config_get_int("mass_horizontal_lower");
	mass_horizontal_upper = config_get_int("mass_horizontal_upper");

	mass_cross_lower = config_get_int("mass_cross_lower");
	mass_cross_upper = config_get_int("mass_cross_upper");

	mass_bypath_lower = config_get_int("mass_bypath_lower");
	mass_bypath_upper = config_get_int("mass_bypath_upper");
}

/**
 * SIGINT signal handler.
 * 
 * \param signal Signal number
 */
static void sigint_handler(int signal)
{
	//cam_end_loop(cam);
	current_state = WAITING;
}

/**
 *
 */
static void * processing_thread_fn(void * ptr)
{
	cam_loop(cam);
	pthread_exit(0);
}

/**
 *
 */
static void * shell_thread_fn(void * ptr)
{
	int i;
	char buffer[255];

	while (1)
	{	
		printf(" >> ");
		if (fgets(buffer, 255, stdin) != NULL)
		{	
			// Skip empty line
			if (buffer[0] == '\n')
			{
				continue;
			}

			// Remove newline at the end
			for (i = 0; i < 255; i++)
			{
				if (buffer[i] == '\0' || buffer[i] == '\n')
				{
					buffer[i] = '\0';
				}
			}

			/**
			 *
			 */
			if (strcmp(buffer, "st") == 0)
			{	
				reset();
				//motor_ctrl_set_speed(20, 20);
				current_state = GOTO_LINE;
			}
			else if (strcmp(buffer, "st2") == 0)
			{	
				reset();
				//motor_ctrl_set_speed(speed_ref, speed_ref);
				current_state = FOLLOW_LINE;
			}
			/**
			 *
			 */
			else if (strcmp(buffer, "s") == 0)
			{
				motor_ctrl_brake();
				//motor_ctrl_set_speed(0, 0);
				current_state = WAITING;
			}
			else if (strcmp(buffer, "run") == 0)
			{
				motor_ctrl_forward();
				current_state = WAITING;
				motor_ctrl_set_speed(20, 20);
			}
			else if (strcmp(buffer, "r") == 0)
			{
				load_config();
				printf("Constants reloaded\n");
			}

			else if (strcmp(buffer, "startcal") == 0)
			{
				current_state = CALIBRATE;
			}

			else if (strcmp(buffer, "stopcal") == 0)
			{
				current_state = WAITING;
			}
			/**
			 *
			 */
			else if (strcmp(buffer, "exit") == 0)
			{
				cam_end_loop(cam);
				pthread_exit(0);
			}
			else if (strcmp(buffer, "w") == 0)
			{
				reset();
				current_state = GOTO_WALL_AND_BACK;
			}
			/**
			 *
			 */
			else if (strcmp(buffer, "goto") == 0)
			{
				current_state = GOTO_LINE;
			}
			else
			{
				printf("Command not found!\n");
			}
		}
	}
}

static void setup_camera()
{
	cam = (struct camera *) malloc(sizeof(struct camera));
	if (cam == NULL)
	{
		printf("Could not allocate memory for camera interface, exiting...\n");
		exit(-1);
	}

	// Camera configuration
	cam->config.frame_cb = frame_callback;
	cam->config.width = WIDTH;
	cam->config.height = HEIGHT;
	cam->config.fps = config_get_int("fps");
	cam->dev = config_get_str("device");
}


/**
 * Main entry point.
 */
int main(int argc, char ** argv)
{
	pthread_t processing_thread, shell_thread;
	struct addrinfo hints, *res;

	// Initialize variables
	frame_counter = 0;
	current_state = WAITING;
	buffer = (unsigned char *) malloc(IMG_SIZE);

	// Init and load the configuration file
	config_init();
	load_config();

	// Allocate and initialize the logging system
	logs = log_create();

	// Setup camera
	setup_camera();	

	// Open i2c bus (by internally opening the i2c device driver)
	if (i2c_bus_open() < 0)
	{
		printf("Failed opening I2C bus, exiting...\n");
		exit(-1);
	}

	// Print a nice welcome message
	printf("\n=== Eyecam ===\n");
	printf("Config: w: %d, h: %d, fps (expected): %d\n", cam->config.width, cam->config.height, cam->config.fps);

	// Catch CTRL-C signal and end looping
	signal(SIGINT, sigint_handler);

	// Open camera and start capturing
	cam_init(cam);
	cam_start_capturing(cam);
	
	// Open TCP server socket, and start listening for connections	
	broadcast_init();
	broadcast_start();

	reset();

	// Create the main processing thread (camera and update loop)
	pthread_create(&processing_thread, NULL, processing_thread_fn, NULL);

	// Create the shell thread
	pthread_create(&shell_thread, NULL, shell_thread_fn, NULL);
	
	// Wait for the main thread and shell thread to finish
	pthread_join(processing_thread, NULL);
	pthread_join(shell_thread, NULL);	
	
	cam_stop_capturing(cam);
	cam_uninit(cam);

	// Stop robot
	motor_ctrl_set_speed(0, 0);

	// Close server socket and drop connections
	broadcast_release();

	//
	// Dump log information to CSV file
	//
	char filename[64];
	time_t now;
	struct tm * timeinfo;

	time(&now);
	timeinfo = localtime(&now);
	sprintf(filename, "run-%d-%d-%d-%d-%d-%d.csv", timeinfo->tm_mday, 
		timeinfo->tm_mon + 1, timeinfo->tm_year + 1900, timeinfo->tm_hour, 
		timeinfo->tm_min, timeinfo->tm_sec);

	//printf("Logging run data to %s\n", filename);
	//log_dump(logs, filename);

	// Print some statistics.
	// TODO: Calculate and show some statistics while running?
	printf("\nActual fps: %f\n", cam_get_measured_fps(cam));
	printf("Done.\n\n");

	return 0;
}


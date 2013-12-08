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
#include "avg_num.h"
#include "ioexp.h"
#include "image.h"

#define delay(ms) 				(usleep(ms * 1000))

#define T 						0.03333

#define P_45					250
#define P_90					(2 * P_45)
#define P_135					(3 * P_45)
#define P_180					(4 * P_45)

#define SPEED_LIMIT				100

#define ROTATE_LEFT				1
#define ROTATE_RIGHT			2

/**
 * Overall states for the state machine.
 */
typedef enum {
	CALIBRATE,
	WAITING,
	START,
	GOTO_LINE,
	FOLLOW_LINE,
	FOLLOW_LINE_SPEEDY,
	FOLLOW_LINE_AFTER_WALL,
	FOLLOW_WALL,
	TRACK_COMPLETE,
	GOTO_WALL,
	FROM_WALL_TO_LINE,
	END_OF_LINE,
	STICK_TO_WALL
} state_t;

/**
 * List of log entries
 */
static log_list_t * logs;

/**
 * Camera interface
 */
static camera_t * cam;

/**
 * Buffer for temporary image data.
 * This buffer is the primary working buffer when
 * doing the image processing.
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

//static int speed_l;
//static int speed_r;

static float last_error = 0;
//static float last_error_2 = 0;

//static float last_correction = 0;
static int I_sum = 0;
static float k_p, k_i, k_d, k_error, k_error_diff;
static float k_constrast, k_brightness;
static int slice_upper_start, slice_upper_end, slice_lower_start, slice_lower_end;
//static int thr_enable, thr_lower, thr_upper;

//static int cnt;

/**
 * Variables containing masses for different types of line intersections
 */
static int mass_horizontal_lower, mass_horizontal_upper;
static int mass_cross_lower, mass_cross_upper;
static int mass_bypath_lower, mass_bypath_upper;

static avg_num_t avg_mass;
static avg_num_t avg_front_dist;
static avg_num_t avg_side_dist;

static int settling_cnt = 0;
static int settling_en = 0;

#define settling_check() 								\
	if (settling_en && settling_cnt-- > 0) { return; } 	\
	else { settling_en = 0; }							\

#define settling_set(frames)							\
	settling_en = 1; settling_cnt = frames				\

/**
 * Function prototypes
 */
static int update_loop(int mass, slice_t * upper, slice_t * lower);

/**
 * Reset all variables used by the controllers.
 */
static void reset()
{
	avg_num_clear(&avg_mass);
	avg_num_clear(&avg_front_dist);
	avg_num_clear(&avg_side_dist);

	motor_ctrl_set_state(STATE_SPEED);
	motor_ctrl_set_speed(0, 0);
	motor_ctrl_forward();

	I_sum = 0;
}

/**
 * Rotate the robot in the direction given by `dir` and the angle
 * given by `angle`
 */
static void rotate(uint8_t dir, int angle)
{
	motor_ctrl_set_state(STATE_POSITION);
	motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
	motor_ctrl_goto_position(dir == ROTATE_RIGHT ? angle : 0,
		dir == ROTATE_LEFT ? angle : 0); 
}

/**
 *
 *
 */
static void straight_forward(uint8_t speed)
{
	motor_ctrl_set_state(STATE_STRAIGHT);
	motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
	motor_ctrl_set_speed(speed, speed);
}

/** 
 * Extract the line
 */
static void extract_line()
{
	extract_slice(buffer, 0, 44, 0);
	extract_slice(buffer, 44, 88, 0);
	extract_slice(buffer, 88, 144, 0);
	extract_slice(buffer, 144, 192, 0);
	extract_slice(buffer, 192, 240, 0);
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
	unsigned int count;
	slice_t lower, upper;

	count = 0;
	ptr = (unsigned char *) frame;

	// Copy to working buffer
	for (i = 0; i < IMG_SIZE; i++)
	{
		buffer[i] = (*ptr);
		ptr += 2;
	}

	if (current_state != CALIBRATE)
	{
		// Extract line
		extract_line();

		// Calculate center of mass at the upper half of the image.
		// (this is where the line is farest away)
		calculate_center_of_mass(buffer, &upper, slice_upper_start, slice_upper_end);

		// Calculate center of mass at the lower half of the image
		calculate_center_of_mass(buffer, &lower, slice_lower_start, slice_lower_end);

		// Aggregated mass of line
		count = upper.mass + lower.mass;
	}

	count = avg_num_add(&avg_mass, count);

	// Transmit every 4rd frame over sockets.
	// This is 15 frames per second when we are capturing
	// 60 frames per second from the camera.
	if (frame_counter % 3 == 0)
	{
		//printf("%d %d -- %d %d\n", lower.x, lower.y, upper.x, upper.y);
		broadcast_send(lower.x, lower.y, upper.x, upper.y, lower.error, 
			upper.error, avg_mass.avg, buffer);
	}

	frame_counter++;

	// Dispatch the updating to another function
	update_loop(avg_mass.avg, &upper, &lower);
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
	int speed_r, speed_l;

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

	I = I_sum * Ki;
	D = Kd * (err - last_error);
	
	// Total correction
	correction = P + I + D;
	last_error = err;

	// Limit speed if the line has big changes in direction 
	// in the future
	err_diff = abs(lower->error - upper->error) * k_error_diff;

	// Calculate new speed
	speed_l = (int) round(speed - err_diff - correction);
	speed_r = (int) round(speed - err_diff + correction);

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
	log_add(logs, entry);	
}

/**
 *
 *
 */
static void goto_wall()
{
	uint8_t front;

	dist_enable(DIST_SENSOR_FRONT);
	avg_num_clear(&avg_front_dist);

	// Go straight till we see the wall
	motor_ctrl_set_state(STATE_STRAIGHT);
	motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
	motor_ctrl_set_speed(speed_ref_slow, speed_ref_slow);

	// Continue while measuring the distance to the wall
	while (1)
	{
		dist_read(&front, NULL);
		avg_num_add(&avg_front_dist, front);

		if (avg_front_dist.avg > 125)
		{
			// Brake and disable the front distance sensor
			motor_ctrl_brake();
			beep();

			printf("Found wall. Braking and disabling the distance sensor\n");
			break;
		}

		usleep(5000);
	}

	dist_enable(DIST_SENSOR_NONE);
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

		case START:
		{
			beep_start_seq();

			printf("Starting!\n");
			motor_ctrl_set_state(STATE_STRAIGHT);
			motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
			motor_ctrl_set_speed(speed_ref_slow, speed_ref_slow);

			current_state = GOTO_LINE;
			break;
		}
		/*
		 *
		 * Go straight till we see the line and then make a left-turn.
		 *
		 */
		case GOTO_LINE:
		{
			if (mass > mass_horizontal_lower && mass < mass_horizontal_upper)
			{
				beep();
				printf("Found the line (%d)\n", mass);
				
				motor_ctrl_set_state(STATE_POSITION);
				motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
				motor_ctrl_goto_position(0, P_45);

				sleep(1);

				motor_ctrl_set_state(STATE_SPEED);

				beep_state_change();
				current_state = FOLLOW_LINE;
				printf("Following the line\n");
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
			if (mass > mass_cross_lower && mass < mass_cross_upper)
			{
				printf("Found the crossing! (%d)\n", mass);

				// Brake and change state
				beep_state_change();
				current_state = GOTO_WALL;
				motor_ctrl_brake();
				sleep(2);
			}

			break;
		}

		/**
		 *
		 *
		 */
		case GOTO_WALL:
		{
			uint8_t dist_front, dist_side;
			double angle;

			// Clear the averager and enable distance measurements
			avg_num_clear(&avg_front_dist);
			dist_enable(DIST_SENSOR_FRONT);
				
			// TODO Measure angle to line
			angle = angle_to_line(upper, lower);
			printf("Angle to line: %f\n", angle);

			// Turn left
			motor_ctrl_set_state(STATE_POSITION);
			motor_ctrl_goto_position(0, P_90);
			delay(2000);

			goto_wall();
			delay(1500);

			// Rotate 135 degress
			motor_ctrl_set_state(STATE_POSITION);
			motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
			motor_ctrl_goto_position(P_135, 0);

			sleep(3);

			printf("Rotated 135deg towards the line\n");

			motor_ctrl_set_state(STATE_STRAIGHT);
			motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
			motor_ctrl_set_speed(speed_ref_slow, speed_ref_slow);
			
			printf("Driving towards the line\n");

			// Wait 60 frames in the next state before taking any action.
			// This is done for the camera to settle and provide accurate
			// and stable images.
			settling_set(60);

			beep();
			avg_num_clear(&avg_mass);
			current_state = FROM_WALL_TO_LINE;

			printf("Going into camera mode again (from wall to line state)\n");

			break;
		}

		/**
		 *
		 *
		 *
		 */
		case FROM_WALL_TO_LINE:
		{
			settling_check();

			printf("mass %d\n", mass);
			if (mass > 15000 && mass < 33000)
			{
				printf("Found the line (%d)\n", mass);

				// Brake and wait
				motor_ctrl_brake(); //// <---- TODO!!
				delay(1000);

				// Speed motor state, next state, settling delay
				motor_ctrl_set_state(STATE_SPEED);
				current_state = FOLLOW_LINE_AFTER_WALL;
				settling_set(100);

				// Signal change
				beep_state_change();
				printf("State: Follow line after wall\n");
			}
			break;
		}
		/*
		 *
		 * Follow line after wall
		 *
		 */
		case FOLLOW_LINE_AFTER_WALL:
		{
			pid_controller(mass, upper, lower, speed_ref, k_error, k_p, k_i, k_d);

			// Skip rest of state if the settling time hasn't expired
			settling_check();

			if (mass > 20000)
			{
				printf("State: Follow line speedy\n");

				settling_set(30);
				current_state = FOLLOW_LINE_SPEEDY;
			}
			
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

			settling_check();
			if (mass > 20000)
			{
				// Beep, brake, wait
				beep_state_change();
				motor_ctrl_brake();
				sleep(1);

				// Next state
				beep_state_change();
				//current_state = END_OF_LINE;
				current_state = WAITING;
			}
			break;
		}

		case END_OF_LINE:
		{
			current_state = STICK_TO_WALL;
			break;
		}

		case STICK_TO_WALL:
		{
			/*
			goto_wall();
			delay(500);

			rotate(ROTATE_RIGHT, P_90);
			delay(1500);

			stick_to_wall(0);
			delay(1000);

			rotate(ROTATE_LEFT, P_90);
			delay(1500);

			stick_to_wall(1);
			*/


			break;
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

	//thr_enable = config_get_int("thr_enable");
	//thr_upper = config_get_int("thr_upper");
	//thr_lower = config_get_int("thr_lower");

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
static void * led_thread_fn(void * ptr)
{

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
			 * Command selection
			 */

			if (strcmp(buffer, "st") == 0)
			{	
				reset();
				current_state = START;
			}
			else if (strcmp(buffer, "st2") == 0)
			{	
				reset();
				motor_ctrl_set_state(STATE_SPEED);
				motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);

				current_state = FOLLOW_LINE;
			}
			else if (strcmp(buffer, "st3") == 0)
			{	
				reset();
				motor_ctrl_set_state(STATE_SPEED);
				motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);

				current_state = FOLLOW_LINE_AFTER_WALL;
			}
			/**
			 * Stop the robot and return to waiting state.
			 */
			else if (strcmp(buffer, "s") == 0)
			{
				motor_ctrl_brake();
				current_state = WAITING;
			}
			/**
			 * Reload the configuration file.
			 */
			else if (strcmp(buffer, "r") == 0)
			{
				load_config();
				printf("Constants reloaded\n");
			}
			/**
			 * Go to calibration state (no image processing)
			 */
			else if (strcmp(buffer, "startcal") == 0)
			{
				current_state = CALIBRATE;
			}
			/**
			 * Stop calibration mode by returning to waiting state.
			 */
			else if (strcmp(buffer, "stopcal") == 0)
			{
				current_state = WAITING;
			}
			/**
			 * Stop the camera loop and exit the program.
			 */
			else if (strcmp(buffer, "exit") == 0)
			{
				cam_end_loop(cam);
				pthread_exit(0);
			}
			/**
			 * Dump the current image buffer to a file (PGM-format / P2)
			 */
			else if (strcmp(buffer, "dump") == 0)
			{
				unsigned char buf[IMG_SIZE];
				char filename[40];
				time_t timer;

				time(&timer);
				memcpy(buf, buffer, IMG_SIZE);
				sprintf(filename, "img-%d.pgm", (int) timer);
				dump_to_pgm(buf, filename);
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
	cam = malloc(sizeof(struct camera));
	// Camera configuration
	cam->config.frame_cb = frame_callback;
	cam->config.width = WIDTH;
	cam->config.height = HEIGHT;
	cam->config.fps = config_get_int("fps");
	cam->dev = config_get_str("device");
}

static void print_welcome_msg()
{
	printf("\n=== Eyebot - The Line Follower ===\n");
	printf("Config: w: %d, h: %d, fps (expected): %d\n", cam->config.width, 
		cam->config.height, cam->config.fps);
}

/**
 * Main entry point.
 */
int main(int argc, char ** argv)
{
	pthread_t processing_thread, shell_thread, led_thread;
	struct addrinfo hints, *res;

	// Initialize variables
	frame_counter = 0;
	current_state = WAITING;
	buffer = malloc(IMG_SIZE);

	// Init and load the configuration file
	config_init();
	load_config();

	// Allocate and initialize the logging system
	logs = log_create();

	// Allocate average number variables
	avg_num_create(&avg_mass, 3);
	avg_num_create(&avg_front_dist, 1);
	avg_num_create(&avg_side_dist, 1);

	// Setup camera with fps, size etc. from configuration
	setup_camera();	

	// Open i2c bus (by internally opening the i2c device driver)
	if (i2c_bus_open() < 0)
	{
		printf("Failed opening I2C bus, exiting...\n");
		exit(-1);
	}

	// Initialize the MCP23016 io-expander
	ioexp_init();

	// Initialize the motor controller
	motor_ctrl_init();

	// Print a nice welcome message
	print_welcome_msg();

	// Catch CTRL-C signal and end looping
	signal(SIGINT, sigint_handler);

	// Open camera and start capturing
	cam_init(cam);
	cam_start_capturing(cam);
	
	// Open TCP server socket, and start listening for connections	
	broadcast_init();
	broadcast_start();

	// Reset all counters and stuff
	reset();

	// Beep once to indicate the robot is ready
	beep();

	//
	// Ready! 
	//

	// Create the main processing thread (camera and update loop)
	pthread_create(&processing_thread, NULL, processing_thread_fn, NULL);

	// Create the shell thread
	pthread_create(&shell_thread, NULL, shell_thread_fn, NULL);
	
	// Wait for the main thread and shell thread to finish
	pthread_join(processing_thread, NULL);
	pthread_join(shell_thread, NULL);	
	
	//
	// Shutting down...
	//

	cam_stop_capturing(cam);
	cam_uninit(cam);

	// Stop robot
	motor_ctrl_brake();

	// Close server socket and drop connections
	broadcast_release();

	// Close the I2C bus
	i2c_bus_close();

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


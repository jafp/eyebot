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
#include "pid.h"

#define delay(ms) 				(usleep(ms * 1000))

#define T 						0.03333

#define P_45					256
#define P_90					(2 * P_45)
#define P_135					(3 * P_45)
#define P_180					(4 * P_45)

#define SPEED_LIMIT				100

#define ROTATE_LEFT				1
#define ROTATE_RIGHT			2

/**
 * Function prototypes
 */
static int update_loop(int mass, slice_t * upper, slice_t * lower);
static void motor_set_control_value(pid_data_t * pid, float cv);
static unsigned char get_limited_speed(int speed);


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
	GOTO_WALL,
	FROM_WALL_TO_LINE,
	END_OF_LINE,
	STICK_TO_WALL,
	STRAIGHT_UNTIL_WALL_DISAPPEARS,
	STRAIGHT_UNTIL_WALL_DISAPPEARS_2,
	FOLLOW_WALL_1,
	FOLLOW_WALL_2,
	TRACK_COMPLETED
} state_t;

typedef enum {
	OFF,
	BLINK,
	BLINK_FAST,
	KNIGHT_NIDER,
	SHUTDOWN
} led_state_t;


typedef struct {
	int speed_start;
	int speed_slow;
	int speed_normal;
	int speed_fast;
	int speed_faster;
} speeds_t;

typedef struct {
	int lower;
	int upper;
} mass_limits_t;

/**
 *
 * ---------------------------------------------------------------------------
 * Global variables
 * ---------------------------------------------------------------------------
 *
 */

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
static unsigned char * buffer_copy;

/**
 * Frame counter
 */
static unsigned long frame_counter;

/**
 * Current state.
 */
static state_t current_state = WAITING;

/**
 * Current state of the bumper LEDs
 */
static led_state_t led_current_state = OFF;

//static int blink_leds = 1;

static float last_error = 0;
static int I_sum = 0;

static avg_num_t avg_mass;
static avg_num_t avg_front_dist;
static avg_num_t avg_side_dist;

static int settling_cnt = 0;
static int settling_en = 0;

static pthread_mutex_t buffer_mutex;


/**
 * Data for the PID controller used when following the line
 * using the camera.
 */
static pid_data_t line_pid = {
	.set_cv = motor_set_control_value
};

// Speed when following the wall
//static int g_wall_speed;
//static int g_wall_max_err;
//static int g_wall_diff_p;

// Controller settings when following the wall
static pid_data_t wall_pid = {
	.set_cv = motor_set_control_value
};

/**
 *
 * ---------------------------------------------------------------------------
 * Functions
 * ---------------------------------------------------------------------------
 *
 */

#define settling_check() 								\
	if (settling_en && settling_cnt-- > 0) { return; } 	\
	else { settling_en = 0; }							\

#define settling_set(frames)							\
	settling_en = 1; settling_cnt = frames				\

/**
 * Convert number from distance sensor, to a distance in centimeters.
 * Approximation calculated using Excel and a bunch of measurements.
 *
 * \param dist
 */
static float get_dist_to_cm(unsigned char dist)
{
	return 1051.43 * powf(dist, -0.944);
}

static float get_angle_to_pulses(float angle)
{
	return angle * ((float) (512/90));
}

/**
 * Calculate side distance process value for PID calculation.
 *
 * \param side_front
 * \param side_rear
 */
static float get_side_dist_pv(float side_front, float side_rear)
{
	return (conf.w_setpoint - side_front) + 
		(side_rear - side_front) * conf.w_diff_p;
}


static void motor_set_control_value(pid_data_t * pid, float cv)
{
	// Speed (set point) for left and right motor
	uint8_t speed_left, speed_right;
	float lcv;

	if (cv > conf.w_max_error) { lcv = conf.w_max_error; }
	else if (cv < -conf.w_max_error) { lcv = -conf.w_max_error; }
	else { lcv = cv; }

	speed_left = get_limited_speed(conf.w_speed + lcv);
	speed_right = get_limited_speed(conf.w_speed - lcv);

	//printf("cv: %4.1f, lcv: %4.1f, l: %4d, r: %4d s: %4d\n", cv, lcv, speed_left, speed_right, g_wall_speed);
	motor_ctrl_set_speed(speed_left, speed_right);
}

/**
 * Read distance from the sensor placed in the front.
 */
static float get_dist_front()
{
	uint8_t dist;
	dist_read(&dist, NULL, NULL);
	return get_dist_to_cm(dist);
}

/**
 * Read distance from the side sensor placed in the front.
 */
static float get_dist_side_front()
{
	uint8_t dist;
	dist_read(NULL, &dist, NULL);
	return get_dist_to_cm(dist);
}

/**
 * Read distance from the side sensor placed in the rear.
 */
static float get_dist_side_rear()
{
	uint8_t dist;
	dist_read(NULL, NULL, &dist);
	return get_dist_to_cm(dist);	
}

/**
 * Limits the speed according to the limit specified in the
 * SPEED_LIMIT constant.
 *
 * \param speed The speed to limit
 * \return The limited speed
 */
static unsigned char get_limited_speed(int speed)
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
	motor_ctrl_wait(200);
}

static void goto_speed_mode()
{
	motor_ctrl_set_state(STATE_SPEED);
	motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
	motor_ctrl_set_speed(0, 0);
}

/**
 *
 *
 */
static void straight_forward()
{
	motor_ctrl_set_state(STATE_STRAIGHT);
	motor_ctrl_set_dir(DIR_LEFT_FORWARD | DIR_RIGHT_FORWARD);
	motor_ctrl_set_speed(conf.speed_straight+3, conf.speed_straight);
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

	// Get mutual access to buffer
	pthread_mutex_lock(&buffer_mutex);

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
		calculate_center_of_mass(buffer, &upper, conf.slice_upper_start, 
			conf.slice_upper_end);

		// Calculate center of mass at the lower half of the image
		calculate_center_of_mass(buffer, &lower, conf.slice_lower_start, 
			conf.slice_lower_end);

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

	// Create copy for dumping later
	memcpy(buffer_copy, buffer, IMG_SIZE);

	// Release mutex
	pthread_mutex_unlock(&buffer_mutex);

	frame_counter++;

	// Dispatch the updating to another function
	update_loop(avg_mass.avg, &upper, &lower);
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
static void pid_controller(int mass, slice_t * upper, slice_t * lower, 
	int speed, float Kerr, float Kp, float Ki, float Kd)
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
	err_diff = abs(lower->error - upper->error) * conf.k_error_diff;

	// Calculate new speed
	speed_l = (int) round(speed - err_diff - correction);
	speed_r = (int) round(speed - err_diff + correction);

	// Send new speeds to motor controller
	// (Each speed is limited to the interval 0-255 (unsigned 8-bit number))
	motor_ctrl_set_speed(get_limited_speed(speed_l), get_limited_speed(speed_r));

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
 * Drives forward until the wall is in the given distance
 * from the front.
 *
 * \param limit_lower
 * \param limit_upper
 */
static void goto_wall(int limit_lower, int limit_upper)
{
	uint8_t front;
	float dist_in_cm;

	// Clear averager
	avg_num_clear(&avg_front_dist);

	// Go straight till we see the wall
	straight_forward();

	// Continue while measuring the distance to the wall
	while (1)
	{
		// Read, average, and convert to cm
		dist_read(&front, NULL, NULL);
		avg_num_add(&avg_front_dist, front);
		dist_in_cm = get_dist_to_cm(avg_front_dist.avg);

		if (dist_in_cm > limit_lower && dist_in_cm < limit_upper)
		{
			// Brake and disable the front distance sensor
			motor_ctrl_brake();
			motor_ctrl_wait(500);
			beep();

			printf("Found wall (%.2f). Braking and disabling the distance sensor\n", 
				dist_in_cm);
			break;
		}
		delay(33);
	}
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

			straight_forward();
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
			printf("mass: %d\n", mass);
			if (mass > conf.mass_horizontal_lower && 
				mass < conf.mass_horizontal_upper 
				)
			{
				beep();
				printf("Found the line (%d)\n", mass);
				
				rotate(ROTATE_LEFT, P_45);
				goto_speed_mode();

				beep_state_change();
				led_current_state = KNIGHT_NIDER;
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
			pid_controller(mass, upper, lower, conf.speed_slow, conf.k_error, 
				conf.k_p, conf.k_i, conf.k_d);

			if (mass > conf.mass_cross_lower && mass < conf.mass_cross_upper)
			{
				printf("Found the crossing! (%d)\n", mass);
				beep();

				motor_ctrl_brake();
				motor_ctrl_wait(1000);

				// Brake and change state
				beep_state_change();
				led_current_state = OFF;
				current_state = GOTO_WALL;
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
			//dist_enable(DIST_SENSOR_FRONT);
				
			// TODO Measure angle to line
			angle = angle_to_line(upper, lower);
			printf("Angle to line: %f\n", angle);

			// Turn left
			rotate(ROTATE_LEFT, P_90);

			// Goto wall
			goto_wall(18, 22);

			// Rotate 135 degress
			rotate(ROTATE_RIGHT, P_135);
			printf("Rotated 135deg towards the line\n");

			straight_forward();
			printf("Driving towards the line\n");

			// Wait 60 frames in the next state before taking any action.
			// This is done for the camera to settle and provide accurate
			// and stable images.
			settling_set(100);

			beep();
			avg_num_clear(&avg_mass);
			led_current_state = BLINK_FAST;
			current_state = FROM_WALL_TO_LINE;
			printf("State: From wall to line\n");

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

			printf("Mass %d\n", mass);
			if (mass > 10000 && mass < 33000)
			{
				printf("Found the line (%d)\n", mass);

				// Speed motor state, next state, settling delay
				goto_speed_mode();
				settling_set(100);
				led_current_state = OFF;
				//led_current_state = KNIGHT_NIDER;
				current_state = FOLLOW_LINE_AFTER_WALL;
				
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
			pid_controller(mass, upper, lower, conf.speed_normal, conf.k_error
				, conf.k_p, conf.k_i, conf.k_d);

			// Skip rest of state if the settling time hasn't expired
			settling_check();

			if (mass > 20000)
			{
				printf("State: Follow line speedy\n");

				beep();
				settling_set(30);
				led_current_state = OFF;
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
			pid_controller(mass, upper, lower, conf.speed_fast, conf.k_error, 
				conf.k_p, conf.k_i, conf.k_d);

			settling_check();
			if (mass > 23000)
			{
				printf("Found end (%d)\n", mass);

				// Beep, brake, wait
				beep_state_change();
				motor_ctrl_brake();
				motor_ctrl_wait(100);

				// Next state
				beep_state_change();
				current_state = END_OF_LINE;
			}
			break;
		}

		case END_OF_LINE:
		{
			// Wait for three seconds as described in the 
			// requirements specification
			delay(3000);
			current_state = STICK_TO_WALL;
			break;
		}

		case STICK_TO_WALL:
		{
			// Goto wall and stop between 20 and 24 cm before the wall
			goto_wall(18, 22);
			printf("Found wall.\n");

			// Rotate
			rotate(ROTATE_RIGHT, P_90);
			//motor_ctrl_wait(200);
			printf("Turned right.\n");

			// Go forward
			straight_forward();

			// Next state
			beep();
			printf("State: straight until wall disappears");
			current_state = STRAIGHT_UNTIL_WALL_DISAPPEARS;

			break;
		}

		case STRAIGHT_UNTIL_WALL_DISAPPEARS:
		{
			float dist_side;
			dist_side = get_dist_side_rear();

			if (dist_side > 14)
			{
				motor_ctrl_brake();
				motor_ctrl_wait(200);
				printf("Wall disappeared.\n");

				rotate(ROTATE_LEFT, P_90);
				//motor_ctrl_wait(200);
				printf("Rotated left.\n");

				straight_forward();
				// TODO!!!
				delay(2000); 

				// Next state
				beep();
				printf("State: straight until wall disappears 2");
				current_state = STRAIGHT_UNTIL_WALL_DISAPPEARS_2;
			}

			break;
		}

		case STRAIGHT_UNTIL_WALL_DISAPPEARS_2:
		{
			float dist;
			dist = get_dist_side_rear();

			if (dist > 20)
			{
				motor_ctrl_brake();
				motor_ctrl_wait(200);
				printf("Wall disappeared 2.\n");

				rotate(ROTATE_LEFT, P_90);
				printf("Rotated left.\n");

				straight_forward();
				delay(1500);

				pid_reset(&wall_pid);
				goto_speed_mode();

				beep();
				printf("State: Follow wall 1\n.");
				current_state = FOLLOW_WALL_1;
			}

			break;
		}


		case FOLLOW_WALL_1:
		{
			uint8_t dist_1, dist_2, dist_f;
			float d_f, d_1, d_2, pv;

			dist_read(&dist_f, &dist_1, &dist_2);
			d_1 = get_dist_to_cm(dist_1);
			d_2 = get_dist_to_cm(dist_2);
			d_f = get_dist_to_cm(dist_f);

			if (d_f > 18 && d_f < 24)
			{
				motor_ctrl_brake();
				motor_ctrl_wait(200);
				
				rotate(ROTATE_RIGHT, P_90);
				//motor_ctrl_wait(200);

				goto_speed_mode();

				beep();
				printf("State: Follow wall 2\n");
				current_state = FOLLOW_WALL_2;
			}
				
			//d_err = (d2 - d1) + d1;
			//d_err = 2 * d1 + d2;	

			// error = dist error + direction error
			//error = (g_wall_pid.set_point - d1) + (d2 - d1) * g_wall_diff_p;

			//printf("d1: %.2f - d2: %.2f - d_err: %.2f  - df: %4.2f\n", d1, d2, error, df);
			// Do PID control (using set point from config `w_setpoint`)
			// Could by changed by g_wall_pid.set_point = ...

			pv = get_side_dist_pv(d_1, d_2);
			pid_ctrl(&wall_pid, (int) pv);

			break;
		}

		case FOLLOW_WALL_2: 
		{
			uint8_t dist_1, dist_2, dist_f;
			float d_f, d_1, d_2, pv;

			dist_read(&dist_f, &dist_1, &dist_2);
			d_1 = get_dist_to_cm(dist_1);
			d_2 = get_dist_to_cm(dist_2);
			d_f = get_dist_to_cm(dist_f);

			if (d_f > 10 && d_f < 16)
			{
				motor_ctrl_brake();
				motor_ctrl_wait(200);
				
				beep();
				printf("State: Track completed\n");
				current_state = TRACK_COMPLETED;
			}
				
			//d_err = (d2 - d1) + d1;
			//d_err = 2 * d1 + d2;	

			// error = dist error + direction error
			//error = (g_wall_pid.set_point - d1) + (d2 - d1) * g_wall_diff_p;

			//printf("d1: %.2f - d2: %.2f - d_err: %.2f  - df: %4.2f\n", d1, d2, error, df);
			// Do PID control (using set point from config `w_setpoint`)
			// Could by changed by g_wall_pid.set_point = ...

			pv = get_side_dist_pv(d_1, d_2);
			pid_ctrl(&wall_pid, (int) pv);

			break;
		}

		case TRACK_COMPLETED:
		{
			beep_state_change();
			beep_start_seq();

			// Finale show!!!
			delay(1000);
			rotate(ROTATE_RIGHT, P_135);
			led_current_state = BLINK_FAST;

			int i;
			for (i = 0; i < 10; i++)
			{
				beep_medium();
				delay(20);
			}

			led_current_state = OFF;
			current_state = WAITING;

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

	wall_pid.P = conf.w_k_p;
	wall_pid.I = conf.w_k_i;
	wall_pid.D = conf.w_k_d;
	wall_pid.max_sum_error = conf.w_max_sum_error;
	wall_pid.set_point = conf.w_setpoint;

	printf("kp: %.2f, ki: %.2f, kd: %.2f\n", conf.k_p, conf.k_i, conf.k_d);
	printf("speed_straight: %d, speed_slow: %d\n", conf.speed_straight, conf.speed_slow);
	printf("mass_horizontal_upper: %d, mass_horizontal_upper: %d\n", conf.mass_horizontal_upper, conf.mass_horizontal_lower);
}


/**
 * SIGINT signal handler.
 * 
 * \param signal Signal number
 */
static void sigint_handler(int signal)
{
	//cam_end_loop(cam);
	motor_ctrl_brake();
	current_state = WAITING;
	led_current_state = OFF;
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
	uint8_t mask = 0xFF;
	uint8_t knight_mask = 0x01;
	uint8_t i;

	while (1)
	{
		switch (led_current_state)
		{
			case BLINK:
			case BLINK_FAST:
			{
				mask = ~mask;
				ioexp_led_set(mask);
				delay(led_current_state == BLINK ? 100 : 50);
				break;
			}
			case KNIGHT_NIDER:
			{
				for (i = 0; i < 7; i++)
				{
					knight_mask = (1 << i);
					ioexp_led_set(knight_mask);
					delay(100);
				}

				for (i = 5; i > 0; i--)
				{
					knight_mask = (1 << i);
					ioexp_led_set(knight_mask);
					delay(100);
				}

				break;
			}
			case SHUTDOWN:
			{
				// Turn off leds and exit
				ioexp_led_set(0);
				pthread_exit(0);
				break;
			}
			case OFF:
			default: 
			{
				ioexp_led_set(0);
				delay(100);
			}
		}
	}
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
				load_config();
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
			else if (strcmp(buffer, "st4") == 0)
			{
				reset();
				beep_state_change();
				current_state = STICK_TO_WALL;
			}
			/**
			 * Stop the robot and return to waiting state.
			 */
			else if (strcmp(buffer, "s") == 0)
			{
				motor_ctrl_brake();
				//dist_enable(0);
				current_state = WAITING;
			}
			/**
			 * Reload the configuration file.
			 */
			else if (strcmp(buffer, "r") == 0)
			{
				config_reload();
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
				led_current_state = SHUTDOWN;
				cam_end_loop(cam);
				pthread_exit(0);
			}
			/**
			 * Dump the current image buffer to a file (PGM-format / P2)
			 */
			else if (strcmp(buffer, "dump") == 0)
			{
				char filename[40];
			
				pthread_mutex_lock(&buffer_mutex);
				
				sprintf(filename, "img-%d.pgm", frame_counter);
				printf("Dumping to %s\n", filename);
				dump_to_pgm(buffer_copy, filename);

				pthread_mutex_unlock(&buffer_mutex);
			}

			else if (strcmp(buffer, "wall") == 0)
			{	
				// Enable side distance sensor
				//dist_enable(DIST_SENSOR_SIDE);

				// Reload config and reset variables
				reset();
				config_reload();
				pid_reset(&wall_pid);
				printf("kp: %f, ki: %f, kd: %f\n", wall_pid.P, wall_pid.I, 
					wall_pid.D);

				current_state = STICK_TO_WALL;
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
	buffer_copy = malloc(IMG_SIZE);

	pthread_mutex_init(&buffer_mutex, NULL);

	// Init and load the configuration file
	config_init();
	load_config();

	// Allocate and initialize the logging system
	logs = log_create();

	// Allocate average number variables
	avg_num_create(&avg_mass, 3);
	avg_num_create(&avg_front_dist, 10);
	avg_num_create(&avg_side_dist, 10);

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

	led_current_state = KNIGHT_NIDER;

	//
	// Ready! 
	//

	pthread_create(&led_thread, NULL, led_thread_fn, NULL);

	// Create the main processing thread (camera and update loop)
	pthread_create(&processing_thread, NULL, processing_thread_fn, NULL);

	// Create the shell thread
	pthread_create(&shell_thread, NULL, shell_thread_fn, NULL);

	
	// Wait for the main thread and shell thread to finish
	pthread_join(processing_thread, NULL);
	pthread_join(shell_thread, NULL);	
	pthread_join(led_thread, NULL);
	
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


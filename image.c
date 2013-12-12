
#include "image.h"
#include "common.h"

#include <math.h>
#include <stdio.h>

/**
 * Calculate the "center of mass" of the given portion of the image,
 * given as an Y-offset and Y-length.
 */
void calculate_center_of_mass(unsigned char * buffer, slice_t * pt, 
	int y_offset_start, int y_offset_end)
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

	pt->mass = 0;

	if (sum > 0)
	{
		pt->x = x / sum;
		pt->y = y / sum;
		pt->error = (WIDTH / 2) - pt->x;
		pt->mass = sum;
	}
}

/**
 * Calculates histogram.
 *
 * \param hist Pointer to float array where to put the histogram
 * \param start Start row
 * \param end End row
 */
void histogram(unsigned char * buffer, float * hist, int start, int end)
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
void optimum_thresholding(unsigned char * buffer, int start, int end, 
	int nice)
{
	int y, x, j, flag, thr;

	float sum;
	float hist[256];
	
	histogram(buffer, hist, start, end);

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

/**
 * Adjust contrast of the live image buffer.
 */
 /*
void constrast(unsigned char * buffer)
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
*/

/**
 * Calculate the robot's angle relative to the line.
 *
 * \param upper
 * \param lower
 */
double angle_to_line(slice_t * upper, slice_t * lower)
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
	return ( (x1*x2+y1*y2) / (sqrt(x1*x1+y1*y1) * sqrt(x2*x2+y2*y2)) ) * 
		(180/PI);
}

/**
 * Extract a specific part of the image
 *
 * \param start
 * \param end
 * \param nice
 */
void extract_slice(unsigned char * buffer, int start, int end, int nice)
{
	optimum_thresholding(buffer, start, end, nice);
}



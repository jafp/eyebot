
#ifndef _IMAGE_H_
#define _IMAGE_H_

/**
 * Information about a `slice` of the image, 
 * including mass of the line, error and so on.
 */
typedef struct slice {
	int x, y, mass, error;
} slice_t;


void calculate_center_of_mass(unsigned char * buffer, slice_t * pt, 
	int y_offset_start, int y_offset_end);

void histogram(unsigned char * buffer, float * hist, int start, int end);

void optimum_thresholding(unsigned char * buffer,int start, int end, int nice);

double angle_to_line(slice_t * upper, slice_t * lower);

void extract_slice(unsigned char * buffer, int start, int end, int nice);

void dump_to_pgm(unsigned char * buffer, const char * file);

#endif


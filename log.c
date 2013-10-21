
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "log.h"

/**
 * Allocate memory for a new log list.
 *
 * \return A new log_list_t
 */
log_list_t * log_create()
{
	log_list_t * list = (log_list_t *) malloc(sizeof(log_list_t));
	assert(list != NULL);
	list->entries = 0;
	list->first = NULL;
	list->last = NULL;
	return list;
}

/**
 * Allocate memeory for a new log entry.
 * 
 * \return A pointer to the newly allocated log entry
 */
log_entry_t * log_entry_create()
{
	log_entry_t * entry = (log_entry_t *) malloc(sizeof(log_entry_t));
	memset(entry, 0, sizeof(log_entry_t));
	return entry;
}

/**
 * Add `entry` to the start of the list `log`.
 *
 * \param log The log
 * \param entry The entry that should be appended to the list
 */
void log_add(log_list_t * log, log_entry_t * entry)
{
	log_entry_t * ptr = log->last;

	if (ptr != NULL)
	{
		ptr->next = entry;
		log->last = entry;
	} 
	else
	{
		log->first = log->last = entry;
	}

	entry->next = NULL;
	log->entries++;
}


/**
 * Dump to a file.
 */
int log_dump(log_list_t * log, const char * filename)
{
	FILE * file = fopen(filename, "w");
	if (file == NULL)
	{
		return -1;
	}

	// Print header
	fprintf(file, "Time,Frame,Error (lower),Error (upper),Mass,P,I,D,\
		Speed (left),Speed (right),Speed Ref (left),Speed Ref (right),\
		Tacho (left),Tacho (right)\n");


	log_entry_t * ptr = log->first;
	while (ptr != log->last)
	{	
		fprintf(file, "%ld,%ld,%d,%d,%d,%f,%f,%f,%d,%d,%d,%d,%d,%d\n", 
			ptr->fields.time, ptr->fields.frame, ptr->fields.error_lower_x, 
			ptr->fields.error_upper_x, ptr->fields.mass, ptr->fields.P, 
			ptr->fields.I, ptr->fields.D, ptr->fields.speed_left, 
			ptr->fields.speed_right,ptr->fields.speed_ref_left, 
			ptr->fields.speed_ref_right, ptr->fields.tacho_left,
			ptr->fields.tacho_right);

		ptr = ptr->next;
	}

	fflush(file);
	fclose(file);
}
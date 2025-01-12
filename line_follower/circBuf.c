// *******************************************************
// 
// circBuf.c
//
// Support for a circular buffer of unsigned longs on the 
//  Stellaris LM3S1968 EVK
// P.J. Bones UCECE
// Last modified:  13.3.2013
// 
// *******************************************************

#include "stdlib.h"
#include "circBuf.h"

// *******************************************************
// initCircBuf: Initialise the circBuf instance. Reset both indices to
// the start of the buffer.  Dynamically allocate the memory and return
// a pointer for the data.  Returns NULL if allocation fails.
unsigned short *
initCircBuf (circBuf_t *buffer, unsigned int size)
{
	buffer->windex = 0;
	buffer->rindex = 0;
	buffer->size = size;
	buffer->data = 
        (unsigned long *) malloc (size * sizeof(unsigned long));

	short i = 0;
	for(i = 0; i < size; i++)
	{
		buffer->data[i] = 0;
	}
	return buffer->data;
}

// *******************************************************
// writeCircBuf: insert entry at the current windex location,
// advance windex.
void
writeCircBuf (circBuf_t *buffer, unsigned short entry)
{
	buffer->data[buffer->windex] = entry;
	buffer->windex++;
	if (buffer->windex >= buffer->size)
	   buffer->windex = 0;
}

// *******************************************************
// readCircBuf: return entry at the current rindex location,
// advance rindex. No checking for overrun.
unsigned short
readCircBuf (circBuf_t *buffer)
{
	unsigned short entry;
	
	entry = buffer->data[buffer->rindex];
	buffer->rindex++;
	if (buffer->rindex >= buffer->size)
	   buffer->rindex = 0;
    return entry;
}

// *******************************************************
// freeCircBuf: Releases the memory allocated to the buffer data,
// sets pointer to NULL and ohter fields to 0. The buffer can
// re initialised by another call to initCircBuf().
void
freeCircBuf (circBuf_t * buffer)
{
	buffer->windex = 0;
	buffer->rindex = 0;
	buffer->size = 0;
	free (buffer->data);
	buffer->data = NULL;
}


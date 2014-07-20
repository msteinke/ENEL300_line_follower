#ifndef CIRCBUF_H_
#define CIRCBUF_H_

// *******************************************************
// 
// circBuf.h
//
// Support for a circular buffer of unsigned longs on the 
//  Stellaris LM3S1968 EVK
// P.J. Bones UCECE
// Last modified:  13.3.2013
// 
// *******************************************************

// *******************************************************
// Buffer structure
typedef struct {
	unsigned int size;	// Number of entries in buffer
	unsigned int windex;	// index for writing, mod(size)
	unsigned int rindex;	// index for reading, mod(size)
	unsigned short *data;	// pointer to the data
} circBuf_t;

// *******************************************************
// initCircBuf: Initialise the circBuf instance. Reset both indices to
// the start of the buffer.  Dynamically allocate the memory and return
// a pointer for the data.  Returns NULL if allocation fails.
unsigned short *
initCircBuf (circBuf_t *buffer, unsigned int size);

// *******************************************************
// writeCircBuf: insert entry at the current windex location,
// advance windex.
void
writeCircBuf (circBuf_t *buffer, unsigned short entry);

// *******************************************************
// readCircBuf: return entry at the current rindex location,
// advance rindex. No checking for overrun.
unsigned short
readCircBuf (circBuf_t *buffer);

// *******************************************************
// freeCircBuf: Releases the memory allocated to the buffer data,
// sets pointer to NULL and ohter fields to 0. The buffer can
// re initialised by another call to initCircBuf().
void
freeCircBuf (circBuf_t * buffer);

#endif /*CIRCBUF_H_*/

/*
 * cfifo.h - a specialized fifo implementation for variable data records
 *
 *  Created on: Jun 28, 2022
 *      Author: Guillermo
 */

#ifndef SOURCE_CFIFO_H_
#define SOURCE_CFIFO_H_

#include "globaldefs.h"


/*-------------------------------------------------------------------- Common data structures  */
typedef void* DBUF_HANDLE;		 // ring buffer handle

typedef struct
{
	uint32_t  Count;
	uint8_t * Buffer;
} dbuf_t;

/*------------------------------------------------------------- A general purpose fix size ring buffer */

DBUF_HANDLE CreateRingBuffer(size_t slot_size, int slot_count, uint8_t *pbuf, size_t buf_size);
void 		PurgeRingBuffer(DBUF_HANDLE db);
void 		DestroyRingBuffer(DBUF_HANDLE rb);
int 		GetRingBufferCount(DBUF_HANDLE rb);
bool 		PushRingBufferData(DBUF_HANDLE db, void* pdata, int data_size); // will not store if all slots are full
void* 		StoreToNextSlot(DBUF_HANDLE db, void* pdata, int data_size);    // revolver without capacity check!
void* 		PopRigngBufferData(DBUF_HANDLE rb);

/*-------------------------------------------------- macros to calculate buffer space required */
#define _HEADER_RESERVE_ (8 * sizeof(uint32_t))
#define RING_BUFFER_SPACE(size,count)  (_HEADER_RESERVE_ + (size*count))


/*-------------------------------------------------- A variable size ring buffer (no overrun check!)  */
typedef struct
{
	uint16_t Size;
	uint8_t*  Data;

} ring_data_t;

typedef struct
{
	uint16_t Count;     // number of records in the ring buffer
    size_t 	 Size;      // total size of the buffer
    uint8_t* Head;      // read pointer index
    uint8_t* Tail;      // write pointer index
    uint8_t* Prev;      // previous record pointer before tail
    uint8_t* Ring;      // the underlying byte array
	uint8_t* End;       // pointer to the end of the bufferr

} frb_header_t, *RDBUF_HANDLE;

/* inline to calculate required buffer memory size */
#define FLEX_RING_BUF_MEM(record_size,  nrecords)	 (sizeof(frb_header_t) + (sizeof(ring_data_t) + record_size) * nrecords)

RDBUF_HANDLE CreateFlexRingBuffer(uint8_t * pbuf, size_t bufsize);
void FlushFlexRingBuffer(RDBUF_HANDLE db);

bool PushToFlexRingBuffer(RDBUF_HANDLE rb, const void* data, uint16_t data_len);

bool PopFromFlexRingBuffer(frb_header_t* rb, ring_data_t* pdata);

//ring_data_t* StoreFlexRingData(RDBUF_HANDLE db, void* pdata, size_t data_size);

bool FlexBufferWillRoll(RDBUF_HANDLE db,  uint16_t data_size);

int FlexBufferFreeSpace(RDBUF_HANDLE rb);

inline float GetFlexBufferFreePct(RDBUF_HANDLE rb )
{
	return  (100.0F * ((float)(FlexBufferFreeSpace(rb))/ (float)rb->Size));
}

inline uint16_t FlexRingBufferItems(RDBUF_HANDLE phdr)
{
	return (phdr)? phdr->Count : 0;
}

inline bool IsFlexRingBufferEmpty(RDBUF_HANDLE phdr)
{
	return ( phdr != NULL)? phdr->Count == 0: false;
}

#endif /* SOURCE_CFIFO_H_ */

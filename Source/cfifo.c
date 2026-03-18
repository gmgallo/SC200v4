/*
 * cfifo.c - a specialized implementation of ring and fifo buffues of fix and variable length
 *
 *  Created on: Jun 28, 2022
 *      Author: Guillermo
 */

#include "common.h"

/***************************************************************************** A fix slot size ring buffer */

typedef struct
{
	int Slots;
	int SlotSize;
	int Used;
	int Nextfree;
	int Nextused;

	uint8_t* NextPtr;
	uint8_t* Ring;

} rb_header_t;

DBUF_HANDLE CreateRingBuffer(size_t slotsize, int slots, uint8_t * pbuf, size_t bufsize)
{
	/* calculate total size of memory required */
	size_t stotal = sizeof(rb_header_t) +				// the header struct
				 slotsize * slots;						// the total buffer space needed

	if (bufsize < stotal)
	{
		slots = (bufsize - sizeof(rb_header_t)) / slotsize;
	}

	rb_header_t* pheader = (rb_header_t*)pbuf;

	if (pheader != NULL)
	{
		pheader->Ring = (uint8_t*)pheader + sizeof(rb_header_t);	// offset to first buffer structure
		pheader->SlotSize = slotsize;
		pheader->Slots = slots;
		pheader->Used = 0;
		pheader->Nextfree = 0;
		pheader->Nextused = 0;
	}

	return pheader;
}

void PurgeRingBuffer(DBUF_HANDLE db)
{
	if( db != NULL)
	{
		uint32_t state = cyhal_system_critical_section_enter();

		rb_header_t* pheader = (rb_header_t*)db;
		pheader->Used = 0;
		pheader->Nextfree = 0;
		pheader->Nextused = 0;

		cyhal_system_critical_section_exit(state);
	}
}

void DestroyRingBuffer(DBUF_HANDLE db)
{
	free(db);
}

int GetRingBufferCount(DBUF_HANDLE db)
{
	return (db != NULL? ((rb_header_t *)db)->Used: 0);
}

void* PopRigngBufferData(DBUF_HANDLE db)
{
	void* pdata = NULL;
	if( db != NULL)
	{
		rb_header_t* phdr = (rb_header_t*)db;

		if (phdr->Used > 0 )
		{
			uint32_t state = cyhal_system_critical_section_enter();

			pdata = phdr->Ring + phdr->Nextused * phdr->SlotSize;

			phdr->Used--;
			phdr->Nextused++;

			if ( phdr->Nextused == phdr->Slots)
				 phdr->Nextused = 0;

		cyhal_system_critical_section_exit(state);
		}
	}

	return pdata;
}

bool PushRingBufferData(DBUF_HANDLE db, void* pdata, int data_size)
{
	bool done = false;

	if( db != NULL)
	{
		rb_header_t* phdr = (rb_header_t*)db;

		uint32_t state = cyhal_system_critical_section_enter();

		if (phdr->Used < phdr->Slots )
		{
			uint8_t * slot = phdr->Ring + phdr->Nextfree * phdr->SlotSize;

		/* warning: user responsible to provide data no larger than slot size ! */

			if ( data_size > phdr->SlotSize)
				data_size = phdr->SlotSize;

			memcpy( slot, pdata, data_size );

			phdr->Used++;
			phdr->Nextfree++;

			if ( phdr->Nextfree == phdr->Slots )
				phdr->Nextfree = 0;

			done = true;
		}
		cyhal_system_critical_section_exit(state);
	}
	return done;
}

/*-------------------------------------------------------------------------
 * StoreToNextSlot() - is a free running revolver
 */
void * StoreToNextSlot(DBUF_HANDLE db, void* pdata, int data_size)
{
	uint8_t* slot = NULL;

	if( db != NULL)
	{
		rb_header_t* phdr = (rb_header_t*)db;

		uint32_t state = cyhal_system_critical_section_enter();

		slot = phdr->Ring + phdr->Nextfree * phdr->SlotSize;

		if ( data_size > phdr->SlotSize )
			data_size = phdr->SlotSize;

		memcpy( slot, pdata, data_size );

		phdr->Nextfree++;

		if ( phdr->Nextfree == phdr->Slots )
			phdr->Nextfree = 0;

		cyhal_system_critical_section_exit(state);
	}

	return slot;
}

/******************************************************************* A Ring Buffer of Flexible records size */
typedef struct
{
    uint8_t* Next;
    uint16_t Size;

} record_hdr_t;

#define RING_RECORD_SIZE(data_size) (sizeof(ring_ptr_t)+(sizeof(uint8_t)*data_size)) //  2 extra Data bytes in ring_ptr_t

int FlexBufferFreeSpace(RDBUF_HANDLE rb)
{
	int bfree = 0;

	if (rb != NULL)
	{
	   if (rb->Head == rb->Tail)
			bfree = rb->Size; // If empty, we can use all but one byte to distinguish full/empty state.

		else if (rb->Tail > rb->Head)
		   bfree = (rb->Size + (rb->Head - rb->Tail));
	   else
		   bfree = (rb->Head - rb->Tail);
	}
	return bfree;
}



RDBUF_HANDLE CreateFlexRingBuffer(uint8_t* pbuf, size_t bufsize) // use static buffers for safety
{
	frb_header_t* phdr = (frb_header_t*)pbuf;

	if (phdr != NULL)
	{
		phdr->Ring = pbuf + sizeof(frb_header_t);			// offset to first buffer structure
		phdr->Size = bufsize - sizeof(frb_header_t); 		// reserve space at the end for Next pointer.
		FlushFlexRingBuffer(phdr);
	}

	return phdr;
}

void FlushFlexRingBuffer(RDBUF_HANDLE rb)
{
	if (rb != NULL)
	{
		uint32_t state = cyhal_system_critical_section_enter();

		memset(rb->Ring, 0, rb->Size ); // include space of next ptr at the end
		rb->End = rb->Ring + rb->Size; 	// Pointer to the end of the buffer
		rb->Head = rb->Ring; 			// Initialize head to the start of the buffer
		rb->Tail = rb->Ring; 			// Initialize tail to the start of the buffer
		rb->Prev = rb->Ring; 			// Initialize previous record pointer to the start of the buffer
		rb->Count = 0; 					// Initialize record count to zero

		cyhal_system_critical_section_exit(state);
	}
}


inline bool FlexBufferWillRoll(RDBUF_HANDLE rb,  uint16_t data_size)
{

    // First, if tail is ahead of or equal to head, check the contiguous space.
    if (rb->Tail >= rb->Head)
    {
    	 uint8_t* new_tail = rb->Tail + sizeof(record_hdr_t) + data_size;

        if ( new_tail >= rb->End )
        {
        	return true;
        }
    }
    return false;  // data fits or already rolled
}



/*-------------------------------------------------------------------------
 * PushToFlexRingBuffer()
 * Push data to the tail of the buffer until capacity is full.
 * Return true if data pushed.
 *
 * Assumes that biggest record size is much larger than the ring buffer.
 */
bool PushToFlexRingBuffer(RDBUF_HANDLE rb, const void* data, uint16_t data_len)
 {
    size_t total_len = sizeof(record_hdr_t) + data_len;

    if (data_len == 0 || data == NULL)
    {
        return false; // Invalid record length or data pointer
	}

    uint8_t* new_tail = rb->Tail + total_len;

    // First, if tail is ahead of or equal to head, check the contiguous space.
    if (rb->Tail >= rb->Head)
    {
        if ( new_tail >= rb->End )
        {
             new_tail = rb->Ring + total_len; // Reset tail to the start of the buffer

            if (new_tail >= rb->Head) // tail can't be == hed when pushign
                return false;

            rb->Tail = rb->Ring;

			// Update the previous record pointer to the current tail
            record_hdr_t* prev = (record_hdr_t*)rb->Prev;
            prev->Next = rb->Tail;

            if (rb->Count == 0)     // resync head too!
                rb->Head = rb->Tail;
        }
    }
	else if (new_tail >= rb->Head) // already rolled over behind head
    {
        return false; // Not enough space, can't push the record
	}

    record_hdr_t header = { new_tail, data_len };

	rb->Prev  = rb->Tail;       // Update the previous record pointer to the current tail

    // Write the header.
    memcpy(rb->Tail, &header, sizeof(header));
    rb->Tail += sizeof(record_hdr_t);

    // Write the record data.
    memcpy( rb->Tail, data, data_len);
	rb->Tail += data_len;

	rb->Count++; // Increment the record count

    return true;

}

bool PopFromFlexRingBuffer(frb_header_t* rb, ring_data_t* pdata)
{

    if (rb == NULL)
    {
        return false; // Buffer is empty, nothing to pop.
    }
    if (rb->Count == 0)
	{
		return false;
	}
	uint32_t state = cyhal_system_critical_section_enter();

    record_hdr_t header;
    memcpy(&header,  rb->Head, sizeof(record_hdr_t));

    pdata->Size = header.Size;
    pdata->Data = rb->Head + sizeof(record_hdr_t);

    rb->Head = header.Next; // Move head to the next record
	rb->Count--;            // D

	cyhal_system_critical_section_exit(state);

	return true;
}


/*-------------------------------------------------------------------------
 * StoreToNextSlot() - is a free running revolver
 *
ring_data_t* StoreFlexRingData(RDBUF_HANDLE phdr, void* pdata, size_t data_size)
{
	ring_data_t* prd = NULL;

	if (phdr != NULL)
	{
		uint32_t state = cyhal_system_critical_section_enter();

		size_t extsize = RING_RECORD_SIZE(data_size);

		if (extsize > phdr->Free) // roll over?
		{
			phdr->Tail = phdr->Ring;
			phdr->Free = phdr->Size;
		}

		prd = &(((ring_ptr_t*)phdr->Tail)->Payload);

		prd->Size = data_size;  // store data size in front of the copied data.

		memcpy(prd->Data, pdata, data_size);
		phdr->Free -= extsize;
		phdr->Tail += extsize;

		cyhal_system_critical_section_exit(state);
	}

	return prd;
}

*/



/***************************************************************************************
 * BENCHMARK RING BUFFERS
 * *************************************************************************************/

inline bool BoundsCheck(uint8_t* buf, size_t size, uint8_t* pt, size_t len, _ports_t port)
{
	char temp[100];
	int tlen=0;

	if (pt < buf)
	{
		tlen = snprintf(temp,sizeof(temp), "Bounds check failed %p < %p\n", pt, buf);
		SendToPort(port,(uint8_t*)temp, tlen);
		return true;
	}
	if ((pt + len) > (buf + size))
	{
		tlen = snprintf(temp,sizeof(temp),"Bounds check failed %p + %d > %p\n", pt, len, buf + size);
		SendToPort(port,(uint8_t*)temp, tlen);
		return true;
	}
	return false;
}

void benchmark_ringBuffer_callback(void* cbarg )
{
	*((bool*)cbarg) = true;
}

void BenchmarkRingBuffer(int size, int count)
{
	if (size < 4 * sizeof(int))
		size = 4 * sizeof(int);

	int rndcnt = size/sizeof(int);
	int store_size = rndcnt * sizeof(int) + sizeof(uint16_t);
	uint8_t* Buffer = malloc(store_size);
	memset(Buffer, 'X', store_size );

	printf("\nBenchmak DataBufferRing with %d buffers of %d bytes\n", count, store_size);

	bool stop = false;
	GPT_HANDLE th = CreateGPTimer(benchmark_ringBuffer_callback, &stop, 5000, true ); // one shot GPTimer stop at 5 sec.

	if (th == INVALID_GPTIMER_HANDLE)
		printf("Create Timer Filed!\n");

	StartGPTimer(th);

	size_t rbsize = RING_BUFFER_SPACE(store_size, count);
	uint8_t *ringbuf = malloc(rbsize);

	DBUF_HANDLE db = CreateRingBuffer(store_size, count, ringbuf, rbsize);

	uint32_t t = GetGPTImerCount(th);

	if (db == NULL)
	{
		printf("CreateRingBuffer() with %d buffers of %d size failed\n", count, store_size);
		return;
	}

	printf("CreateRingBuffer() created in %ld us\n", t);

	int* pint = (int*)(Buffer);
	int  crcoffset =  store_size - sizeof(uint16_t);
	uint16_t* pcrc = (uint16_t*)(Buffer +crcoffset);

	int wcnt = 0;

	stop = false;
	StartGPTimer(th);

	while(!stop)
	{
		for( int i = 0; i < rndcnt; i++) // fill the buffer with random data
		{
			pint[i] = rand();
		}
		*pcrc = crc16_ccitt(Buffer,crcoffset);	// add a CRC

		if (!PushRingBufferData(db, Buffer, store_size))
			break;

		wcnt++;
	};

	if (stop)
		printf("Timeout StoreDataBufferRecord() after 1 second\n");

	t = GetGPTImerCount(th);

	printf("Data Buffer filled with %d records of %d size in %ld us\n", wcnt, store_size, t);

	int rcnt = 0;
	int errors = 0;

	stop = false;
	StartGPTimer(th);

	while( GetRingBufferCount(db) != 0 )
	{
		if (stop)
			break;

		uint8_t* dread = PopRigngBufferData(db);

		if (dread == NULL)
			break;

		uint16_t dcrc = *((uint16_t*)(dread + crcoffset));

		if (dcrc !=  crc16_ccitt(dread,crcoffset) )
			errors++;

		rcnt++;
	};

	if (stop)
		printf("Timeout FifoPop() after 5 seconds\n");

	t = GetGPTImerCount(th);

	printf("Data Buffer Ring emptied %d records with %d errors in %ld us\n", rcnt, errors, t);

	ReleaseGPTimer(th);
	DestroyRingBuffer(db); // frees the ringbuffer;

	free(Buffer);

	printf("*** Benchmark ended with %d Errors ***\n",errors);
}

/*---------------------------------------------------------------------------------------------------*/

int errors = 0;

char* BenchmarkFlexRingBuffer(char**tokens,int cnt,_ports_t port)
{
	//int32_t bufsize, maxsize;
	//char temp[200];
	//int tlen= 0;

	return ("OBSOLETE Needs rewrite\n");

#ifdef NOT_OBSOLETE
	if (cnt < 4)
	{
		return ("BENCHMARK 3 recsize bufsize [outport]\n");
	}

	if ( ToInt32(tokens[2], &maxsize) == 0 )
	{
		return( "bad parameter 2: recsize\n");
	}
	 ToInt32(tokens[3], &bufsize);

	if (bufsize == 0 || bufsize < maxsize)
		return ( "bad parameter 3. bufsize");

	if (bufsize < 4 * (maxsize + sizeof(uint16_t)))	// a min of 4x data capactity
		bufsize = 4 * (maxsize + sizeof(uint16_t));

	if (cnt > 4)
	{
		_ports_t p = FindPortID(tokens[4]);
		if (p != INVALID_PORT)
			port = p;
		else
			return "Incorrect PORT name.";
	}

	// create a test data buffer
	int rndcnt = maxsize / sizeof(int);
	int store_size = rndcnt * sizeof(int) + sizeof(uint16_t); // add space for the crc
	uint8_t* databuffer = (uint8_t*) malloc(store_size);
	memset(databuffer, 0, store_size);

	// create a storage for the ring buff allocations
	int maxlocs = 2000;
	ring_data_t** locs = (ring_data_t**)malloc(sizeof(uint8_t*) * maxlocs);
	memset(locs, 0, sizeof(uint8_t*) * maxlocs);


	tlen = snprintf(temp, sizeof(temp),"\nBenchmak FlexRingBuffer with max %ld data size and %d buffer size\n", maxsize, store_size);
	SendToPort(port,(uint8_t*)temp, tlen);

	// create a timeout timer
	bool stop = false;
	GPT_HANDLE th = CreateGPTimer(benchmark_ringBuffer_callback, &stop, 60000, true); // one shot GPTimer stop at 5 sec.

	if (th == INVALID_GPTIMER_HANDLE)
		SendStringToPort(port,"Create Timer Filed!\n");

	StartGPTimer(th); // start the timeout timer

	//create a buffer for the FlexRingBuffer()
	uint8_t* ringbuf = (uint8_t*)malloc(bufsize);


	RDBUF_HANDLE db = CreateFlexRingBuffer(ringbuf, bufsize);

	uint32_t t = GpTimerEllapsedMs(th);

	if (db == NULL)
	{
		tlen = snprintf(temp, sizeof(temp),"CreateFlexRingBuffer() %ld size failed\n", bufsize);
		SendToPort(port,(uint8_t*)temp, tlen);
		return "Can't Continue;";
	}

	tlen = snprintf(temp, sizeof(temp),"CreateFlexRingBuffer() created in %ld us\n", t);
	SendToPort(port,(uint8_t*)temp, tlen);

	// Forward references
	void PopAndCheckData(RDBUF_HANDLE db, uint8_t * ringbuf, size_t bufsize, _ports_t port);
	ring_data_t* PushAndCheck(RDBUF_HANDLE db, int maxsize, uint8_t * databuffer, uint8_t * ringbuf, size_t bufsize, _ports_t port);

	stop = false;
	StartGPTimer(th);

	int loccnt = 0; // # of data samples stored
	int loops = 100;

	for (int i = 0; i < maxlocs; i++) // fill the buffer with random data
	{
		int isize = (rand() % maxsize);

		if (isize < 8)
			isize = 8;

		if (FlexBufferWillRoll(db, isize))
		{
			tlen = snprintf(temp, sizeof(temp),"Data Buffer filled with %d records in %ld us\n", loccnt, t);
			SendToPort(port,(uint8_t*)temp, tlen);

			for (int k = 0; k < loccnt; k++)
			{
				ring_data_t* tp = locs[k];

				uint16_t size = tp->Size;

				if (BoundsCheck(ringbuf, bufsize, tp->Data, size, port) )
					goto STOP;

				uint16_t jsize = size - sizeof(uint16_t);
				uint16_t crc = *((uint16_t*)(tp->Data + jsize));
				uint16_t CRC = crc16_ccitt(tp->Data, jsize);

				if (crc != CRC)
				{
					errors++;
				}

				tlen =  snprintf(temp,sizeof(temp),"%4d - Read %6d from %p %s\n", k, size, tp->Data, (crc == CRC? "OK": "CRC Error"));
				SendToPort(port,(uint8_t*)temp, tlen);
			}
			loccnt = 0;

			if (--loops == 0)
				goto STOP;
		}

		// Fill the payload with random data and a CRC
		int jsize = isize - sizeof(uint16_t);

		memset(databuffer, 0, store_size);

		for (int j = 0; j < jsize; j++)
		{
			databuffer[j] = (uint8_t)rand();
		}
		uint16_t CRC = crc16_ccitt(databuffer, jsize);	// add a CRC

		*((uint16_t*)(databuffer + jsize)) = CRC;

		ring_data_t* rd = (ring_data_t*)(db->Tail + sizeof(uint8_t*));

		PushToFlexRingBuffer(db, databuffer, isize);

 		tlen =  snprintf(temp,sizeof(temp),"%4d - Loop %3d - Stored %6d at %p - Percent free: %.1f%%\n", i, 100 - loops, isize, rd->Data, GetFlexBufferFreePct(db));
		SendToPort(port,(uint8_t*)temp, tlen);

		if (BoundsCheck(ringbuf, bufsize, rd->Data, isize, port))
			goto STOP;

		if (rd == NULL)
		{
			SendStringToPort(port,"Error storing with: StoreToFlexRingBuffer()\n");
			break;
		}

		locs[loccnt++] = rd;

		if (stop)
			goto End;
	}

	STOP:

	tlen =  snprintf(temp,sizeof(temp),"Round robin test ended with %d errors in %ld us\n", errors, t);
	SendToPort(port,(uint8_t*)temp, tlen);

	SendStringToPort(port,"*\n* TEST 2: Push/Pop  data in FIFO mode\n*\n");
	errors = 0;

	FlushFlexRingBuffer(db);

	ring_data_t* uprec = NULL;

	int testloops = 2000;

	int stores = 1;

	do // fill the buffer with random data
	{
		//int stores = (rand() % 50);

		if (stores == 10)
			stores = 1;

		for (int i = 0; i < stores; i++)
		{
			uprec = PushAndCheck(db, 100, databuffer, ringbuf, bufsize, port);
			if (uprec == NULL)
				break;
		}
		stores++;

		PopAndCheckData(db, ringbuf, bufsize, port);

 	} while ( --testloops > 0 && !stop);

	End:

	t = GetGPTImerCount(th);

	if (stop)
	{
		tlen =  snprintf(temp,sizeof(temp),"Timeout StoreDataBufferRecord() after %ld seconds\n", t);
		SendToPort(port,(uint8_t*)temp, tlen);
	}

	ReleaseGPTimer(th);

   	free(databuffer);
	free(locs);
	free(ringbuf);

	tlen =  snprintf(temp,sizeof(temp),"*** Benchmark ended with %d Errors ***\n", errors);
	SendToPort(port,(uint8_t*)temp, tlen);
	printf( "Benchmark END\n");

	return "Benchmark END\n";

#endif // NOT_OBSOLTE
}


uint16_t reccnt = 0;

ring_data_t* PushAndCheck(RDBUF_HANDLE db, int maxsize, uint8_t* databuffer,  uint8_t* ringbuf, size_t bufsize, _ports_t port)
{
	int isize = 0;
	ring_data_t* uprec = NULL;
	char temp[100];
	int tlen=0;

	isize = (rand() % maxsize);

	if (isize < 8)
		isize = 8;

	*((uint16_t*)databuffer) = ++reccnt; // store sequence number

	// Fill the payload with random data and a CRC
	int jsize = isize - sizeof(uint16_t);

	for (int j = sizeof(uint16_t); j < jsize; j++)
	{
		databuffer[j] = (uint8_t)rand();
	}
	uint16_t CRC = crc16_ccitt(databuffer, jsize);	// add a CRC

	*((uint16_t*)(databuffer + jsize)) = CRC;

	uprec = (ring_data_t*)(db->Head + sizeof(uint8_t*));

	PushToFlexRingBuffer(db, databuffer, isize);

	if (uprec != NULL)
	{
		tlen =  snprintf(temp,sizeof(temp),"%4d Store %4d at %p\n", reccnt, isize, uprec);
		SendToPort(port,(uint8_t*)temp, tlen);

		if (BoundsCheck(ringbuf, bufsize, uprec->Data, isize, port))
		{
 			errors++;
			return NULL;
		}
	}
	return uprec;
}

void PopAndCheckData(RDBUF_HANDLE db, uint8_t*ringbuf, size_t bufsize, _ports_t port)
{
	ring_data_t rec;
	char temp[100];
	int tlen=0;

	while ((PopFromFlexRingBuffer(db, &rec)) == true)
	{
		uint16_t size = rec.Size;
		uint8_t* pd = rec.Data;

		if (BoundsCheck(ringbuf, bufsize, rec.Data, size,port))
			break;

		uint16_t jsize = size - sizeof(uint16_t);
		uint16_t crc = *((uint16_t*)(pd + jsize));
		uint16_t CRC = crc16_ccitt(pd, jsize);

		uint16_t nr = *((uint16_t*)rec.Data);

		if (crc != CRC)
			errors++;

		tlen =  snprintf(temp,sizeof(temp),"%4d Read  %4d at %p %s\n", nr, size, rec.Data, (crc == CRC ? "OK" : "CRC Error"));
		SendToPort(port,(uint8_t*)temp, tlen);
	}
}


/***************************************************************************************
 * BENCHMARK MALLOC
 * *************************************************************************************/
void BenchmarkMalloc(size_t bufsize, int count)
{
	printf("\nBenchmak DataBufferRing with %d buffer size in %d increments\n", bufsize, count);

	bool stop = false;

	GPT_HANDLE th = CreateGPTimer(benchmark_ringBuffer_callback, &stop, 15000, true ); // one shot GPTimer

	uint32_t i = 0, last_size=0;

	StartGPTimer(th);

	while( i++ < count && !stop )
	{
		size_t size = i*bufsize;

		void *pbuf = malloc(size);

		if (pbuf == NULL)
			break;

		printf("Alloc %u bytes\n", size);

		last_size = size;

		free(pbuf);
	}

	if (stop)
		printf("Timeout STOP\n");

	uint32_t t = GetGPTImerCount(th);

	ReleaseGPTimer(th);
	printf("Max size allocated %lu in %lu us\n", last_size, t );
}




/**
 *  @brief
 *      Blocks for Secure Digital Memory Card.
 *
 *  @file
 *      block.c
 *  @author
 *      Peter Schmid, peter@spyr.ch
 *  @date
 *      2020-06-03
 *  @remark
 *      Language: C, STM32CubeIDE GCC
 *  @copyright
 *      Peter Schmid, Switzerland
 *
 *      This project Mecrsip-Cube is free software: you can redistribute it
 *      and/or modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation, either version 3 of
 *      the License, or (at your option) any later version.
 *
 *      Mecrsip-Cube is distributed in the hope that it will be useful, but
 *      WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *      General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with Mecrsip-Cube. If not, see http://www.gnu.org/licenses/.
 */

// System include files
// ********************
#include "cmsis_os.h"

// Application include files
// *************************
#include "app_common.h"
#include "main.h"
#include "block.h"
#include "sd.h"


// Defines
// *******

#define BLOCK_BUFFER_COUNT			4
#define BLOCK_BUFFER_SIZE			1024


// Private typedefs
// ****************

typedef struct {
	uint8_t Data[BLOCK_BUFFER_SIZE];
	int BlockNumber;  // -1 = Buffer unassigned
	uint8_t Current;
	uint8_t Updated;
} block_buffer_t;



// Private function prototypes
// ***************************

// SD raw block functions
static void get_block(int block_number, int buffer_index);
static void save_buffer(int buffer_index);
static void init_block(int block_number, int buffer_index);


// Global Variables
// ****************

// RTOS resources
// **************

static osMutexId_t BLOCK_MutexID;
static const osMutexAttr_t BLOCK_MutexAttr = {
		NULL,				// no name required
		osMutexPrioInherit,	// attr_bits
		NULL,				// memory for control block
		0U					// size for control block
};


// Hardware resources
// ******************


// Private Variables
// *****************

block_buffer_t BlockBuffers[BLOCK_BUFFER_COUNT];



// Public Functions
// ****************

/**
 *  @brief
 *      Initializes the Secure Digital Memory Card.
 *  @return
 *      None
 */
void BLOCK_init(void) {
	BLOCK_MutexID = osMutexNew(&BLOCK_MutexAttr);
	if (BLOCK_MutexID == NULL) {
		Error_Handler();
	}

	BLOCK_emptyBuffers();
}


/**
 *  @brief
 *      Empties all buffers.
 *
 *      empty-buffers ( -- ) Marks all block buffers as empty
 *  @return
 *      None
 */
void BLOCK_emptyBuffers(void) {
	int i;

	// only one thread is allowed to use blocks
	osMutexAcquire(BLOCK_MutexID, osWaitForever);

	for (i=0; i<BLOCK_BUFFER_COUNT; i++) {
		BlockBuffers[i].BlockNumber = -1;
		BlockBuffers[i].Current = FALSE;
		BlockBuffers[i].Updated = FALSE;
	}

	osMutexRelease(BLOCK_MutexID);
}


/**
 *  @brief
 *      Makes the most recent block dirty (updated).
 *
 *      update	( -- )	Mark most recent block as updated
 *  @return
 *      none
 */
void BLOCK_update(void) {
	int i;

	// only one thread is allowed to use blocks
	osMutexAcquire(BLOCK_MutexID, osWaitForever);

	for (i=0; i<BLOCK_BUFFER_COUNT; i++) {
		if (BlockBuffers[i].Current) {
			BlockBuffers[i].Updated = TRUE;
			break;
		}
	}

	osMutexRelease(BLOCK_MutexID);
}


/**
 *  @brief
 *      Gets a block from raw (unformatted) SD.
 *
 *      block ( n -- addr )	Return address of buffer for block n
 *      If a block buffer is assigned for block u, return its start address, a-addr.
 *      Otherwise, assign a block buffer for block u (if the assigned block buffer
 *      has been updated, transfer the contents to mass storage), read the block i
 *      to the block buffer and return its start address, a-addr.
 *  @param[in]
 *  	block_number
 *  @return
 *      Buffer Address
 */
uint8_t *BLOCK_get(int block_number) {
	int i;
	int j;
	uint8_t* buffer_p = NULL;

	// only one thread is allowed to use blocks
	osMutexAcquire(BLOCK_MutexID, osWaitForever);

	// already assigned?
	for (i=0; i<BLOCK_BUFFER_COUNT; i++) {
		if (BlockBuffers[i].BlockNumber == block_number) {
			// the block is already in the buffer -> make current
			for (j=0; j<BLOCK_BUFFER_COUNT; j++) {
				BlockBuffers[i].Current = FALSE;
			}
			BlockBuffers[i].Current = TRUE;
			buffer_p = &BlockBuffers[i].Data[0];
			break;
		}
	}

	if (buffer_p == NULL) {
		// is there an unassigned (empty) buffer?
		for (i=0; i<BLOCK_BUFFER_COUNT; i++) {
			if (BlockBuffers[i].BlockNumber < 0) {
				// buffer is unassigned (empty)
				get_block(block_number, i);
				buffer_p = &BlockBuffers[i].Data[0];
				break;
			}
		}
	}

	if (buffer_p == NULL) {
		// take the first not current buffer
		for (i=0; i<BLOCK_BUFFER_COUNT; i++) {
			if (! BlockBuffers[i].Current) {
				// buffer is not current
				if (BlockBuffers[i].Updated) {
					// Buffer is updated -> save buffer to SD
					save_buffer(i);
				}
				get_block(block_number, i);
				buffer_p = &BlockBuffers[i].Data[0];
				break;
			}
		}
	}

	osMutexRelease(BLOCK_MutexID);

	return buffer_p;
}


/**
 *  @brief
 *      Assigns a block. No file I/O.
 *
 *      buffer ( n -- addr )	Return address of buffer for block n
 *  @param[in]
 *  	block_number
 *  @return
 *      Buffer Address
 */
uint8_t *BLOCK_assign(int block_number) {
	int i;
	int j;
	uint8_t* buffer_p = NULL;

	// only one thread is allowed to use blocks
	osMutexAcquire(BLOCK_MutexID, osWaitForever);

	// already assigned?
	for (i=0; i<BLOCK_BUFFER_COUNT; i++) {
		if (BlockBuffers[i].BlockNumber == block_number) {
			// the block is already in the buffer -> make current
			for (j=0; j<BLOCK_BUFFER_COUNT; j++) {
				BlockBuffers[i].Current = FALSE;
			}
			BlockBuffers[i].Current = TRUE;
			buffer_p = &BlockBuffers[i].Data[0];
			break;
		}
	}

	if (buffer_p == NULL) {
		// is there an unassigned (empty) buffer?
		for (i=0; i<BLOCK_BUFFER_COUNT; i++) {
			if (BlockBuffers[i].BlockNumber < 0) {
				// buffer is unassigned (empty)
				init_block(block_number, i);
				buffer_p = &BlockBuffers[i].Data[0];
				break;
			}
		}
	}

	if (buffer_p == NULL) {
		// take the first not current buffer
		for (i=0; i<BLOCK_BUFFER_COUNT; i++) {
			if (! BlockBuffers[i].Current) {
				// buffer is not current
				if (BlockBuffers[i].Updated) {
					// Buffer is updated -> save buffer to SD
					save_buffer(i);
				}
				// fill the block with spaces
				init_block(block_number, i);
				buffer_p = &BlockBuffers[i].Data[0];
				break;
			}
		}
	}

	osMutexRelease(BLOCK_MutexID);

	return buffer_p;
}


/**
 *  @brief
 *      Saves all updated buffers to SD.
 *
 *      save-buffers ( -- ) Transfer the contents of each updated block buffer
 *      to mass storage, then mark all block buffers as assigned-clean.
 *  @return
 *      none
 */
void BLOCK_saveBuffers(void) {
	int i;

	// only one thread is allowed to use blocks
	osMutexAcquire(BLOCK_MutexID, osWaitForever);

	for (i=0; i<BLOCK_BUFFER_COUNT; i++) {
		if (BlockBuffers[i].Updated) {
			save_buffer(i);
			BlockBuffers[i].Updated = FALSE;
		}
	}

	osMutexRelease(BLOCK_MutexID);
}


/**
 *  @brief
 *      Saves and empties all updated buffers to SD.
 *
 *      flush ( -- ) save-buffers empty-buffers
 *  @return
 *      none
 */
void BLOCK_flushBuffers(void) {
	BLOCK_saveBuffers();
	BLOCK_emptyBuffers();
}


// Private Functions
// *****************

/**
 *  @brief
 *      Gets a block from raw (unformatted) SD.
 *
 *      The block consists of 2 SD blocks.
 *  @param[in]
 *  	block_number	raw block number.
 *  @param[in]
 *  	buffer_index	Buffer array index.
 *  @return
 *      none
 */
static void get_block(int block_number, int buffer_index) {
	SD_ReadBlocks(&BlockBuffers[buffer_index].Data[0], block_number*2, 2);
	BlockBuffers[buffer_index].BlockNumber = block_number;
	BlockBuffers[buffer_index].Current = TRUE;
	BlockBuffers[buffer_index].Updated = FALSE;
}


/**
 *  @brief
 *      Inits a block with spaces.
 *
 *  @param[in]
 *  	block_number	raw block number.
 *  @param[in]
 *  	buffer_index	Buffer array index.
 *  @return
 *      none
 */
static void init_block(int block_number, int buffer_index) {
	memset(&BlockBuffers[buffer_index].Data[0], ' ', BLOCK_BUFFER_SIZE);
	BlockBuffers[buffer_index].BlockNumber = block_number;
	BlockBuffers[buffer_index].Current = TRUE;
	BlockBuffers[buffer_index].Updated = FALSE;
}


/**
 *  @brief
 *      Saves a buffer to the SD.
 *
 *      The block consists of 2 SD blocks.
 *  @param[in]
 *  	buffer_index	Buffer array index.
 *  @return
 *      none
 */
static void save_buffer(int buffer_index) {
	SD_WriteBlocks(&BlockBuffers[buffer_index].Data[0], BlockBuffers[buffer_index].BlockNumber*2, 2);
	BlockBuffers[buffer_index].Updated = FALSE;
}



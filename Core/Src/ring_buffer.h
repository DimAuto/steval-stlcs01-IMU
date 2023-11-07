/**
 * @file ring_buffer.h
 * @author Dimitris Kalaitzakis (dkalaitzakis@theon.com)
 * @brief
 * @version 0.1
 * @date 2022-06-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <stdint.h>
#include <stdio.h>

#define EOF -1
#define RB_SIZE 40


/**
 * Ring buffer structure.
 */
typedef struct {
  uint8_t buffer[RB_SIZE];
  volatile int rdpos;
  volatile int wrpos;
  uint8_t size;
} RB_t;

/**
 * Initializes a ring buffer.
 *
 * @param	rb			The RB object.
 * @param	size		The size of the buffer in bytes.
 * @return Returns 1 if succeeds, 0 otherwise.
 */
uint8_t RB_init(RB_t* rb, uint8_t size);

/**
 * De-initializes a ring buffer.
 *
 * @warning RB_deinit should be used only on dynamically allocated
 * ring buffers!
 *
 * @param	rb			The RB object.
 */
void RB_deinit(RB_t* rb);

/**
 * Pushes a byte in the ring buffer.
 *
 * @param	rb			The RB object.
 * @param	byte		The byte to push.
 */
void RB_push(RB_t* rb, uint8_t byte);

/**
 * Pops a byte from the ring buffer.
 *
 * @param	rb			The RB object.
 * @return Returns the first byte in the ring buffer.
 */
uint8_t RB_pop(RB_t* rb);

/**
 * Pushes a byte in the front of the ring buffer. Useful to access
 * the ring buffer, in reverse or in LIFO order.
 *
 * @param	rb			The RB object.
 * @param	byte		The byte to push.
 */
void RB_pushFront(RB_t* rb, uint8_t byte);

/**
 * Pops a byte from the back of the ring buffer. Useful to access
 * the ring buffer, in reverse or in LIFO order.
 *
 * @param	rb			The RB object.
 * @return Returns the last byte in the ring buffer.
 */
uint8_t RB_popBack(RB_t* rb);

/**
 * Returns the capacity of the ring buffer.
 *
 * @param	rb			The RB object.
 * @return Returns the capacity of the ring buffer.
 */
uint8_t RB_capacity(RB_t* rb);

/**
 * Returns the size of the ring buffer
 *
 * @param	rb			The RB object.
 * @return Returns the bytes available in the ring buffer.
 */
uint8_t RB_size(RB_t* rb);

/**
 * Checks whether the ring buffer is empty.
 *
 * @param	rb			The RB object.
 * @return Returns 1 if empty, 0 otherwise.
 */
uint8_t RB_isEmpty(RB_t* rb);

/**
 * Checks whether the ring buffer is full.
 *
 * @param	rb			The RB object.
 * @return Returns 1 if full, 0 otherwise.
 */
uint8_t RB_isFull(RB_t* rb);

/**
 * Clears all data from the ring buffer.
 *
 * @param	rb			The RB object.
 */
void RB_clear(RB_t* rb);

#endif

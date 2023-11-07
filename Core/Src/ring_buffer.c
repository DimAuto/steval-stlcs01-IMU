/**
 * @file ring_buffer.c
 * @author Dimitris Kalaitzakis (dkalaitzakis@theon.com)
 * @brief
 * @version 0.1
 * @date 2022-06-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "ring_buffer.h"
#include <string.h>
#include "assert.h"

uint8_t RB_init(RB_t* rb, uint8_t size) {
  memset(rb->buffer, 0, sizeof(uint8_t)*size);
//  rb->buffer = malloc(size * sizeof(char));
//
  if (!rb->buffer) {
    return 0;
  }
  rb->rdpos = 0;
  rb->wrpos = 0;
  rb->size = size;

  return 1;
}

void RB_deinit(RB_t* rb) {
  rb->size = 0;
  rb->rdpos = 0;
  rb->wrpos = 0;
  memset(rb->buffer, 0, sizeof(rb->buffer));
}

void RB_push(RB_t* rb, uint8_t byte) {
  if ((rb->wrpos + 1) % rb->size == rb->rdpos) {
    return;
  }

  rb->buffer[rb->wrpos] = byte;
  rb->wrpos = (rb->wrpos + 1) % rb->size;
}

uint8_t RB_pop(RB_t* rb) {
  if (rb->rdpos == rb->wrpos) return EOF;

  uint8_t byte = rb->buffer[rb->rdpos];
  rb->rdpos = (rb->rdpos + 1) % rb->size;

  return byte;
}

void RB_pushFront(RB_t* rb, uint8_t byte) {
  uint8_t newrpos = (rb->rdpos - 1) < 0 ? (rb->size - 1) : rb->rdpos - 1;

  if (newrpos == rb->wrpos) {
    return;
  }

  rb->rdpos = newrpos;
  rb->buffer[rb->rdpos] = byte;
}

uint8_t RB_popBack(RB_t* rb) {
  if (rb->rdpos == rb->wrpos) return EOF;

  rb->wrpos = (rb->wrpos - 1) < 0 ? (rb->size - 1) : (rb->wrpos - 1);
  return rb->buffer[rb->wrpos];
}

uint8_t RB_capacity(RB_t* rb) { return rb->size; }

uint8_t RB_size(RB_t* rb) {
  return (rb->wrpos + rb->size - rb->rdpos) % rb->size;
}

uint8_t RB_isEmpty(RB_t* rb) { return (rb->rdpos == rb->wrpos); }

uint8_t RB_isFull(RB_t* rb) {
  return ((rb->wrpos + 1) % rb->size == rb->rdpos);
}

void RB_clear(RB_t* rb) {
  memset(rb->buffer, 0, sizeof(rb->buffer));
  rb->rdpos = 0;
  rb->wrpos = 0;
}

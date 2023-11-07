/*
 * helpers.c
 *
 *  Created on: Jun 7, 2022
 *      Author: dkalaitzakis
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include "helpers.h"


//void arrayRemoveFirstCharacter(uint8_t *array, uint8_t max_size){
//	uint8_t i;
//	for (i=0; i<max_size, i++){
//		if (i>0){
//
//		}
//	}
//}

const char *float_to_char(float num){
    char *tmp;
    char *sign = (num < 0) ? "-" : "";
    float val = (num < 0) ? -num : num;

    int tmp_int1 = val;
    if (val < 1){
        tmp_int1 = 0;
    }
    float tmp_frac = val - tmp_int1;      // Get fraction.
    int tmp_int2 = tmp_frac * 10000;  // Turn into integer.

    sprintf(tmp, "%d.%d", tmp_int1, tmp_int2);

    return tmp;
}

int float2char(float num, char *buffer){
    int ret = sprintf(buffer, sizeof buffer, "%f", num);

    if (ret < 0) {
        return 1;
    }
    if (ret >= sizeof buffer) {
        return 1;
    }
    return 0;
}

//const char *int_to_char(uint16_t num){
//    char tmp[16]={0};
//    sprintf(tmp, '%d', num);
//    return tmp;
//}
//uint16_t random(uint16_t min, uint16_t max){
//    srand(time(NULL));
//    uint16_t n = rand() % (max + 1 - min) + min;
//    return n;
//}


int my_itoa(int value, uint8_t *sp, int radix)
{
	uint8_t tmp[16] = {0};
	uint8_t *tp = tmp;
    uint8_t i;
    unsigned v;

    int sign = (radix == 10 && value < 0);
    if (sign)
        v = -value;
    else
        v = (unsigned)value;

    while (v || tp == tmp)
    {
        i = v % radix;
        v /= radix;
        if (i < 10)
          *tp++ = i+'0';
        else
          *tp++ = i + 'a' - 10;
    }

    uint8_t len = tp - tmp;
    if (sign)
    {
        *sp++ = '-';
        len++;
    }

    while (tp > tmp)
        *sp++ = *--tp;

    return len;
}

char* strtoke(char *str, const char *delim)
{
  static char *start = NULL; /* stores string str for consecutive calls */
  char *token = NULL; /* found token */
  /* assign new start in case */
  if (str) start = str;
  /* check whether text to parse left */
  if (!start) return NULL;
  /* remember current start as found token */
  token = start;
  /* find next occurrence of delim */
  start = strpbrk(start, delim);
  /* replace delim with terminator and move start to follower */
  if (start) *start++ = '\0';
  /* done */
  return token;
}

float atofLR(char *str) {
  float result = 0;
  float divisor = 10;
  int sign = 1;

  // Skip whitespace
  while (*str == ' ') {
    str++;
  }

  // Handle optional sign
  if (*str == '-') {
    sign = -1;
    str++;
  } else if (*str == '+') {
    sign = 1;
    str++;
  }

  // Parse integer part
  while (*str >= '0' && *str <= '9') {
    result = (result * 10) + (*str - '0');
    str++;
  }

  // Parse decimal part
  if (*str == '.') {
    str++;

    while (*str >= '0' && *str <= '9') {
      result += (*str - '0') / divisor;
      divisor *= 10;
      str++;
    }
  }

  return sign * result;
}

long altAtol(char *str) {
  float result = 0;
  long ret = 0;
  float divisor = 10;
  int sign = 1;

  // Skip whitespace
  while (*str == ' ') {
    str++;
  }

  // Handle optional sign
  if (*str == '-') {
    sign = -1;
    str++;
  } else if (*str == '+') {
    sign = 1;
    str++;
  }

  // Parse integer part
  while (*str >= '0' && *str <= '9') {
    result = (result * 10) + (*str - '0');
    str++;
  }

  // Parse decimal part
  if (*str == '.') {
    str++;

    while (*str >= '0' && *str <= '9') {
      result += (*str - '0') / divisor;
      divisor *= 10;
      str++;
    }
  }
  ret = (long)(result * 200000);

  return ret;
}

long coorsAtol(char *coors, char sign){
    uint8_t i = 0;
    uint8_t dotIndex=0;
    char tempBuffer[12]={0};
    int deg = 0;
    float min = 0;
    float divisor = 10;
    int s = 1;
    long result =0;


    // Skip whitespace
    while (*coors == ' ') {
      coors++;
    }
    while ((*coors >= '0' && *coors <= '9') || (*coors == '.')){
        tempBuffer[i] = *coors;
        if (*coors == '.'){
            dotIndex = i;
        }
        i++;
        coors++;
    }

    for (i=0; i<(dotIndex-2); i++){
        deg = (deg * 10) + (tempBuffer[i] - '0');
    }
//    deg = deg * 60;

    for (i=dotIndex-2; i<dotIndex;i++){
            min = (min * 10) + (tempBuffer[i] - '0');
    }
    for (i=dotIndex+1;i<12;i++){
      min += (tempBuffer[i] - '0') / divisor;
      divisor *= 10;
    }
    min = min /60;

    result = (long)((deg+min) * 200000);

    if ((sign == 'W')  || (sign == 'S')){
        s = -1;
    }
    return s * result;
}

uint32_t extractLong(uint8_t *payload, uint8_t startDigit){
    uint32_t val = 0;
    val |= (uint32_t)payload[startDigit + 0] << 8 * 0;
    val |= (uint32_t)payload[startDigit + 1] << 8 * 1;
    val |= (uint32_t)payload[startDigit + 2] << 8 * 2;
    val |= (uint32_t)payload[startDigit + 3] << 8 * 3;
    return val;
}

uint16_t extractInt(uint8_t *payload, uint8_t startDigit)
{
  uint16_t val = 0;
  val |= (uint16_t)payload[startDigit + 0] << 8 * 0;
  val |= (uint16_t)payload[startDigit + 1] << 8 * 1;
  return val;
}

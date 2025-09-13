/*
 * segment_display.h
 *
 *  Created on: May 13, 2025
 *      Author: merta
 */

#ifndef INC_SEGMENT_DISPLAY_H_
#define INC_SEGMENT_DISPLAY_H_


#include "stm32f4xx_hal.h"

void segment_init(void);
void update7Segment(uint8_t number);
void segment_refresh_loop(uint8_t number);

#endif

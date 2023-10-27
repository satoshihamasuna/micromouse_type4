/*
 * kalman_filter.h
 *
 *  Created on: Sep 15, 2022
 *      Author: sato1
 */

#ifndef MODULE_INC_KALMAN_FILTER_H_
#define MODULE_INC_KALMAN_FILTER_H_

#include "index.h"

float calc_speed_filter(float acc,float velo);
void filter_init();
#endif /* MODULE_INC_KALMAN_FILTER_H_ */

/*
 * operation_check.h
 *
 *  Created on: 2022/08/14
 *      Author: sato1
 */

#ifndef MODULE_INC_OPERATION_CHECK_H_
#define MODULE_INC_OPERATION_CHECK_H_

#include "index.h"
#include "run_param.h"

void slalom_check_R90();
void slalom_check_L90();
void slalom_fast_L90();
void fast_run_check();
void slalom_check(const t_straight_param*  velo_param,const t_param *const *mode,t_run_pattern run_pt);
void enkaigei(uint16_t time);
void wall_controll_check(uint16_t time);
#endif /* MODULE_INC_OPERATION_CHECK_H_ */

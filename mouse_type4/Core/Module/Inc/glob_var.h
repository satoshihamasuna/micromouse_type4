/*
 * glob_var.h
 *
 *  Created on: Aug 13, 2022
 *      Author: sato1
 */

#include "typedef.h"
#include "macro.h"

#ifndef MODULE_INC_GLOB_VAR_H_
#define MODULE_INC_GLOB_VAR_H_
#define GLOBAL
#else
#define GLOBAL extern
#endif

GLOBAL t_node closed_list[MAZE_SIZE_X][MAZE_SIZE_Y];
GLOBAL t_wall	 wall[MAZE_SIZE_X][MAZE_SIZE_Y];
GLOBAL uint16_t	 map[MAZE_SIZE_X][MAZE_SIZE_Y];
GLOBAL t_wall save_wall[MAZE_SIZE_X][MAZE_SIZE_Y];
GLOBAL t_position log_run_pos[MAZE_SIZE];

GLOBAL t_position mypos;
GLOBAL t_sensor sen_r,sen_l,sen_fr,sen_fl;

GLOBAL t_encoder enc_R,enc_L;
GLOBAL uint8_t mouse_mode;
GLOBAL t_bool is_mode_enable;

GLOBAL uint8_t run_mode;

GLOBAL t_sp_param machine;
GLOBAL t_sp_param target;
GLOBAL t_sp_param max_set;

GLOBAL t_pid_gain velo_g;
GLOBAL t_pid_gain omega_g;

GLOBAL t_wall_controll wall_controll;
GLOBAL t_bool diag_r_hosei_check,diag_l_hosei_check;
GLOBAL t_bool st_r_hosei_check,st_l_hosei_check;

GLOBAL float prev_V_r,V_r;
GLOBAL float prev_V_l,V_l;
GLOBAL float velo_Integral_controller;
GLOBAL float rad_Integral_controller;
GLOBAL int motor_out_r;
GLOBAL int motor_out_l;

GLOBAL float diag_predict_xr;
GLOBAL float diag_predict_xl;

GLOBAL float log_data[8][LOG_COUNT];
GLOBAL t_bool log_flag;
GLOBAL uint32_t time;

GLOBAL int enable_lsm;

/* MODULE_INC_GLOB_VAR_H_ */

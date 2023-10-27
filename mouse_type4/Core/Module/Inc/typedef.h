/*
 * typedef.h
 *
 *  Created on: Aug 13, 2022
 *      Author: sato1
 */
#include "index.h"
#include "macro.h"

#ifndef MODULE_INC_TYPEDEF_H_
#define MODULE_INC_TYPEDEF_H_

//#include "index.h"
//#include "macro.h"

typedef enum{
	 false = 0,
	 true  = 1,
}t_bool;

typedef struct{
	int32_t sp_pulse;
	int32_t prev_sp_pulse;
	float 	wheel_speed;
	float	prev_wheel_speed;
}t_encoder;

typedef enum{
	sensor_fl = 3,
	sensor_fr = 4,
	sensor_sl = 1,
	sensor_sr = 2,
}t_sensor_dir;

typedef enum{
	front = 0,
	right = 1,
	rear  = 2,
	left  = 3,
}t_local_dir;

typedef struct{
	t_bool front;
	t_bool right;
	t_bool rear;
	t_bool left;
}t_local_wall;

typedef enum{
	north  = 0,
	east   = 1,
	south  = 2,
	west   = 3,
	center = 4,
}t_direction;


typedef enum{
	North 		= 0,
	NorthEast 	= 1,
	East		= 2,
	SouthEast	= 3,
	South		= 4,
	SouthWest 	= 5,
	West		= 6,
	NorthWest	= 7,
	Dir_None    = 8,
}t_eight_dir;

typedef enum{
	x_axis = 0,
	y_axis = 1,
	z_axis = 2,
}t_axis;

typedef enum{
	turn_left	= LEFT,
	turn_right 	= RIGHT,
}t_turn_dir;


typedef struct{
	uint8_t x;
	uint8_t y;
	t_direction dir;
}t_position;

typedef struct{
	unsigned char north : 2;
	unsigned char east  : 2;
	unsigned char south : 2;
	unsigned char west  : 2;
}t_wall;

typedef enum{
	Straight 		= 1,
	Diagonal		= 2,
	Long_turnR90	= 3,
	Long_turnL90	= 4,
	Long_turnR180	= 5,
	Long_turnL180	= 6,
	Turn_in_R45		= 7,
	Turn_in_L45		= 8,
	Turn_out_R45	= 9,
	Turn_out_L45	= 10,
	Turn_in_R135	= 11,
	Turn_in_L135	= 12,
	Turn_out_R135	= 13,
	Turn_out_L135	= 14,
	Turn_RV90		= 15,
	Turn_LV90		= 16,
	Diagonal_R		= 17,
	Diagonal_L		= 18,
	run_pt_none		= 19,
}t_run_pattern;

typedef enum{
	non_wall_controll 	= 0,
	side_wall_controll  = 1,
	front_wall_controll	= 2,
}t_wall_controll_pt;

typedef struct{
	t_position	pos;
	t_position 	parent;
	t_eight_dir mouse_dir;
	uint16_t	time;
	uint8_t	acc_cnt;
	t_bool		determine;
	t_run_pattern run_pt;
}t_node_el;


typedef struct{
	int16_t st_x;
	int16_t st_y;
	int16_t cost;
	int16_t cost_h;
}t_MapNode;

typedef struct QUEUE{
	int tail;
	t_MapNode node[MAZE_SIZE];
}t_queue;


typedef struct{
	t_node_el north;
	t_node_el east;
	t_node_el south;
	t_node_el west;
	t_node_el center;
}t_node;

typedef struct{
	int16_t value;
	int16_t buff[50];
	float  distance;
	t_bool is_wall;
	t_bool is_controll;
	float  error;
}t_sensor;

typedef struct{
	float velo;
	float omega;
	float omega_acc;
	float Lstart;
	float Lend;
	float degree;
	t_turn_dir turn_dir;
}t_turn_param;

typedef struct{
	float velo;
	float r_min;
	float Lstart;
	float Lend;
	float degree;
	t_turn_dir turn_dir;
}t_turn_param_table;

typedef struct{
	float velo;
	float prev_velo;
	float I_velo;
	float accel;
	float prev_accel;
	float rad_velo;
	float prev_rad_velo;
	float I_rad_velo;
	float rad_accel;
	float prev_rad_accel;
	float length;
	float radian;
}t_sp_param;

typedef struct{
	float Kp;
	float Ki;
	float Kd;
}t_pid_gain;

typedef struct{
	t_pid_gain side_om_wall_gain;
	t_pid_gain front_om_wall_gain;
	t_pid_gain front_sp_wall_gain;
	t_bool	   is_controll;
	t_wall_controll_pt wall_controll_pt;
	float	om_error;
	float   prev_om_error;
	float	sum_om_error;
	float	sp_error;
	float   prev_sp_error;
	float	sum_sp_error;
}t_wall_controll;

typedef struct{
	t_turn_param_table const* param;
	t_pid_gain const* sp_gain;
	t_pid_gain const* om_gain;
}t_param;

typedef struct{
	//float base_velo;
	float max_velo;
	float acc;
}t_velo_param;

typedef struct{
	t_velo_param const* param;
	t_pid_gain const* sp_gain;
	t_pid_gain const* om_gain;
}t_straight_param;

#endif /* MODULE_INC_TYPEDEF_H_ */

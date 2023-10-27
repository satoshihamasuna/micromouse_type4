/*
 * search_run.c
 *
 *  Created on: 2023/02/20
 *      Author: sato1
 */
#include "index.h"
#include "glob_var.h"
#include "run_param.h"
#include "rad_accel_table.h"

void search_straight_update_maze(float len_target,float acc,float max_sp,float end_sp,int *x, int *y,int goal_size ,int mask){
	//Machine_Param_Initialize();
	//Target_Param_Initialize();
	//MAX_Param_Initialize();

	Sp_Param_I_Initialize(&machine);
	Sp_Param_I_Initialize(&target);
	int maze_map_update_flag = 0;
	if(end_sp == 0.0f)
	{
		run_mode = STRAIGHT_MODE;

		max_set.velo = max_sp;
		int wait_time = (int)(max_sp/3.0*1000.0);
		max_set.accel = acc;
		target.accel = acc;
		max_set.length = len_target;

		while(max_set.length - machine.length > 1000.0 * (max_set.velo *max_set.velo - end_sp*end_sp)/(2.0*max_set.accel)){
			if(maze_map_update_flag == 0){
				make_map_queue(x,y,goal_size,mask);
				maze_map_update_flag = 1;
			}
		}
		target.accel = -acc;
		while(machine.length < max_set.length){
			if(target.velo < 0.0){
				target.accel = 0.0;target.velo = 0.0;
				break;
			}
		}
		target.accel = 0.0;target.velo = 0.0;
		HAL_Delay(wait_time);
		run_mode = NON_CON_MODE;
	}
	else
	{
		run_mode = STRAIGHT_MODE;

		max_set.velo = max_sp;
		max_set.accel = acc;
		target.accel = acc;
		max_set.length = len_target;

		t_bool hosei_flag = false;
		t_bool r_wall = sen_r.is_wall;
		t_bool l_wall = sen_r.is_wall;
		uint16_t r_wall_cnt = 0;
		uint16_t l_wall_cnt = 0;
		while(max_set.length - machine.length > 1000.0 * (max_set.velo *max_set.velo- end_sp*end_sp)/(2.0*max_set.accel))
		{
			if(maze_map_update_flag == 0){
				make_map_queue(x,y,goal_size,mask);
				maze_map_update_flag = 1;
			}
			if(len_target == SECTION)
			{
				if(r_wall == true)	r_wall_cnt++;
				else				r_wall_cnt = 0;

				if(l_wall == true)	l_wall_cnt++;
				else				l_wall_cnt = 0;

				if((r_wall == true && sen_r.is_wall == false) && machine.length > 45.0 && r_wall_cnt > 100)
				{
					if(hosei_flag == false)
					{
						machine.length = SEARCH_HOSEI;
						hosei_flag = true;
					}
				}
				if((l_wall == true && sen_l.is_wall == false)&& machine.length > 45.0 && l_wall_cnt > 100)
				{
					if(hosei_flag == false)
					{
						machine.length = SEARCH_HOSEI;
						hosei_flag = true;
					}
				}
			}
			r_wall = sen_r.is_wall;
			l_wall = sen_l.is_wall;
		}
		target.accel = -acc;
		while(machine.length < max_set.length){
			if(target.velo < end_sp){
				target.accel = 0.0;target.velo = end_sp;
			}
		}
		target.accel = 0.0;target.velo = end_sp;
	}

	machine.length = 0.0;
	target.length =  0.0;
	//run_mode = NON_CON_MODE;

}

void search_turn90_table_update_maze(const t_param* parameter,int *x, int *y,int goal_size ,int mask)
{

	Set_PID_Gain(&velo_g, parameter->sp_gain->Kp, parameter->sp_gain->Ki, parameter->sp_gain->Kd);
	Set_PID_Gain(&omega_g, parameter->om_gain->Kp, parameter->om_gain->Ki, parameter->om_gain->Kd);
	Sp_Param_I_Initialize(&target);
	Sp_Param_I_Initialize(&machine);
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);

	run_mode = STRAIGHT_MODE;

	max_set.velo = parameter->param->velo;
	target.velo  = parameter->param->velo;
	max_set.length = parameter->param->Lstart;

	int maze_map_update_flag = 0;
	if(sen_fr.is_wall == true && sen_fl.is_wall == true)
	{
		while((90.0 + 0.0 - ((sen_fr.distance+sen_fl.distance)/2 )) < parameter->param->Lstart){
		}
	}
	else{
		while(machine.length < parameter->param->Lstart ){
			//if(maze_map_update_flag == 0){
				//make_map_queue(x,y,goal_size,mask);
				//maze_map_update_flag = 1;
			//}
			if((90.0 - (sen_fr.distance+sen_fl.distance)/2) > parameter->param->Lstart) break;
		}
	}


	run_mode = TURN_MODE_TABLE;
	max_set.rad_velo = parameter->param->velo/(parameter->param->r_min/1000);							//rad\s
	set_turn_time = DEG2RAD(parameter->param->degree)/(accel_Integral*max_set.rad_velo);	//s

	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	turn_time = 0;


	wall_controll.is_controll = true;
	while((float)turn_time < set_turn_time * 1000){
		if(maze_map_update_flag == 0){
			make_map_queue(x,y,goal_size,mask);
			maze_map_update_flag = 1;
		}
	}
	max_set.rad_velo = 0.0f;
	target.rad_velo = 0.0f;
	target.rad_accel = 0.0f;

	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);

	run_mode = STRAIGHT_MODE;
	machine.length	= target.length	= 0.0f;
	max_set.velo	= target.velo	= parameter->param->velo;
	max_set.length = parameter->param->Lend;
	while(machine.length < max_set.length );

	machine.length = target.length  = 0.0f;
	wall_controll.is_controll = true;

}


void search_straight_update_maze_zenmen(float len_target,float acc,float max_sp,float end_sp,int *x, int *y,int goal_size ,int mask){
	//Machine_Param_Initialize();
	//Target_Param_Initialize();
	//MAX_Param_Initialize();

	Sp_Param_I_Initialize(&machine);
	Sp_Param_I_Initialize(&target);
	int maze_map_update_flag = 0;
	if(end_sp == 0.0f)
	{
		run_mode = STRAIGHT_MODE;

		max_set.velo = max_sp;
		int wait_time = (int)(max_sp/3.0*1000.0);
		max_set.accel = acc;
		target.accel = acc;
		max_set.length = len_target;

		while(max_set.length - machine.length > 1000.0 * (max_set.velo *max_set.velo - end_sp*end_sp)/(2.0*max_set.accel)){
			if(maze_map_update_flag == 0){
				make_map_queue_zenmen(x,y,goal_size,mask);
				maze_map_update_flag = 1;
			}
		}
		target.accel = -acc;
		while(machine.length < max_set.length){
			if(target.velo < 0.0){
				target.accel = 0.0;target.velo = 0.0;
				break;
			}
		}
		target.accel = 0.0;target.velo = 0.0;
		HAL_Delay(wait_time);
		run_mode = NON_CON_MODE;
	}
	else
	{
		run_mode = STRAIGHT_MODE;

		max_set.velo = max_sp;
		max_set.accel = acc;
		target.accel = acc;
		max_set.length = len_target;

		t_bool hosei_flag = false;
		t_bool r_wall = sen_r.is_wall;
		t_bool l_wall = sen_r.is_wall;
		uint16_t r_wall_cnt = 0;
		uint16_t l_wall_cnt = 0;
		while(max_set.length - machine.length > 1000.0 * (max_set.velo *max_set.velo- end_sp*end_sp)/(2.0*max_set.accel))
		{
			if(maze_map_update_flag == 0){
				make_map_queue_zenmen(x,y,goal_size,mask);
				maze_map_update_flag = 1;
			}
			if(len_target == SECTION)
			{
				if(r_wall == true)	r_wall_cnt++;
				else				r_wall_cnt = 0;

				if(l_wall == true)	l_wall_cnt++;
				else				l_wall_cnt = 0;

				if((r_wall == true && sen_r.is_wall == false) && machine.length > 45.0 && r_wall_cnt > 100)
				{
					if(hosei_flag == false)
					{
						machine.length = SEARCH_HOSEI;
						hosei_flag = true;
					}
				}
				if((l_wall == true && sen_l.is_wall == false)&& machine.length > 45.0 && l_wall_cnt > 100)
				{
					if(hosei_flag == false)
					{
						machine.length = SEARCH_HOSEI;
						hosei_flag = true;
					}
				}
			}
			r_wall = sen_r.is_wall;
			l_wall = sen_l.is_wall;
		}
		target.accel = -acc;
		while(machine.length < max_set.length){
			if(target.velo < end_sp){
				target.accel = 0.0;target.velo = end_sp;
			}
		}
		target.accel = 0.0;target.velo = end_sp;
	}

	machine.length = 0.0;
	target.length =  0.0;
	//run_mode = NON_CON_MODE;

}

void search_turn90_table_update_maze_zenmen(const t_param* parameter,int *x, int *y,int goal_size ,int mask)
{

	Set_PID_Gain(&velo_g, parameter->sp_gain->Kp, parameter->sp_gain->Ki, parameter->sp_gain->Kd);
	Set_PID_Gain(&omega_g, parameter->om_gain->Kp, parameter->om_gain->Ki, parameter->om_gain->Kd);
	Sp_Param_I_Initialize(&target);
	Sp_Param_I_Initialize(&machine);
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);

	run_mode = STRAIGHT_MODE;

	max_set.velo = parameter->param->velo;
	target.velo  = parameter->param->velo;
	max_set.length = parameter->param->Lstart;

	int maze_map_update_flag = 0;
	if(sen_fr.is_wall == true && sen_fl.is_wall == true)
	{
		while((90.0 + 0.0 - ((sen_fr.distance+sen_fl.distance)/2 )) < parameter->param->Lstart){
		}
	}
	else{
		while(machine.length < parameter->param->Lstart ){
			//if(maze_map_update_flag == 0){
				//make_map_queue(x,y,goal_size,mask);
				//maze_map_update_flag = 1;
			//}
			if((90.0 - (sen_fr.distance+sen_fl.distance)/2) > parameter->param->Lstart) break;
		}
	}


	run_mode = TURN_MODE_TABLE;
	max_set.rad_velo = parameter->param->velo/(parameter->param->r_min/1000);							//rad\s
	set_turn_time = DEG2RAD(parameter->param->degree)/(accel_Integral*max_set.rad_velo);	//s

	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	turn_time = 0;


	wall_controll.is_controll = true;
	while((float)turn_time < set_turn_time * 1000){
		if(maze_map_update_flag == 0){
			make_map_queue_zenmen(x,y,goal_size,mask);
			maze_map_update_flag = 1;
		}
	}
	max_set.rad_velo = 0.0f;
	target.rad_velo = 0.0f;
	target.rad_accel = 0.0f;

	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);

	run_mode = STRAIGHT_MODE;
	machine.length	= target.length	= 0.0f;
	max_set.velo	= target.velo	= parameter->param->velo;
	max_set.length = parameter->param->Lend;
	while(machine.length < max_set.length );

	machine.length = target.length  = 0.0f;
	wall_controll.is_controll = true;

}

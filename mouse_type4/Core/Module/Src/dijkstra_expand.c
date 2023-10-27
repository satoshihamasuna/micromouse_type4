/*
 * dijkstra_expand.c
 *
 *  Created on: 2022/08/25
 *      Author: sato1
 */
#include "index.h"
#include "macro.h"
#include "glob_var.h"
#include "typedef.h"
#include "run_param.h"
#include "dijkstra.h"
#include "rad_accel_table.h"
#include <stdio.h>

typedef struct{
	uint16_t long_turn_90_time;
	uint16_t long_turn_180_time;
	uint16_t turn_V90_time;
	uint16_t turn_in45_time;
	uint16_t turn_out45_time;
	uint16_t turn_in135_time;
	uint16_t turn_out135_time;
}t_turn_time;

t_turn_time create_turn_time;

static const t_straight_param *const *glob_st_param;
static uint16_t glob_st_param_size;
static const t_straight_param *const *glob_di_param;
static uint16_t glob_di_param_size;

void Initialize_turn_time(const t_param *const *mode)
{
	float omega_mx = 0.0f;

	omega_mx = mode[Long_turnL180]->param->velo/(mode[Long_turnL180]->param->r_min/1000.0);
	create_turn_time.long_turn_180_time  = (uint16_t)(mode[Long_turnL180]->param->Lstart/mode[Long_turnL180]->param->velo);
	create_turn_time.long_turn_180_time += (uint16_t)(DEG2RAD(mode[Long_turnL180]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	create_turn_time.long_turn_180_time += (uint16_t)(mode[Long_turnL180]->param->Lend/mode[Long_turnL180]->param->velo);

	omega_mx = mode[Long_turnL90]->param->velo/(mode[Long_turnL90]->param->r_min/1000.0);
	create_turn_time.long_turn_90_time  = (uint16_t)(mode[Long_turnL90]->param->Lstart/mode[Long_turnL90]->param->velo);
	create_turn_time.long_turn_90_time += (uint16_t)(DEG2RAD(mode[Long_turnL90]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	create_turn_time.long_turn_90_time += (uint16_t)(mode[Long_turnL90]->param->Lend/mode[Long_turnL90]->param->velo);

	omega_mx = mode[Turn_LV90]->param->velo/(mode[Turn_LV90]->param->r_min/1000.0);
	create_turn_time.turn_V90_time  = (uint16_t)(mode[Turn_LV90]->param->Lstart/mode[Turn_LV90]->param->velo);
	create_turn_time.turn_V90_time += (uint16_t)(DEG2RAD(mode[Turn_LV90]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	create_turn_time.turn_V90_time += (uint16_t)(mode[Turn_LV90]->param->Lend/mode[Turn_LV90]->param->velo);

	omega_mx = mode[Turn_in_L45]->param->velo/(mode[Turn_in_L45]->param->r_min/1000.0);
	create_turn_time.turn_in45_time  = (uint16_t)(mode[Turn_in_L45]->param->Lstart/mode[Turn_in_L45]->param->velo);
	create_turn_time.turn_in45_time += (uint16_t)(DEG2RAD(mode[Turn_in_L45]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	create_turn_time.turn_in45_time += (uint16_t)(mode[Turn_in_L45]->param->Lend/mode[Turn_in_L45]->param->velo);

	omega_mx = mode[Turn_out_L45]->param->velo/(mode[Turn_out_L45]->param->r_min/1000.0);
	create_turn_time.turn_out45_time  = (uint16_t)(mode[Turn_out_L45]->param->Lstart/mode[Turn_out_L45]->param->velo);
	create_turn_time.turn_out45_time += (uint16_t)(DEG2RAD(mode[Turn_out_L45]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	create_turn_time.turn_out45_time += (uint16_t)(mode[Turn_out_L45]->param->Lend/mode[Turn_out_L45]->param->velo);

	omega_mx = mode[Turn_in_L135]->param->velo/(mode[Turn_in_L135]->param->r_min/1000.0);
	create_turn_time.turn_in135_time  = (uint16_t)(mode[Turn_in_L135]->param->Lstart/mode[Turn_in_L135]->param->velo);
	create_turn_time.turn_in135_time += (uint16_t)(DEG2RAD(mode[Turn_in_L135]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	create_turn_time.turn_in135_time += (uint16_t)(mode[Turn_in_L135]->param->Lend/mode[Turn_in_L135]->param->velo);

	omega_mx = mode[Turn_out_L135]->param->velo/(mode[Turn_out_L135]->param->r_min/1000.0);
	create_turn_time.turn_out135_time  = (uint16_t)(mode[Turn_out_L135]->param->Lstart/mode[Turn_out_L135]->param->velo);
	create_turn_time.turn_out135_time += (uint16_t)(DEG2RAD(mode[Turn_out_L135]->param->degree)/(accel_Integral*omega_mx)*1000.0);
	create_turn_time.turn_out135_time += (uint16_t)(mode[Turn_out_L135]->param->Lend/mode[Turn_out_L135]->param->velo);

}

void Initialize_st_param_for_time(const t_straight_param *const *st_mode,uint16_t size_st_mode)
{
	glob_st_param = st_mode;
	glob_st_param_size = size_st_mode;
}

void Initialize_di_param_for_time(const t_straight_param *const *di_mode,uint16_t size_di_mode)
{
	glob_di_param = di_mode;
	glob_di_param_size = size_di_mode;
}

uint16_t calc_straight_time(const t_straight_param *const *mode,uint16_t mode_size,float length)
{
	uint16_t time = UINT16_MAX;
	float start_velo 	= mode[0]->param->max_velo;
	float end_velo 		= mode[0]->param->max_velo;
	float acc_time = 0.0;	float deacc_time = 0.0;
	float acc_length = 0.0; float deacc_length = 0.0;
	for(int i = mode_size-1; i >= 0;i--){
		float max_velo	= mode[i]->param->max_velo;
		float accel		= mode[i]->param->acc;
        acc_length		= ((max_velo*1000.0)*(max_velo*1000.0)-(start_velo*1000.0)*(start_velo*1000.0))/(2*accel*1000.0);
        deacc_length    = ((max_velo*1000.0)*(max_velo*1000.0)-(end_velo*1000.0)*(end_velo*1000.0))/(2*accel*1000.0);
        if(length-OFF_SET_LENGTH-(acc_length+deacc_length) >= 0.0)
        {
        	acc_time = (max_velo - start_velo)/accel * 1000.0;
        	deacc_time = (max_velo - end_velo)/accel * 1000.0;
        	time = (uint16_t)OFF_SET_LENGTH/mode[0]->param->max_velo+(uint16_t)((length-(acc_length+deacc_length))/max_velo) + (uint16_t)acc_time + (uint16_t)deacc_time;
        	break;
        }
	}
	return time;
}

t_straight_param calc_straight_max_velo(const t_straight_param *const *mode,uint16_t mode_size,float length)
{
	t_straight_param return_param;

	uint16_t time = UINT16_MAX;;
	float start_velo 	= mode[0]->param->max_velo;
	float end_velo 		= mode[0]->param->max_velo;
	return_param.param = mode[0]->param;
	return_param.sp_gain		  = mode[0]->sp_gain;
	return_param.om_gain		  = mode[0]->om_gain;
	float acc_time = 0.0;	float deacc_time = 0.0;
	float acc_length = 0.0; float deacc_length = 0.0;
	for(int i = mode_size-1; i >= 0;i--){
		float max_velo	= mode[i]->param->max_velo;
		float accel		= mode[i]->param->acc;
        acc_length		= ((max_velo*1000.0)*(max_velo*1000.0)-(start_velo*1000.0)*(start_velo*1000.0))/(2*accel*1000.0);
        deacc_length    = ((max_velo*1000.0)*(max_velo*1000.0)-(end_velo*1000.0)*(end_velo*1000.0))/(2*accel*1000.0);
        if(length-OFF_SET_LENGTH-(acc_length+deacc_length) >= 0.0)
        {
        	return_param.param 			  =	mode[i]->param;
        	return_param.sp_gain		  = mode[i]->sp_gain;
        	return_param.om_gain		  = mode[i]->om_gain;
        	acc_time = (max_velo - start_velo)/accel * 1000.0;
        	deacc_time = (max_velo - end_velo)/accel * 1000.0;
        	time = (uint16_t)OFF_SET_LENGTH/mode[0]->param->max_velo+(uint16_t)((length-OFF_SET_LENGTH-(acc_length+deacc_length))/max_velo) + (uint16_t)acc_time + (uint16_t)deacc_time;
        	break;
        }
	}
	return return_param;
}


t_straight_param calc_end_straight_max_velo(const t_straight_param *const *mode,uint16_t mode_size,float length)
{
	t_straight_param return_param;

	uint16_t time = UINT16_MAX;;
	float start_velo 	= mode[0]->param->max_velo;
	float end_velo 		= 0.0;
	return_param.param = mode[0]->param;
	return_param.sp_gain		  = mode[0]->sp_gain;
	return_param.om_gain		  = mode[0]->om_gain;
	float acc_time = 0.0;	float deacc_time = 0.0;
	float acc_length = 0.0; float deacc_length = 0.0;
	for(int i = mode_size-1; i >= 0;i--){
		float max_velo	= mode[i]->param->max_velo;
		float accel		= mode[i]->param->acc;
        acc_length		= ((max_velo*1000.0)*(max_velo*1000.0)-(start_velo*1000.0)*(start_velo*1000.0))/(2*accel*1000.0);
        deacc_length    = ((max_velo*1000.0)*(max_velo*1000.0)-(end_velo*1000.0)*(end_velo*1000.0))/(2*accel*1000.0);
        if(length-OFF_SET_LENGTH-(acc_length+deacc_length) >= 0.0)
        {
        	return_param.param 			  =	mode[i]->param;
        	return_param.sp_gain		  = mode[i]->sp_gain;
        	return_param.om_gain		  = mode[i]->om_gain;
        	acc_time = (max_velo - start_velo)/accel * 1000.0;
        	deacc_time = (max_velo - end_velo)/accel * 1000.0;
        	time = (uint16_t)OFF_SET_LENGTH/mode[0]->param->max_velo+(uint16_t)((length-OFF_SET_LENGTH-(acc_length+deacc_length))/max_velo) + (uint16_t)acc_time + (uint16_t)deacc_time;
        	break;
        }
	}
	return return_param;
}


uint16_t calc_time(t_run_pattern run_pt,int acc_cnt){
	uint16_t time = 0;
	switch(run_pt)
	{
		case Straight:
			time = calc_straight_time(glob_st_param, glob_st_param_size,SECTION*(float)acc_cnt);
#if DEBUG_MODE == 1
	printf("Straight:time:%d\n",time);
#endif
			break;
		case Diagonal:
			time = calc_straight_time(glob_di_param, glob_di_param_size,DIAG_SECTION*(float)acc_cnt);
#if DEBUG_MODE == 1
	printf("Diagonal:time:%d\n",time);
#endif
			break;
		case Long_turnL90:
		case Long_turnR90:
			time = create_turn_time.long_turn_90_time;
#if DEBUG_MODE == 1
	printf("Long_turn90:time:%d\n",time);
#endif
			break;
		case Long_turnL180:
		case Long_turnR180:
			time = create_turn_time.long_turn_180_time;
#if DEBUG_MODE == 1
	printf("Long_turn180:time:%d\n",time);
#endif
			break;
		case Turn_LV90:
		case Turn_RV90:
			time = create_turn_time.turn_V90_time;
#if DEBUG_MODE == 1
	printf("turn_V90:time:%d\n",time);
#endif
			break;
		case Turn_in_L45:
		case Turn_in_R45:
			time = create_turn_time.turn_in45_time;
#if DEBUG_MODE == 1
	printf("turn_in45:time:%d\n",time);
#endif
			break;
		case Turn_out_L45:
		case Turn_out_R45:
			time = create_turn_time.turn_out45_time;
#if DEBUG_MODE == 1
	printf("turn_out45:time:%d\n",time);
#endif
			break;
		case Turn_in_L135:
		case Turn_in_R135:
			time = create_turn_time.turn_in135_time;
#if DEBUG_MODE == 1
	printf("turn_in135:time:%d\n",time);
#endif
			break;
		case Turn_out_L135:
		case Turn_out_R135:
			time = create_turn_time.turn_out135_time;
#if DEBUG_MODE == 1
	printf("turn_out135:time:%d\n",time);
#endif
			break;
		case Diagonal_L:
		case Diagonal_R:
		case run_pt_none:
			time = 0;
			break;
	}
	return time;
}

uint16_t pos_run_time(t_position pos){
	uint16_t time;
	switch(pos.dir)
	{
		case north:
			time = closed_list[pos.x][pos.y].north.time;
			break;
		case east:
			time = closed_list[pos.x][pos.y].east.time;
			break;
		case south:
			time = closed_list[pos.x][pos.y].south.time;
			break;
		case west:
			time = closed_list[pos.x][pos.y].west.time;
			break;
		case center:
			time = closed_list[pos.x][pos.y].center.time;
			break;
	}
	return time;
}

t_run_pattern pos_run_pt(t_position pos)
{
	t_run_pattern run_pt;
	switch(pos.dir)
	{
		case north:
			run_pt = closed_list[pos.x][pos.y].north.run_pt;
			break;
		case east:
			run_pt = closed_list[pos.x][pos.y].east.run_pt;
			break;
		case south:
			run_pt = closed_list[pos.x][pos.y].south.run_pt;
			break;
		case west:
			run_pt = closed_list[pos.x][pos.y].west.run_pt;
			break;
		case center:
			run_pt = closed_list[pos.x][pos.y].center.run_pt;
			break;
	}
	return run_pt;
}

t_position pos_parent(t_position pos)
{
	t_position parent;

	switch(pos.dir)
	{
		case north:
			parent = closed_list[pos.x][pos.y].north.parent;
			break;
		case east:
			parent = closed_list[pos.x][pos.y].east.parent;
			break;
		case south:
			parent = closed_list[pos.x][pos.y].south.parent;
			break;
		case west:
			parent = closed_list[pos.x][pos.y].west.parent;
			break;
		case center:
			parent = closed_list[pos.x][pos.y].center.parent;
			break;
	}
	return parent;
}

t_eight_dir return_mouse_dir(t_position pos)
{
	t_eight_dir mouse_dir;
	switch(pos.dir)
	{
		case north:
			mouse_dir = closed_list[pos.x][pos.y].north.mouse_dir;
			break;
		case east:
			mouse_dir = closed_list[pos.x][pos.y].east.mouse_dir;
			break;
		case south:
			mouse_dir = closed_list[pos.x][pos.y].south.mouse_dir;
			break;
		case west:
			mouse_dir = closed_list[pos.x][pos.y].west.mouse_dir;
			break;
		case center:
			mouse_dir = closed_list[pos.x][pos.y].center.mouse_dir;
			break;
	}
	return mouse_dir;
}

t_node_el return_close_list_el(t_position pos){
	t_node_el  node_el;
	switch(pos.dir)
	{
		case north:
			node_el = closed_list[pos.x][pos.y].north;
			break;
		case east:
			node_el = closed_list[pos.x][pos.y].east;
			break;
		case south:
			node_el = closed_list[pos.x][pos.y].south;
			break;
		case west:
			node_el = closed_list[pos.x][pos.y].west;
			break;
		case center:
			node_el = closed_list[pos.x][pos.y].center;
			break;
	}
	return node_el;
}

t_bool is_determine(t_position pos)
{
	t_bool determine;
	switch(pos.dir)
	{
		case north:
			determine = closed_list[pos.x][pos.y].north.determine;
			break;
		case east:
			determine = closed_list[pos.x][pos.y].east.determine;
			break;
		case south:
			determine = closed_list[pos.x][pos.y].south.determine;
			break;
		case west:
			determine = closed_list[pos.x][pos.y].west.determine;
			break;
		case center:
			determine = closed_list[pos.x][pos.y].center.determine;
			break;
	}
	return determine;
}



void straight_expand(t_position pos,t_eight_dir mouse_dir)
{
	t_position next_pos 	= make_position(pos.x, pos.y, pos.dir);
	t_position parent_pos	= make_position(pos.x, pos.y, pos.dir);
	t_run_pattern run_pt	= Straight;

#if DEBUG_MODE == 1
	printf("STRAIGHT_EXPAND\n");
	HAL_Delay(1);
#endif

	if(pos.dir == center)
	{
		if(mouse_dir == North || mouse_dir == South)
		{
			for(int i = 0;i < MAZE_SIZE_Y;i++)
			{
				if(wall_check(make_position(next_pos.x, next_pos.y, (mouse_dir/2)), 0x03) == false)
				{
					if(mouse_dir == North)
					{
						next_pos.y = next_pos.y + 1;
					}
					else if(mouse_dir == South)
					{
						next_pos.y = next_pos.y - 1;
					}

					if(next_pos.y < MAZE_SIZE_Y && next_pos.y >= 0)
					{
						t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
						uint16_t set_run_t = pos_run_time(parent_pos) + calc_time(run_pt,i+1);

						if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
							close_list_set(set_pos,parent_pos,mouse_dir,set_run_t,run_pt,(i+1),false);
						}
					}
					else
					{
						break;
					}
				}
				else
				{
					break;
				}
			}
		}
		else if(mouse_dir == East || mouse_dir == West)
		{
			for(int i = 0;i < MAZE_SIZE_X;i++)
			{
				if(wall_check(make_position(next_pos.x, next_pos.y, (mouse_dir/2)), 0x03) == false)
				{
					if(mouse_dir == East)
					{
						next_pos.x = next_pos.x + 1;
					}
					else if(mouse_dir == West)
					{
						next_pos.x = next_pos.x - 1;
					}

					if(next_pos.x < MAZE_SIZE_X &&next_pos.x >= 0)
					{
						t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
						uint16_t set_run_t = pos_run_time(parent_pos) + calc_time(run_pt,i+1);
						if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
							close_list_set(set_pos,parent_pos,mouse_dir,set_run_t,run_pt,(i+1),false);
						}
					}
					else
					{
						break;
					}

				}
				else
				{
					break;
				}
			}
		}
	}
}

void diagonal_expand(t_position pos,t_eight_dir mouse_dir)
{
	t_position next_pos 	= make_position(pos.x, pos.y, pos.dir);
	t_position set_pos		= make_position(pos.x, pos.y, pos.dir);
	t_position parent_pos	= make_position(pos.x, pos.y, pos.dir);
	t_run_pattern run_pt	= Diagonal;
	for(int i = 0;i < MAZE_SIZE_X + MAZE_SIZE_Y;i++)
	{
		if(wall_check(next_pos, 0x03) != false) break;
		set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
        if(next_pos.dir == north)
        {
            if(mouse_dir == NorthEast)
            {
                next_pos.x = next_pos.x + 1;
                next_pos.y = next_pos.y + 1;
            }
            else if(mouse_dir == SouthEast)
            {
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
            }
            else if(mouse_dir == SouthWest)
            {
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
            }
            else if(mouse_dir ==  NorthWest)
            {
                next_pos.x = next_pos.x - 1;
                next_pos.y = next_pos.y + 1;
            }
        }

        else if(next_pos.dir == east)
        {
            if(mouse_dir ==  NorthEast)
            {
                next_pos.x = next_pos.x + 1;
                next_pos.y = next_pos.y + 1;
            }
			else if( mouse_dir ==  SouthEast)
            {
                next_pos.x = next_pos.x + 1;
                next_pos.y = next_pos.y - 1;
            }
            else if( mouse_dir ==  SouthWest)
            {
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
            }
            else if( mouse_dir ==  NorthWest)
            {
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
            }
        }

        else if( next_pos.dir == south)
        {
            if (mouse_dir ==  NorthEast)
			{
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
			}
			else if( mouse_dir ==  SouthEast)
			{
                next_pos.x = next_pos.x + 1;
                next_pos.y = next_pos.y - 1;
			}
			else if( mouse_dir ==  SouthWest)
			{
                next_pos.x = next_pos.x - 1;
                next_pos.y = next_pos.y - 1;
			}
            else if( mouse_dir ==  NorthWest)
            {
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
            }
        }
        else if( next_pos.dir == west)
        {
            if(mouse_dir ==  NorthEast)
            {
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
            }
            else if( mouse_dir ==  SouthEast)
            {
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
            }
            else if( mouse_dir ==  SouthWest)
            {
                next_pos.x = next_pos.x - 1;
                next_pos.y = next_pos.y - 1;
            }
			else if( mouse_dir ==  NorthWest)
			{
                next_pos.x = next_pos.x - 1;
                next_pos.y = next_pos.y + 1 ;
			}
        }
        next_pos.dir = (next_pos.dir + 2*(next_pos.dir%2) + (8 + 4) - mouse_dir)%4;

        if(next_pos.x < 0 || next_pos.x >= MAZE_SIZE_X) break;
        if(next_pos.y < 0 || next_pos.y >= MAZE_SIZE_Y) break;

        if(wall_check(next_pos, 0x03) == false && i != 0)
        {
        	int set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, i);
			if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
				close_list_set(set_pos,parent_pos,mouse_dir,set_run_t,run_pt,i,false);
			}
        }

	}
}

void turn_in_R45_expand(t_position pos,t_eight_dir mouse_dir)
{
     t_eight_dir next_mouse_dir = (mouse_dir +1 + 8)%8;
     t_position next_pos = make_position(pos.x,pos.y,pos.dir);
     t_position parent_pos = make_position(pos.x,pos.y,pos.dir);
     t_bool update_flag = false;
     t_run_pattern run_pt = Turn_in_R45;
     if(pos.dir == center)
     {
         if(mouse_dir == North)
         {
             if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 <  MAZE_SIZE_Y)
             {
                 //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y+1][(int(mouse_dir/2) + 1 + 4)%4] == False:
                 if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y+1, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                 {
                	 //if self.wall_data[pos.x+1][pos.y+1][int(mouse_dir/2)] == False:
                	 if(wall_check(make_position(pos.x+1, pos.y+1, mouse_dir/2), 0x03) == false)
                	 {
                		 next_pos.x = pos.x + 1;
                		 next_pos.y = pos.y + 1;
                		 next_pos.dir = west;
                		 update_flag = true;
                	 }
                 }
             }
         }
         else if(mouse_dir == East)
         {
             if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 >= 0)
             {
                 //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x+1][pos.y][(int(mouse_dir/2) + 1 + 4)%4] == False:
                 if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x+1, pos.y, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                 {
                	 //if self.wall_data[pos.x+1][pos.y-1][int(mouse_dir/2)] == False:
                	 if(wall_check(make_position(pos.x+1, pos.y-1, mouse_dir/2), 0x03) == false)
                	 {
						 next_pos.x = pos.x + 1;
						 next_pos.y = pos.y - 1;
						 next_pos.dir = north;
						 update_flag = true;
                	 }
                 }
             }
     	 }
         else if(mouse_dir == South)
         {
             if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
             {
                 //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y-1][(int(mouse_dir/2) + 1 + 4)%4] == False:
                 if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y-1, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                 {
                	 //if self.wall_data[pos.x-1][pos.y-1][int(mouse_dir/2)] == False:
                	 if(wall_check(make_position(pos.x-1, pos.y-1, mouse_dir/2), 0x03) == false)
                	 {
						 next_pos.x = pos.x - 1;
						 next_pos.y = pos.y - 1;
						 next_pos.dir = east;
						 update_flag = true;
                	 }
                 }
             }
         }
         else if(mouse_dir == West)
         {
             if(pos.x - 1 >= 0 && pos.y + 1 < MAZE_SIZE_Y)
             {
                 //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x-1][pos.y][(int(mouse_dir/2) + 1 + 4)%4] == False:
                 if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x-1, pos.y, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                 {
                	 //if self.wall_data[pos.x-1][pos.y+1][int(mouse_dir/2)] == False:
                	 if(wall_check(make_position(pos.x-1, pos.y+1, mouse_dir/2), 0x03) == false)
                	 {
                		 next_pos.x = pos.x - 1;
                		 next_pos.y = pos.y + 1;
                		 next_pos.dir = south;
                		 update_flag = true;
                	 }
                 }
             }
         }
     }

     if(update_flag == true)
     {
    	 t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
    	 uint16_t	set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, 0);
    	 if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
    		 close_list_set(set_pos,parent_pos,next_mouse_dir,set_run_t,run_pt,0,false);
    	 }

     }
}

void turn_in_L45_expand(t_position pos,t_eight_dir mouse_dir)
{
    t_eight_dir next_mouse_dir = (mouse_dir -1 + 8)%8;
    t_position next_pos = make_position(pos.x,pos.y,pos.dir);
    t_position parent_pos = make_position(pos.x,pos.y,pos.dir);
    t_bool update_flag = false;
    t_run_pattern run_pt = Turn_in_L45;

    if(pos.dir == center)
    {
        if (mouse_dir == North)
        {
            if(pos.x - 1 >= 0 && pos.y + 1 < MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y+1][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y+1, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	//if self.wall_data[pos.x-1][pos.y+1][int(mouse_dir/2)] == False:
                	if(wall_check(make_position(pos.x-1, pos.y+1, mouse_dir/2), 0x03) == false)
                	{
                		next_pos.x = pos.x - 1;
                		next_pos.y = pos.y + 1;
                		next_pos.dir = east;
                		update_flag = true;
                	}
                }
            }
        }
        else if(mouse_dir == East)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 < MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x+1][pos.y][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x+1, pos.y, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	//if self.wall_data[pos.x+1][pos.y+1][int(mouse_dir/2)] == False:
                	if(wall_check(make_position(pos.x+1, pos.y+1, mouse_dir/2), 0x03) == false)
                	{
						next_pos.x = pos.x + 1;
						next_pos.y = pos.y + 1;
						next_pos.dir = south;
						update_flag = true;
                	}
                }
            }
        }
        else if(mouse_dir == South)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y-1][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y-1, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	//if self.wall_data[pos.x+1][pos.y-1][int(mouse_dir/2)] == False:
                	if(wall_check(make_position(pos.x+1, pos.y-1, mouse_dir/2), 0x03) == false)
                	{
						next_pos.x = pos.x + 1;
						next_pos.y = pos.y - 1;
						next_pos.dir = west;
						update_flag = true;
                	}
                }
            }
        }
        else if(mouse_dir == West)
        {
            if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x-1][pos.y][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x-1, pos.y, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	//if self.wall_data[pos.x-1][pos.y-1][int(mouse_dir/2)] == False:
                	if(wall_check(make_position(pos.x-1, pos.y-1, mouse_dir/2), 0x03) == false)
                	{
                		next_pos.x = pos.x - 1;
                		next_pos.y = pos.y - 1;
                		next_pos.dir = north;
                		update_flag = true;
                	}
                }
            }
        }
    }

    if(update_flag == true)
    {
    	t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
    	uint16_t	set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, 0);
		if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
			close_list_set(set_pos,parent_pos,next_mouse_dir,set_run_t,run_pt,0,false);
		}
    }
}

void turn_out_R45_expand(t_position pos,t_eight_dir mouse_dir)
{
    t_eight_dir next_mouse_dir = (mouse_dir + 1 + 8)%8;
    t_position next_pos = make_position(pos.x,pos.y,pos.dir);
    t_position parent_pos = make_position(pos.x,pos.y,pos.dir);
    t_bool update_flag = false;
    t_run_pattern run_pt = Turn_out_R45;

    if(pos.dir == north)
    {
        if(mouse_dir == NorthEast)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 < MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x+1][pos.y + 1][Node_pos_dir.West] == False:
                if(wall_check(make_position(pos.x + 1, pos.y + 1, west), 0x03) == false)
                {
                    next_pos.x = pos.x + 1;
                    next_pos.y = pos.y + 1;
                    next_pos.dir = center;
                    update_flag  = true;
                }
            }
        }
        else if(mouse_dir == SouthWest)
        {
            if(pos.x - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][Node_pos_dir.West] == False:
                if(wall_check(make_position(pos.x, pos.y, west), 0x03) == false)
                {
                    next_pos.x = pos.x - 1;
                    next_pos.y = pos.y;
					next_pos.dir = center;
					update_flag  = true;
                }
            }
        }
    }
    else if(pos.dir == east)
    {
        if(mouse_dir == SouthEast)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x+1][pos.y - 1][Node_pos_dir.North] == False:
                if(wall_check(make_position(pos.x + 1, pos.y - 1,north), 0x03) == false)
                {
                    next_pos.x = pos.x + 1;
                    next_pos.y = pos.y - 1;
                    next_pos.dir = center;
                    update_flag  = true;
                }
            }
        }
        else if(mouse_dir == NorthWest)
        {
            if(pos.y + 1 < MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x][pos.y][Node_pos_dir.North] == False:
                if(wall_check(make_position(pos.x, pos.y, north), 0x03) == false)
                {
                    next_pos.x = pos.x;
                    next_pos.y = pos.y + 1;
                    next_pos.dir = center;
                    update_flag  = true;
                }
            }
        }
    }

    else if(pos.dir == south)
    {
        if(mouse_dir == NorthEast)
        {
            if(pos.x + 1 < MAZE_SIZE_X)
            {
                //if self.wall_data[pos.x][pos.y][Node_pos_dir.East] == False:
                if(wall_check(make_position(pos.x, pos.y, east), 0x03) == false)
                {
                    next_pos.x = pos.x + 1;
                    next_pos.y = pos.y;
                    next_pos.dir = center;
                    update_flag  = true;
                }
            }
        }
        else if(mouse_dir == SouthWest)
        {
            if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
			{
                //if self.wall_data[pos.x-1][pos.y-1][Node_pos_dir.East] == False:
                if(wall_check(make_position(pos.x-1, pos.y-1, east), 0x03) == false)
                {
                    next_pos.x = pos.x - 1;
                    next_pos.y = pos.y - 1;
                    next_pos.dir = center;
                    update_flag  = true;
                }
			}
        }
    }
    else if(pos.dir == west)
    {
        if(mouse_dir == NorthWest)
        {
            if(pos.x - 1 >= 0 && pos.y + 1 < MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x-1][pos.y+1][Node_pos_dir.South] == False:
                if(wall_check(make_position(pos.x-1, pos.y+1, south), 0x03) == false)
                {
                    next_pos.x = pos.x - 1;
                    next_pos.y = pos.y + 1;
                    next_pos.dir = center;
					update_flag  = true;
                }
            }
        }
        else if(mouse_dir == SouthEast)
        {
            if(pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][Node_pos_dir.South] == False:
                if(wall_check(make_position(pos.x, pos.y, south), 0x03) == false)
                {
                    next_pos.x = pos.x;
                    next_pos.y = pos.y - 1;
                    next_pos.dir = center;
                    update_flag  = true;
                }
            }
        }
    }

    if(update_flag == true)
    {
    	t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
    	uint16_t	set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, 0);
		if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
			close_list_set(set_pos,parent_pos,next_mouse_dir,set_run_t,run_pt,0,false);
		}
    }
}

void turn_out_L45_expand(t_position pos,t_eight_dir mouse_dir)
{
    t_eight_dir next_mouse_dir = (mouse_dir - 1 + 8)%8;
    t_position next_pos = make_position(pos.x,pos.y,pos.dir);
    t_position parent_pos = make_position(pos.x,pos.y,pos.dir);
    t_bool update_flag = false;
    t_run_pattern run_pt = Turn_out_L45;

    if(pos.dir == north)
    {
    	if(mouse_dir == NorthWest)
    	{
            if(pos.x - 1 > 0 && pos.y + 1 < MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x-1][pos.y + 1][Node_pos_dir.East] == False:
            	if(wall_check(make_position(pos.x-1,pos.y + 1,east), 0x03) == false)
            	{
                    next_pos.x = pos.x - 1;
                    next_pos.y = pos.y + 1;
                    next_pos.dir = center;
                    update_flag  = true;
            	}
            }
    	}
    	else if(mouse_dir == SouthEast)
    	{
            if(pos.x + 1 < MAZE_SIZE_X)
            {
            	//if self.wall_data[pos.x][pos.y][Node_pos_dir.East] == False:
            	if(wall_check(make_position(pos.x,pos.y,east), 0x03) == false)
            	{
                    next_pos.x = pos.x + 1;
                    next_pos.y = pos.y;
                    next_pos.dir = center;
                    update_flag  = true;
            	}
            }
    	}
    }
    else if(pos.dir == east)
    {
    	if(mouse_dir == NorthEast)
    	{
            if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 < MAZE_SIZE_Y)
            {
            	//if self.wall_data[pos.x+1][pos.y + 1][Node_pos_dir.South] == False:
            	if(wall_check(make_position(pos.x + 1,pos.y + 1,south), 0x03) == false)
            	{
                    next_pos.x = pos.x + 1;
                    next_pos.y = pos.y + 1;
                    next_pos.dir = center;
                    update_flag  = true;
            	}
            }
    	}
    	else if(mouse_dir == SouthWest)
    	{
            if(pos.y - 1 >= 0)
            {
            	//if self.wall_data[pos.x][pos.y][Node_pos_dir.South] == False:
            	if(wall_check(make_position(pos.x,pos.y,south), 0x03) == false)
            	{
                    next_pos.x = pos.x;
                    next_pos.y = pos.y - 1;
                    next_pos.dir = center;
                    update_flag  = true;
            	}
            }
    	}
    }
    else if(pos.dir == south)
    {
    	if(mouse_dir == NorthWest)
    	{
            if(pos.x - 1 >= 0)
            {
            	//if self.wall_data[pos.x][pos.y][Node_pos_dir.West] == False:
            	if(wall_check(make_position(pos.x,pos.y,west), 0x03) == false)
            	{
                    next_pos.x = pos.x - 1;
                    next_pos.y = pos.y;
                    next_pos.dir = center;
                    update_flag  = true;
            	}
            }
    	}
    	else if(mouse_dir == SouthEast)
    	{
            if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 >= 0)
            {
            	//if self.wall_data[pos.x+1][pos.y-1][Node_pos_dir.West] == False:
            	if(wall_check(make_position(pos.x+1,pos.y - 1,west), 0x03) == false)
            	{
                    next_pos.x = pos.x + 1;
                    next_pos.y = pos.y - 1;
                    next_pos.dir = center;
                    update_flag  = true;
            	}
            }
    	}
    }
    else if(pos.dir == west)
    {
    	if(mouse_dir == NorthEast)
    	{
            if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
            {
            	//if self.wall_data[pos.x-1][pos.y-1][Node_pos_dir.North] == False:
            	if(wall_check(make_position(pos.x-1,pos.y - 1,north), 0x03) == false)
            	{
                    next_pos.x = pos.x - 1;
                    next_pos.y = pos.y - 1;
                    next_pos.dir = center;
                    update_flag  = true;
            	}
            }
    	}
    	else if(mouse_dir == SouthWest)
    	{
            if(pos.y + 1 < MAZE_SIZE_Y)
            {
            	//if self.wall_data[pos.x][pos.y][Node_pos_dir.North] == False:
            	if(wall_check(make_position(pos.x,pos.y ,north), 0x03) == false)
            	{
                    next_pos.x = pos.x;
                    next_pos.y = pos.y + 1;
                    next_pos.dir = center;
                    update_flag  = true;
            	}
            }
    	}
    }

    if(update_flag == true)
    {
    	t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
    	uint16_t	set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, 0);
		if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
			close_list_set(set_pos,parent_pos,next_mouse_dir,set_run_t,run_pt,0,false);
		}
    }
}

void turn_in_R135_expand(t_position pos,t_eight_dir mouse_dir)
{
    t_eight_dir next_mouse_dir = (mouse_dir + 3 + 8)%8;
    t_position next_pos = make_position(pos.x,pos.y,pos.dir);
    t_position parent_pos = make_position(pos.x,pos.y,pos.dir);
    t_bool update_flag = false;
    t_run_pattern run_pt = Turn_in_R135;

    if(pos.dir == center)
    {
        if(mouse_dir == North)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 <  MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y+1][(int(mouse_dir/2) + 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y+1, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                {
               	 //if self.wall_data[pos.x+1][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x+1][pos.y][(int(mouse_dir/2)+1+4)%4] == False:
               	 if(wall_check(make_position(pos.x+1, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x+1, pos.y, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
               	 {
               		 next_pos.x = pos.x + 1;
               		 next_pos.y = pos.y;
               		 next_pos.dir = north;
               		 update_flag = true;
               	 }
                }
            }
        }
        else if(mouse_dir == East)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 > 0)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x+1][pos.y][(int(mouse_dir/2) + 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x+1, pos.y, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                {
               	 //if self.wall_data[pos.x][pos.y-1][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y-1][(int(mouse_dir/2)+1+4)%4] == False:
               	 if(wall_check(make_position(pos.x, pos.y-1, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y-1, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
               	 {
						 next_pos.x = pos.x;
						 next_pos.y = pos.y - 1;
						 next_pos.dir = east;
						 update_flag = true;
               	 }
                }
            }
    	 }
        else if(mouse_dir == South)
        {
            if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y-1][(int(mouse_dir/2) + 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y-1, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                {
               	 //if self.wall_data[pos.x-1][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x-1][pos.y][(int(mouse_dir/2)+1+4)%4] == False:
               	 if(wall_check(make_position(pos.x-1, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x-1, pos.y, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
               	 {
						 next_pos.x = pos.x - 1;
						 next_pos.y = pos.y;
						 next_pos.dir = south;
						 update_flag = true;
               	 }
                }
            }
        }
        else if(mouse_dir == West)
        {
            if(pos.x - 1 >= 0 && pos.y + 1 < MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x-1][pos.y][(int(mouse_dir/2) + 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x-1, pos.y, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                {
               	 // if self.wall_data[pos.x][pos.y+1][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y+1][(int(mouse_dir/2)+1+4)%4] == False:
               	 if(wall_check(make_position(pos.x, pos.y+1, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y+1, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false )
               	 {
               		 next_pos.x = pos.x;
               		 next_pos.y = pos.y + 1;
               		 next_pos.dir = west;
               		 update_flag = true;
               	 }
                }
            }
        }
    }

    if(update_flag == true)
    {
    	t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
    	uint16_t	set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, 0);
		if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
			close_list_set(set_pos,parent_pos,next_mouse_dir,set_run_t,run_pt,0,false);
		}
    }
}

void turn_in_L135_expand(t_position pos,t_eight_dir mouse_dir)
{
    t_eight_dir next_mouse_dir = (mouse_dir - 3 + 8)%8;
    t_position next_pos = make_position(pos.x,pos.y,pos.dir);
    t_position parent_pos = make_position(pos.x,pos.y,pos.dir);
    t_bool update_flag = false;
    t_run_pattern run_pt = Turn_in_L135;

    if(pos.dir == center)
    {
        if (mouse_dir == North)
        {
            if(pos.x - 1 >= 0 && pos.y + 1 < MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y+1][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y+1, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	//if self.wall_data[pos.x-1][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x-1][pos.y][(int(mouse_dir/2)-1+4)%4] == False:
                	if(wall_check(make_position(pos.x-1, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x-1, pos.y, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                	{
                		next_pos.x = pos.x - 1;
                		next_pos.y = pos.y;
                		next_pos.dir = north;
                		update_flag = true;
                	}
                }
            }
        }
        else if(mouse_dir == East)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 < MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x+1][pos.y][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x+1, pos.y, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	//if self.wall_data[pos.x][pos.y+1][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y+1][(int(mouse_dir/2)-1+4)%4] == False:
                	if(wall_check(make_position(pos.x, pos.y+1, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y+1, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                	{
						next_pos.x = pos.x;
						next_pos.y = pos.y + 1;
						next_pos.dir = east;
						update_flag = true;
                	}
                }
            }
        }
        else if(mouse_dir == South)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y-1][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y-1, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	//if self.wall_data[pos.x+1][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x+1][pos.y][(int(mouse_dir/2)-1+4)%4] == False:
                	if(wall_check(make_position(pos.x+1, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x+1, pos.y, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                	{
						next_pos.x = pos.x + 1;
						next_pos.y = pos.y;
						next_pos.dir = south;
						update_flag = true;
                	}
                }
            }
        }
        else if(mouse_dir == West)
        {
            if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x-1][pos.y][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x-1, pos.y, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	// if self.wall_data[pos.x][pos.y-1][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y-1][(int(mouse_dir/2)-1+4)%4] == False:
                	if(wall_check(make_position(pos.x, pos.y-1, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y-1, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                	{
                		next_pos.x = pos.x;
                		next_pos.y = pos.y - 1;
                		next_pos.dir = west;
                		update_flag = true;
                	}
                }
            }
        }
    }

    if(update_flag == true)
    {
    	t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
    	uint16_t	set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, 0);
		if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
			close_list_set(set_pos,parent_pos,next_mouse_dir,set_run_t,run_pt,0,false);
		}
    }
}

void turn_out_R135_expand(t_position pos,t_eight_dir mouse_dir)
{
    t_eight_dir next_mouse_dir = (mouse_dir + 3 + 8)%8;
    t_position next_pos = make_position(pos.x,pos.y,pos.dir);
    t_position parent_pos = make_position(pos.x,pos.y,pos.dir);
    t_bool update_flag = false;
    t_run_pattern run_pt = Turn_out_R135;

    if(pos.dir == north)
    {
    	if(mouse_dir == NorthEast)
    	{
    		if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 < MAZE_SIZE_Y)
    		{
    			//if self.wall_data[pos.x+1][pos.y + 1][Node_pos_dir.West] == False and self.wall_data[pos.x+1][pos.y][Node_pos_dir.North] == False:
                if( wall_check(make_position(pos.x+1,pos.y + 1,west),0x03) == false && wall_check(make_position(pos.x+1,pos.y,north),0x03) == false)
                {
					next_pos.x = pos.x + 1;
                    next_pos.y = pos.y;
                    next_pos.dir = center;
                    update_flag = true;
                }
    		}
    	}
    	else if(mouse_dir == SouthWest)
    	{
    		if(pos.x - 1 >= 0 && pos.y + 1 < MAZE_SIZE_Y)
    		{
    			//if self.wall_data[pos.x][pos.y][Node_pos_dir.West] == False and  self.wall_data[pos.x-1][pos.y][Node_pos_dir.North] == False:
                if( wall_check(make_position(pos.x,pos.y,west),0x03) == false && wall_check(make_position(pos.x-1,pos.y,north),0x03) == false)
                {
					next_pos.x = pos.x - 1;
                    next_pos.y = pos.y + 1;
                    next_pos.dir = center;
                    update_flag = true;
                }
    		}
    	}
    }
    else if(pos.dir == east)
    {
    	if(mouse_dir == NorthWest)
    	{
    		if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 < MAZE_SIZE_Y)
    		{
                //if self.wall_data[pos.x][pos.y][Node_pos_dir.North] == False and self.wall_data[pos.x][pos.y + 1][Node_pos_dir.East] == False :
    			if( wall_check(make_position(pos.x,pos.y,north),0x03) == false && wall_check(make_position(pos.x,pos.y + 1,east),0x03) == false)
                {
    				next_pos.x = pos.x + 1;
                    next_pos.y = pos.y + 1;
                    next_pos.dir = center;
                    update_flag = true;

                }
    		}
    	}
    	else if(mouse_dir == SouthEast)
    	{
    		if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 >= 0){
                //if self.wall_data[pos.x+1][pos.y - 1][Node_pos_dir.North] == False and self.wall_data[pos.x][pos.y - 1][Node_pos_dir.East] == False:
    			if( wall_check(make_position(pos.x+1,pos.y-1,north),0x03) == false && wall_check(make_position(pos.x,pos.y - 1,east),0x03) == false)
                {
					next_pos.x = pos.x;
                    next_pos.y = pos.y - 1;
                    next_pos.dir = center;
                    update_flag = true;
                }
    		}
    	}
    }
    else if(pos.dir == south)
    {
    	if(mouse_dir == NorthEast)
    	{
    		if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 >= 0)
    		{
    			// if self.wall_data[pos.x][pos.y][Node_pos_dir.East] == False and self.wall_data[pos.x+1][pos.y][Node_pos_dir.South]:
    			if( wall_check(make_position(pos.x,pos.y,east),0x03) == false && wall_check(make_position(pos.x+1,pos.y,south),0x03) == false)
                {
					next_pos.x = pos.x + 1;
                    next_pos.y = pos.y - 1;
                    next_pos.dir = center;
                    update_flag = true;
                }
    		}
    	}
    	else if(mouse_dir == SouthWest)
    	{
    		if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
    		{
    			//if self.wall_data[pos.x-1][pos.y-1][Node_pos_dir.East] == False and self.wall_data[pos.x-1][pos.y][Node_pos_dir.South] == False:
    			if( wall_check(make_position(pos.x-1,pos.y-1,east),0x03) == false && wall_check(make_position(pos.x-1,pos.y,south),0x03) == false)
                {
					next_pos.x = pos.x - 1;
                    next_pos.y = pos.y;
                    next_pos.dir = center;
                    update_flag = true;
                }
    		}
    	}
    }
    else if(pos.dir == west)
    {
    	if(mouse_dir == NorthWest)
    	{
    		if(pos.x - 1 >= 0 && pos.y + 1 < MAZE_SIZE_Y)
    		{
    			//if self.wall_data[pos.x-1][pos.y+1][Node_pos_dir.South] == False and self.wall_data[pos.x][pos.y+1][Node_pos_dir.West]:
    			if( wall_check(make_position(pos.x-1,pos.y+1,south),0x03) == false && wall_check(make_position(pos.x,pos.y+1,west),0x03) == false)
    			{
                	next_pos.x = pos.x;
                    next_pos.y = pos.y + 1;
                	next_pos.dir = center;
                	update_flag = true;

    			}
    		}
    	}
    	else if(mouse_dir == SouthEast)
    	{
    		if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
    		{
    			//if self.wall_data[pos.x][pos.y][Node_pos_dir.South] == False and self.wall_data[pos.x][pos.y-1][Node_pos_dir.West] :
    			if( wall_check(make_position(pos.x,pos.y,south),0x03) == false && wall_check(make_position(pos.x,pos.y-1,west),0x03) == false)
    			{
                	next_pos.x = pos.x - 1;
                    next_pos.y = pos.y - 1;
                	next_pos.dir = center;
                	update_flag = true;

    			}
    		}
    	}
    }

    if(update_flag == true)
    {
    	t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
    	uint16_t	set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, 0);
		if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
			close_list_set(set_pos,parent_pos,next_mouse_dir,set_run_t,run_pt,0,false);
		}
    }
}

void turn_out_L135_expand(t_position pos,t_eight_dir mouse_dir)
{
    t_eight_dir next_mouse_dir = (mouse_dir - 3 + 8)%8;
    t_position next_pos = make_position(pos.x,pos.y,pos.dir);
    t_position parent_pos = make_position(pos.x,pos.y,pos.dir);
    t_bool update_flag = false;
    t_run_pattern run_pt = Turn_out_L135;

    if(pos.dir == north)
    {
    	if(mouse_dir == NorthWest)
    	{
            if(pos.x - 1 >= 0 && pos.y + 1 < MAZE_SIZE_Y)
            {
				//if self.wall_data[pos.x - 1][pos.y + 1][Node_pos_dir.East] == False and  self.wall_data[pos.x - 1][pos.y][Node_pos_dir.North] == False:
				if( wall_check(make_position(pos.x-1,pos.y+1,east),0x03) == false && wall_check(make_position(pos.x-1,pos.y,north),0x03) == false)
				{
					next_pos.x = pos.x - 1;
					next_pos.y = pos.y;
					next_pos.dir = center;
					update_flag = true;
				}
            }
    	}
    	else if(mouse_dir == SouthEast)
    	{
    		if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 < MAZE_SIZE_Y)
    		{
    			//if self.wall_data[pos.x][pos.y][Node_pos_dir.East] == False and  self.wall_data[pos.x+1][pos.y][Node_pos_dir.North] == False:
				if( wall_check(make_position(pos.x,pos.y,east),0x03) == false && wall_check(make_position(pos.x+1,pos.y,north),0x03) == false)
				{
					next_pos.x = pos.x + 1;
					next_pos.y = pos.y + 1;
					next_pos.dir = center;
					update_flag = true;
				}
    		}
    	}
    }
    else if(pos.dir == east)
    {
    	if(mouse_dir == NorthEast)
    	{
    		if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 < MAZE_SIZE_Y)
    		{
				//if self.wall_data[pos.x+1][pos.y + 1][Node_pos_dir.South] == False and self.wall_data[pos.x][pos.y + 1][Node_pos_dir.East] == False:
				if( wall_check(make_position(pos.x+1,pos.y+1,south),0x03) == false && wall_check(make_position(pos.x,pos.y+1,east),0x03) == false)
				{
					next_pos.x = pos.x;
					next_pos.y = pos.y + 1;
					next_pos.dir = center;
					update_flag = true;
				}
    		}
    	}
    	else if(mouse_dir == SouthWest)
    	{
            if(pos.y - 1 >= 0 && pos.x + 1 < MAZE_SIZE_X)
            {
				//if self.wall_data[pos.x][pos.y][Node_pos_dir.South] == False and self.wall_data[pos.x][pos.y-1][Node_pos_dir.East] == False:
				if(wall_check(make_position(pos.x, pos.y, south),0x03) == false && wall_check(make_position(pos.x, pos.y-1, east),0x03) == false )
				{
					next_pos.x = pos.x + 1;
					next_pos.y = pos.y - 1;
					next_pos.dir = center;
					update_flag = true;
				}
            }
    	}
    }
    else if(pos.dir == south)
    {
    	if(mouse_dir == NorthWest)
    	{
            if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
            {
            //if self.wall_data[pos.x][pos.y][Node_pos_dir.West] == False and  self.wall_data[pos.x-1][pos.y][Node_pos_dir.South] == False:
				if(wall_check(make_position(pos.x, pos.y, west),0x03) == false && wall_check(make_position(pos.x-1, pos.y, south),0x03) == false )
				{
            		next_pos.x = pos.x - 1;
					next_pos.y = pos.y - 1;
					next_pos.dir = center;
					update_flag = true;
				}
            }
    	}
    	else if(mouse_dir == SouthEast)
    	{
            if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x+1][pos.y-1][Node_pos_dir.West] == False and self.wall_data[pos.x+1][pos.y][Node_pos_dir.South] == False:
				if(wall_check(make_position(pos.x+1, pos.y-1, west),0x03) == false && wall_check(make_position(pos.x+1, pos.y, south),0x03) == false )
				{
            		next_pos.x = pos.x + 1;
                    next_pos.y = pos.y;
                    next_pos.dir = center;
                    update_flag = true;
				}
            }
    	}
    }
    else if(pos.dir == west)
    {
    	if(mouse_dir == NorthEast)
    	{
            if(pos.y + 1 < MAZE_SIZE_Y && pos.x - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][Node_pos_dir.North] == False and self.wall_data[pos.x][pos.y+1][Node_pos_dir.West] == False:
				if(wall_check(make_position(pos.x, pos.y, north),0x03) == false && wall_check(make_position(pos.x, pos.y+1, west),0x03) == false )
				{
            		next_pos.x = pos.x - 1;
                    next_pos.y = pos.y + 1;
                    next_pos.dir = center;
                    update_flag = true;
				}
            }
    	}
    	else if(mouse_dir == SouthWest)
    	{
            if(pos.y - 1 >= 0 && pos.x - 1 >= 0)
            {
                //if self.wall_data[pos.x-1][pos.y-1][Node_pos_dir.North] == False and self.wall_data[pos.x][pos.y-1][Node_pos_dir.West] == False:
				if(wall_check(make_position(pos.x-1, pos.y-1, north),0x03) == false && wall_check(make_position(pos.x, pos.y-1, west),0x03) == false )
				{
            		next_pos.x = pos.x;
                    next_pos.y = pos.y - 1;
                    next_pos.dir = center;
                    update_flag = true;
				}
            }
    	}
    }

    if(update_flag == true)
    {
    	t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
    	uint16_t	set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, 0);
		if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
			close_list_set(set_pos,parent_pos,next_mouse_dir,set_run_t,run_pt,0,false);
		}
    }
}

void turn_RV90_expand(t_position pos,t_eight_dir mouse_dir)
{
    t_eight_dir next_mouse_dir = (mouse_dir + 2 + 8)%8;
    t_position next_pos = make_position(pos.x,pos.y,pos.dir);
    t_position parent_pos = make_position(pos.x,pos.y,pos.dir);
    t_bool update_flag = false;
    t_run_pattern run_pt = Turn_RV90;

    if(pos.dir == north)
    {
    	if(mouse_dir == NorthEast)
    	{
    		if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 < MAZE_SIZE_Y)
    		{
    			//if self.wall_data[pos.x+1][pos.y + 1][Node_pos_dir.West] == False and self.wall_data[pos.x+1][pos.y][Node_pos_dir.North] == False:
                if( wall_check(make_position(pos.x+1,pos.y + 1,west),0x03) == false && wall_check(make_position(pos.x+1,pos.y,north),0x03) == false)
                {
                    //if self.wall_data[pos.x+1][pos.y][Node_pos_dir.East] == False:
                	if( wall_check(make_position(pos.x+1,pos.y,east),0x03) == false)
                	{
                		next_pos.x = pos.x + 1;
						next_pos.y = pos.y;
						next_pos.dir = north;
						update_flag = true;
                	}
                }
    		}
    	}
    	else if(mouse_dir == SouthWest)
    	{
    		if(pos.x - 1 >= 0 && pos.y + 1 < MAZE_SIZE_Y)
    		{
    			//if self.wall_data[pos.x][pos.y][Node_pos_dir.West] == False and  self.wall_data[pos.x-1][pos.y][Node_pos_dir.North] == False:
                if( wall_check(make_position(pos.x,pos.y,west),0x03) == false && wall_check(make_position(pos.x-1,pos.y,north),0x03) == false)
                {

                	//if self.wall_data[pos.x-1][pos.y+1][Node_pos_dir.West] == False:
                	if( wall_check(make_position(pos.x-1,pos.y+1,west),0x03) == false)
                	{
                		next_pos.x = pos.x - 1;
                		next_pos.y = pos.y;
                		next_pos.dir = north;
                		update_flag = true;
                	}
                }
    		}
    	}
    }
    else if(pos.dir == east)
    {
    	if(mouse_dir == NorthWest)
    	{
    		if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 < MAZE_SIZE_Y)
    		{
                //if self.wall_data[pos.x][pos.y][Node_pos_dir.North] == False and self.wall_data[pos.x][pos.y + 1][Node_pos_dir.East] == False :
    			if( wall_check(make_position(pos.x,pos.y,north),0x03) == false && wall_check(make_position(pos.x,pos.y + 1,east),0x03) == false)
                {
    				//if self.wall_data[pos.x + 1][pos.y+1][Node_pos_dir.North] == False:
                	if( wall_check(make_position(pos.x+1,pos.y+1,north),0x03) == false)
                	{
    					next_pos.x = pos.x;
    					next_pos.y = pos.y + 1;
    					next_pos.dir = east;
    					update_flag = true;
                	}

                }
    		}
    	}
    	else if(mouse_dir == SouthEast)
    	{
    		if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 >= 0){
                //if self.wall_data[pos.x+1][pos.y - 1][Node_pos_dir.North] == False and self.wall_data[pos.x][pos.y - 1][Node_pos_dir.East] == False:
    			if( wall_check(make_position(pos.x+1,pos.y-1,north),0x03) == false && wall_check(make_position(pos.x,pos.y - 1,east),0x03) == false)
                {
                    //if self.wall_data[pos.x][pos.y-1][Node_pos_dir.South] == False:
                	if( wall_check(make_position(pos.x,pos.y-1,south),0x03) == false)
                	{
						next_pos.x = pos.x;
						next_pos.y = pos.y - 1;
						next_pos.dir = east;
						update_flag = true;
                	}
                }
    		}
    	}
    }
    else if(pos.dir == south)
    {
    	if(mouse_dir == NorthEast)
    	{
    		if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 >= 0)
    		{
    			// if self.wall_data[pos.x][pos.y][Node_pos_dir.East] == False and self.wall_data[pos.x+1][pos.y][Node_pos_dir.South]:
    			if( wall_check(make_position(pos.x,pos.y,east),0x03) == false && wall_check(make_position(pos.x+1,pos.y,south),0x03) == false)
                {
    				//if self.wall_data[pos.x+1][pos.y-1][Node_pos_dir.East] == False:
    				if( wall_check(make_position(pos.x+1,pos.y-1,east),0x03) == false)
    				{
    					next_pos.x = pos.x + 1;
    					next_pos.y = pos.y;
    					next_pos.dir = south;
    					update_flag = true;
    				}
                }
    		}
    	}
    	else if(mouse_dir == SouthWest)
    	{
    		if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
    		{
    			//if self.wall_data[pos.x-1][pos.y-1][Node_pos_dir.East] == False and self.wall_data[pos.x-1][pos.y][Node_pos_dir.South] == False:
    			if( wall_check(make_position(pos.x-1,pos.y-1,east),0x03) == false && wall_check(make_position(pos.x-1,pos.y,south),0x03) == false)
                {
    				//if self.wall_data[pos.x-1][pos.y][Node_pos_dir.West] == False:
    				if( wall_check(make_position(pos.x-1,pos.y,west),0x03) == false)
    				{
						next_pos.x = pos.x - 1;
                    	next_pos.y = pos.y;
                    	next_pos.dir = south;
                    	update_flag = true;
    				}
                }
    		}
    	}
    }
    else if(pos.dir == west)
    {
    	if(mouse_dir == NorthWest)
    	{
    		if(pos.x - 1 >= 0 && pos.y + 1 < MAZE_SIZE_Y)
    		{
    			//if self.wall_data[pos.x-1][pos.y+1][Node_pos_dir.South] == False and self.wall_data[pos.x][pos.y+1][Node_pos_dir.West]:
    			if( wall_check(make_position(pos.x-1,pos.y+1,south),0x03) == false && wall_check(make_position(pos.x,pos.y+1,west),0x03) == false)
    			{
    				//if self.wall_data[pos.x][pos.y+1][Node_pos_dir.North] == False:
    				if( wall_check(make_position(pos.x,pos.y+1,north),0x03) == false)
    				{
    					next_pos.x = pos.x;
                		next_pos.y = pos.y + 1;
                		next_pos.dir = west;
                		update_flag = true;
    				}
    			}
    		}
    	}
    	else if(mouse_dir == SouthEast)
    	{
    		if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
    		{
    			//if self.wall_data[pos.x][pos.y][Node_pos_dir.South] == False and self.wall_data[pos.x][pos.y-1][Node_pos_dir.West] :
    			if( wall_check(make_position(pos.x,pos.y,south),0x03) == false && wall_check(make_position(pos.x,pos.y-1,west),0x03) == false)
    			{
    				if( wall_check(make_position(pos.x-1,pos.y-1,south),0x03) == false)
    				{
    					next_pos.x = pos.x;
    					next_pos.y = pos.y - 1;
    					next_pos.dir = west;
    					update_flag = true;
    				}

    			}
    		}
    	}
    }

    if(update_flag == true)
    {
    	t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
    	uint16_t	set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, 0);
		if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
			close_list_set(set_pos,parent_pos,next_mouse_dir,set_run_t,run_pt,0,false);
		}
    }
}

void turn_LV90_expand(t_position pos,t_eight_dir mouse_dir)
{
    t_eight_dir next_mouse_dir = (mouse_dir - 2 + 8)%8;
    t_position next_pos = make_position(pos.x,pos.y,pos.dir);
    t_position parent_pos = make_position(pos.x,pos.y,pos.dir);
    t_bool update_flag = false;
    t_run_pattern run_pt = Turn_LV90;

    if(pos.dir == north)
    {
    	if(mouse_dir == NorthWest)
    	{
            if(pos.x - 1 >= 0 && pos.y + 1 < MAZE_SIZE_Y)
            {
				//if self.wall_data[pos.x - 1][pos.y + 1][Node_pos_dir.East] == False and  self.wall_data[pos.x - 1][pos.y][Node_pos_dir.North] == False:
				if( wall_check(make_position(pos.x-1,pos.y+1,east),0x03) == false && wall_check(make_position(pos.x-1,pos.y,north),0x03) == false)
				{
					if( wall_check(make_position(pos.x-1,pos.y,west),0x03) == false)
					{
						next_pos.x = pos.x - 1;
						next_pos.y = pos.y;
						next_pos.dir = north;
						update_flag = true;
					}
				}
            }
    	}
    	else if(mouse_dir == SouthEast)
    	{
    		if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 < MAZE_SIZE_Y)
    		{
    			//if self.wall_data[pos.x][pos.y][Node_pos_dir.East] == False and  self.wall_data[pos.x+1][pos.y][Node_pos_dir.North] == False:
				if( wall_check(make_position(pos.x,pos.y,east),0x03) == false && wall_check(make_position(pos.x+1,pos.y,north),0x03) == false)
				{
					if( wall_check(make_position(pos.x+1,pos.y+1,east),0x03) == false)
					{
						next_pos.x = pos.x + 1;
						next_pos.y = pos.y;
						next_pos.dir = north;
						update_flag = true;
					}
				}
    		}
    	}
    }
    else if(pos.dir == east)
    {
    	if(mouse_dir == NorthEast)
    	{
    		if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 < MAZE_SIZE_Y)
    		{
				//if self.wall_data[pos.x+1][pos.y + 1][Node_pos_dir.South] == False and self.wall_data[pos.x][pos.y + 1][Node_pos_dir.East] == False:
				if( wall_check(make_position(pos.x+1,pos.y+1,south),0x03) == false && wall_check(make_position(pos.x,pos.y+1,east),0x03) == false)
				{
					if( wall_check(make_position(pos.x,pos.y+1,north),0x03) == false)
					{
						next_pos.x = pos.x;
						next_pos.y = pos.y + 1;
						next_pos.dir = east;
						update_flag = true;
					}
				}
    		}
    	}
    	else if(mouse_dir == SouthWest)
    	{
            if(pos.y - 1 >= 0 && pos.x + 1 < MAZE_SIZE_X)
            {
				//if self.wall_data[pos.x][pos.y][Node_pos_dir.South] == False and self.wall_data[pos.x][pos.y-1][Node_pos_dir.East] == False:
				if(wall_check(make_position(pos.x, pos.y, south),0x03) == false && wall_check(make_position(pos.x, pos.y-1, east),0x03) == false )
				{
					if( wall_check(make_position(pos.x+1,pos.y-1,south),0x03) == false)
					{
						next_pos.x = pos.x;
						next_pos.y = pos.y - 1;
						next_pos.dir = east;
						update_flag = true;
					}
				}
            }
    	}
    }
    else if(pos.dir == south)
    {
    	if(mouse_dir == NorthWest)
    	{
            if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
            {
            //if self.wall_data[pos.x][pos.y][Node_pos_dir.West] == False and  self.wall_data[pos.x-1][pos.y][Node_pos_dir.South] == False:
				if(wall_check(make_position(pos.x, pos.y, west),0x03) == false && wall_check(make_position(pos.x-1, pos.y, south),0x03) == false )
				{
					if( wall_check(make_position(pos.x-1,pos.y-1,west),0x03) == false)
					{
						next_pos.x = pos.x - 1;
						next_pos.y = pos.y;
						next_pos.dir = north;
						update_flag = true;
					}
				}
            }
    	}
    	else if(mouse_dir == SouthEast)
    	{
            if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x+1][pos.y-1][Node_pos_dir.West] == False and self.wall_data[pos.x+1][pos.y][Node_pos_dir.South] == False:
				if(wall_check(make_position(pos.x+1, pos.y-1, west),0x03) == false && wall_check(make_position(pos.x+1, pos.y, south),0x03) == false )
				{
					if( wall_check(make_position(pos.x+1,pos.y,east),0x03) == false)
					{
						next_pos.x = pos.x + 1;
						next_pos.y = pos.y;
						next_pos.dir = north;
						update_flag = true;
					}
				}
            }
    	}
    }
    else if(pos.dir == west)
    {
    	if(mouse_dir == NorthEast)
    	{
            if(pos.y + 1 < MAZE_SIZE_Y && pos.x - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][Node_pos_dir.North] == False and self.wall_data[pos.x][pos.y+1][Node_pos_dir.West] == False:
				if(wall_check(make_position(pos.x, pos.y, north),0x03) == false && wall_check(make_position(pos.x, pos.y+1, west),0x03) == false )
				{
					if( wall_check(make_position(pos.x-1,pos.y+1,north),0x03) == false)
					{
						next_pos.x = pos.x;
						next_pos.y = pos.y + 1;
						next_pos.dir = west;
						update_flag = true;
					}
				}
            }
    	}
    	else if(mouse_dir == SouthWest)
    	{
            if(pos.y - 1 >= 0 && pos.x - 1 >= 0)
            {
                //if self.wall_data[pos.x-1][pos.y-1][Node_pos_dir.North] == False and self.wall_data[pos.x][pos.y-1][Node_pos_dir.West] == False:
				if(wall_check(make_position(pos.x-1, pos.y-1, north),0x03) == false && wall_check(make_position(pos.x, pos.y-1, west),0x03) == false )
				{
					if( wall_check(make_position(pos.x,pos.y-1,south),0x03) == false)
					{
						next_pos.x = pos.x;
						next_pos.y = pos.y - 1;
						next_pos.dir = west;
						update_flag = true;
					}
				}
            }
    	}
    }

    if(update_flag == true)
    {
    	t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
    	uint16_t	set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, 0);
		if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
			close_list_set(set_pos,parent_pos,next_mouse_dir,set_run_t,run_pt,0,false);
		}
    }
}

void long_turn_R90_expand(t_position pos,t_eight_dir mouse_dir)
{
    t_eight_dir next_mouse_dir = (mouse_dir + 2 + 8)%8;
    t_position next_pos = make_position(pos.x,pos.y,pos.dir);
    t_position parent_pos = make_position(pos.x,pos.y,pos.dir);
    t_bool update_flag = false;
    t_run_pattern run_pt = Long_turnR90;
    if(pos.dir == center)
    {
        if(mouse_dir == North)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 <  MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y+1][(int(mouse_dir/2) + 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y+1, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                {
               		next_pos.x = pos.x + 1;
               		next_pos.y = pos.y + 1;
               	    next_pos.dir = center;
               		update_flag = true;
                }
            }
        }
        else if(mouse_dir == East)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 > 0)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x+1][pos.y][(int(mouse_dir/2) + 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x+1, pos.y, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                {
					next_pos.x = pos.x + 1;
					next_pos.y = pos.y - 1;
					next_pos.dir = center;
					update_flag = true;
                }
            }
    	 }
        else if(mouse_dir == South)
        {
            if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y-1][(int(mouse_dir/2) + 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y-1, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                {
                	next_pos.x = pos.x - 1;
					next_pos.y = pos.y - 1;
					next_pos.dir = center;
					update_flag = true;
                }
            }
        }
        else if(mouse_dir == West)
        {
            if(pos.x - 1 >= 0 && pos.y + 1 < MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x-1][pos.y][(int(mouse_dir/2) + 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x-1, pos.y, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                {
               		next_pos.x = pos.x - 1;
               		next_pos.y = pos.y + 1;
               		next_pos.dir = center;
               		update_flag = true;

                }
            }
        }
    }
    if(update_flag == true)
    {
    	t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
    	uint16_t	set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, 0);
		if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
			close_list_set(set_pos,parent_pos,next_mouse_dir,set_run_t,run_pt,0,false);
		}
    }
}

void long_turn_L90_expand(t_position pos,t_eight_dir mouse_dir)
{
    t_eight_dir next_mouse_dir = (mouse_dir - 2 + 8)%8;
    t_position next_pos = make_position(pos.x,pos.y,pos.dir);
    t_position parent_pos = make_position(pos.x,pos.y,pos.dir);
    t_bool update_flag = false;
    t_run_pattern run_pt = Long_turnL90;

    if(pos.dir == center)
    {
        if (mouse_dir == North)
        {
            if(pos.x - 1 >= 0 && pos.y + 1 < MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y+1][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y+1, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	next_pos.x = pos.x - 1;
                	next_pos.y = pos.y + 1;
               		next_pos.dir = center;
               		update_flag = true;
                }
            }
        }
        else if(mouse_dir == East)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 < MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x+1][pos.y][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x+1, pos.y, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	//if self.wall_data[pos.x+1][pos.y+1][int(mouse_dir/2)] == False:
					next_pos.x = pos.x + 1;
					next_pos.y = pos.y + 1;
					next_pos.dir = center;
					update_flag = true;
                }
            }
        }
        else if(mouse_dir == South)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y-1][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y-1, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	//if self.wall_data[pos.x+1][pos.y-1][int(mouse_dir/2)] == False:
					next_pos.x = pos.x + 1;
					next_pos.y = pos.y - 1;
					next_pos.dir = center;
					update_flag = true;
                }
            }
        }
        else if(mouse_dir == West)
        {
            if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x-1][pos.y][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x-1, pos.y, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	//if self.wall_data[pos.x-1][pos.y-1][int(mouse_dir/2)] == False:
               		next_pos.x = pos.x - 1;
               		next_pos.y = pos.y - 1;
                	next_pos.dir = center;
                	update_flag = true;
                }
            }
        }
    }

    if(update_flag == true)
    {
    	t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
    	uint16_t	set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, 0);
		if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
			close_list_set(set_pos,parent_pos,next_mouse_dir,set_run_t,run_pt,0,false);
		}
    }
}

void long_turn_R180_expand(t_position pos,t_eight_dir mouse_dir)
{
    t_eight_dir next_mouse_dir = (mouse_dir + 4 + 8)%8;
    t_position next_pos = make_position(pos.x,pos.y,pos.dir);
    t_position parent_pos = make_position(pos.x,pos.y,pos.dir);
    t_bool update_flag = false;
    t_run_pattern run_pt = Long_turnR180;

    if(pos.dir == center)
    {
        if(mouse_dir == North)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 <  MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y+1][(int(mouse_dir/2) + 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y+1, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                {
               	 //if self.wall_data[pos.x+1][pos.y][int(mouse_dir/2)] == False:
               	 if(wall_check(make_position(pos.x+1, pos.y, mouse_dir/2), 0x03) == false)
               	 {
               		 next_pos.x = pos.x + 1;
               		 next_pos.y = pos.y;
               		 next_pos.dir = center;
               		 update_flag = true;
               	 }
                }
            }
        }
        else if(mouse_dir == East)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x+1][pos.y][(int(mouse_dir/2) + 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x+1, pos.y, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                {
               	 //if self.wall_data[pos.x][pos.y-1][int(mouse_dir/2)] == False:
               	 if(wall_check(make_position(pos.x, pos.y-1, mouse_dir/2), 0x03) == false)
               	 {
						 next_pos.x = pos.x;
						 next_pos.y = pos.y - 1;
						 next_pos.dir = center;
						 update_flag = true;
               	 }
                }
            }
    	 }
        else if(mouse_dir == South)
        {
            if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y-1][(int(mouse_dir/2) + 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y-1, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                {
               	 //if self.wall_data[pos.x-1][pos.y][int(mouse_dir/2)] == False:
               	 if(wall_check(make_position(pos.x-1, pos.y, mouse_dir/2), 0x03) == false)
               	 {
						 next_pos.x = pos.x - 1;
						 next_pos.y = pos.y;
						 next_pos.dir = center;
						 update_flag = true;
               	 }
                }
            }
        }
        else if(mouse_dir == West)
        {
            if(pos.x - 1 >= 0 && pos.y + 1 < MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x-1][pos.y][(int(mouse_dir/2) + 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x-1, pos.y, ((mouse_dir/2) + 1 + 4)%4), 0x03) == false)
                {
               	 // if self.wall_data[pos.x][pos.y+1][int(mouse_dir/2)] == False:
               	 if(wall_check(make_position(pos.x, pos.y+1, mouse_dir/2), 0x03) == false)
               	 {
               		 next_pos.x = pos.x;
               		 next_pos.y = pos.y + 1;
               		 next_pos.dir = center;
               		 update_flag = true;
               	 }
                }
            }
        }
    }

    if(update_flag == true)
    {
    	t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
    	uint16_t	set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, 0);
		if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
			close_list_set(set_pos,parent_pos,next_mouse_dir,set_run_t,run_pt,0,false);
		}
    }
}

void long_turn_L180_expand(t_position pos,t_eight_dir mouse_dir)
{
    t_eight_dir next_mouse_dir = (mouse_dir - 4 + 8)%8;
    t_position next_pos = make_position(pos.x,pos.y,pos.dir);
    t_position parent_pos = make_position(pos.x,pos.y,pos.dir);
    t_bool update_flag = false;
    t_run_pattern run_pt = Long_turnL180;

    if(pos.dir == center)
    {
        if (mouse_dir == North)
        {
            if(pos.x - 1 >= 0 && pos.y + 1 < MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y+1][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y+1, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	//if self.wall_data[pos.x-1][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x-1][pos.y][(int(mouse_dir/2)-1+4)%4] == False:
                	if(wall_check(make_position(pos.x-1, pos.y, mouse_dir/2), 0x03) == false)
                	{
                		next_pos.x = pos.x - 1;
                		next_pos.y = pos.y;
                		next_pos.dir = center;
                		update_flag = true;
                	}
                }
            }
        }
        else if(mouse_dir == East)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y + 1 < MAZE_SIZE_Y)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x+1][pos.y][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x+1, pos.y, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	//if self.wall_data[pos.x][pos.y+1][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y+1][(int(mouse_dir/2)-1+4)%4] == False:
                	if(wall_check(make_position(pos.x, pos.y+1, mouse_dir/2), 0x03) == false)
                	{
						next_pos.x = pos.x;
						next_pos.y = pos.y + 1;
                		next_pos.dir = center;
						update_flag = true;
                	}
                }
            }
        }
        else if(mouse_dir == South)
        {
            if(pos.x + 1 < MAZE_SIZE_X && pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y-1][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x, pos.y-1, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	//if self.wall_data[pos.x+1][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x+1][pos.y][(int(mouse_dir/2)-1+4)%4] == False:
                	if(wall_check(make_position(pos.x+1, pos.y, mouse_dir/2), 0x03) == false)
                	{
						next_pos.x = pos.x + 1;
						next_pos.y = pos.y;
                		next_pos.dir = center;
						update_flag = true;
                	}
                }
            }
        }
        else if(mouse_dir == West)
        {
            if(pos.x - 1 >= 0 && pos.y - 1 >= 0)
            {
                //if self.wall_data[pos.x][pos.y][int(mouse_dir/2)] == False and self.wall_data[pos.x-1][pos.y][(int(mouse_dir/2) - 1 + 4)%4] == False:
                if(wall_check(make_position(pos.x, pos.y, mouse_dir/2), 0x03) == false && wall_check(make_position(pos.x-1, pos.y, ((mouse_dir/2) - 1 + 4)%4), 0x03) == false)
                {
                	// if self.wall_data[pos.x][pos.y-1][int(mouse_dir/2)] == False and self.wall_data[pos.x][pos.y-1][(int(mouse_dir/2)-1+4)%4] == False:
                	if(wall_check(make_position(pos.x, pos.y-1, mouse_dir/2), 0x03) == false)
                	{
                		next_pos.x = pos.x;
                		next_pos.y = pos.y - 1;
                		next_pos.dir = center;
                		update_flag = true;
                	}
                }
            }
        }
    }

    if(update_flag == true)
    {
    	t_position set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
    	uint16_t	set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, 0);
		if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
			close_list_set(set_pos,parent_pos,next_mouse_dir,set_run_t,run_pt,0,false);
		}
    }
}

void north_expand(t_position pos)
{
	t_eight_dir mouse_dir = return_mouse_dir(pos);
	if(mouse_dir == NorthEast)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_R45_expand(pos, mouse_dir);
		turn_out_R135_expand(pos, mouse_dir);
		turn_RV90_expand(pos, mouse_dir);
	}
	else if(mouse_dir == SouthEast)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_L45_expand(pos, mouse_dir);
		turn_out_L135_expand(pos, mouse_dir);
		turn_LV90_expand(pos, mouse_dir);
	}
	else if(mouse_dir == SouthWest)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_R45_expand(pos, mouse_dir);
		turn_out_R135_expand(pos, mouse_dir);
		turn_RV90_expand(pos, mouse_dir);
	}
	else if(mouse_dir == NorthWest)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_L45_expand(pos, mouse_dir);
		turn_out_L135_expand(pos, mouse_dir);
		turn_LV90_expand(pos, mouse_dir);
	}
}

void east_expand(t_position pos)
{
	t_eight_dir mouse_dir = return_mouse_dir(pos);
	if(mouse_dir == NorthEast)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_L45_expand(pos, mouse_dir);
		turn_out_L135_expand(pos, mouse_dir);
		turn_LV90_expand(pos, mouse_dir);
	}
	else if(mouse_dir == SouthEast)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_R45_expand(pos, mouse_dir);
		turn_out_R135_expand(pos, mouse_dir);
		turn_RV90_expand(pos, mouse_dir);
	}
	else if(mouse_dir == SouthWest)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_L45_expand(pos, mouse_dir);
		turn_out_L135_expand(pos, mouse_dir);
		turn_LV90_expand(pos, mouse_dir);
	}
	else if(mouse_dir == NorthWest)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_R45_expand(pos, mouse_dir);
		turn_out_R135_expand(pos, mouse_dir);
		turn_RV90_expand(pos, mouse_dir);
	}
}

void south_expand(t_position pos)
{
	t_eight_dir mouse_dir = return_mouse_dir(pos);
	if(mouse_dir == NorthEast)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_R45_expand(pos, mouse_dir);
		turn_out_R135_expand(pos, mouse_dir);
		turn_RV90_expand(pos, mouse_dir);
	}
	else if(mouse_dir == SouthEast)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_L45_expand(pos, mouse_dir);
		turn_out_L135_expand(pos, mouse_dir);
		turn_LV90_expand(pos, mouse_dir);
	}
	else if(mouse_dir == SouthWest)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_R45_expand(pos, mouse_dir);
		turn_out_R135_expand(pos, mouse_dir);
		turn_RV90_expand(pos, mouse_dir);
	}
	else if(mouse_dir == NorthWest)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_L45_expand(pos, mouse_dir);
		turn_out_L135_expand(pos, mouse_dir);
		turn_LV90_expand(pos, mouse_dir);
	}
}

void west_expand(t_position pos)
{
	t_eight_dir mouse_dir = return_mouse_dir(pos);
	if(mouse_dir == NorthEast)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_L45_expand(pos, mouse_dir);
		turn_out_L135_expand(pos, mouse_dir);
		turn_LV90_expand(pos, mouse_dir);
	}
	else if(mouse_dir == SouthEast)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_R45_expand(pos, mouse_dir);
		turn_out_R135_expand(pos, mouse_dir);
		turn_RV90_expand(pos, mouse_dir);
	}
	else if(mouse_dir == SouthWest)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_L45_expand(pos, mouse_dir);
		turn_out_L135_expand(pos, mouse_dir);
		turn_LV90_expand(pos, mouse_dir);
	}
	else if(mouse_dir == NorthWest)
	{
		diagonal_expand(pos, mouse_dir);
		turn_out_R45_expand(pos, mouse_dir);
		turn_out_R135_expand(pos, mouse_dir);
		turn_RV90_expand(pos, mouse_dir);
	}
}

void center_expand(t_position pos)
{
#if DEBUG_MODE == 1
	printf("CENTER_EXPAND\n");
	HAL_Delay(1);
#endif
	t_eight_dir mouse_dir = return_mouse_dir(pos);
    straight_expand(pos,mouse_dir);
    long_turn_L90_expand(pos,mouse_dir);
    turn_in_L45_expand(pos,mouse_dir);
    turn_in_L135_expand(pos,mouse_dir);
    long_turn_L180_expand(pos,mouse_dir);
    long_turn_R90_expand(pos,mouse_dir);
    turn_in_R45_expand(pos,mouse_dir);
    turn_in_R135_expand(pos,mouse_dir);
    long_turn_R180_expand(pos,mouse_dir);
}

void expand_dijkstra(t_position pos)
{
#if DEBUG_MODE == 1
	printf("EXPAND\n");
	HAL_Delay(1);
#endif
	switch(pos.dir)
	{
		case north:
			north_expand(pos);
			break;
		case east:
			east_expand(pos);
			break;
		case south:
			south_expand(pos);
			break;
		case west:
			west_expand(pos);
			break;
		case center:
			center_expand(pos);
			break;
	}
}


t_position last_straight_expand(t_position pos, int *gx,int *gy,int goal_size)
{
	t_position next_pos 	= make_position(pos.x, pos.y, pos.dir);
	t_position parent_pos	= make_position(pos.x, pos.y, pos.dir);
	t_position set_pos		= make_position(pos.x, pos.y, pos.dir);
	t_eight_dir mouse_dir	= return_mouse_dir(pos);
	t_run_pattern run_pt	= Straight;
	if(pos.dir == center)
	{
		if(mouse_dir == North || mouse_dir == South)
		{
			for(int i = 0;i < MAZE_SIZE_Y;i++)
			{
				if(wall_check(make_position(next_pos.x, next_pos.x, (mouse_dir/2)), 0x01) == false)
				{
					if(mouse_dir == North)
					{
						next_pos.y = next_pos.y + 1;
					}
					else if(mouse_dir == South)
					{
						next_pos.y = next_pos.y - 1;
					}

                    if(is_goal_dijkstra(set_pos,gx,gy,goal_size) == true && is_goal_dijkstra(next_pos,gx,gy,goal_size) == false)
                    {
                        break;
                    }

					if(next_pos.y < MAZE_SIZE_Y &&next_pos.y >= 0)
					{
						set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
						uint16_t set_run_t = pos_run_time(parent_pos) + calc_time(run_pt,i+1);
						if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
							close_list_set(set_pos,parent_pos,mouse_dir,set_run_t,run_pt,(i+1),false);
						}
					}
					else
					{
						break;
					}
				}
				else
				{
					break;
				}
			}
		}
		else if(mouse_dir == East || mouse_dir == West)
		{
			for(int i = 0;i < MAZE_SIZE_X;i++)
			{
				if(wall_check(make_position(next_pos.x, next_pos.y, (mouse_dir/2)), 0x01) == false)
				{
					if(mouse_dir == East)
					{
						next_pos.x = next_pos.x + 1;
					}
					else if(mouse_dir == West)
					{
						next_pos.x = next_pos.x - 1;
					}

                    if(is_goal_dijkstra(set_pos,gx,gy,goal_size) == true && is_goal_dijkstra(next_pos,gx,gy,goal_size) == false)
                    {
                        break;
                    }

					if(next_pos.x < MAZE_SIZE_X && next_pos.x >= 0)
					{
						set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
						uint16_t set_run_t = pos_run_time(parent_pos) + calc_time(run_pt,i+1);
						if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
							close_list_set(set_pos,parent_pos,mouse_dir,set_run_t,run_pt,(i+1),false);
						}
					}
					else
					{
						break;
					}

				}
				else
				{
					break;
				}
			}
		}
	}
	return set_pos;

}

t_position last_diagonal_expand(t_position pos, int *gx,int *gy,int goal_size)
{
	t_position next_pos 	= make_position(pos.x, pos.y, pos.dir);
	t_position set_pos		= make_position(pos.x, pos.y, pos.dir);
	t_position parent_pos	= make_position(pos.x, pos.y, pos.dir);
	t_run_pattern run_pt	= Diagonal;
	t_eight_dir mouse_dir	= return_mouse_dir(pos);
	int run_cnt = 0;
	if(pos_run_pt(pos) == Diagonal)
	{
		parent_pos = pos_parent(pos);
		next_pos 	= make_position(parent_pos.x, parent_pos.y, parent_pos.dir);
		set_pos		= make_position(parent_pos.x, parent_pos.y, parent_pos.dir);
	}

	for(int i = 0;i < MAZE_SIZE_X + MAZE_SIZE_Y;i++)
	{
		run_cnt = i;
		if(wall_check(next_pos, 0x03) != false) break;
		set_pos = make_position(next_pos.x, next_pos.y, next_pos.dir);
        if(next_pos.dir == north)
        {
            if(mouse_dir == NorthEast)
            {
                next_pos.x = next_pos.x + 1;
                next_pos.y = next_pos.y + 1;
            }
            else if(mouse_dir == SouthEast)
            {
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
            }
            else if(mouse_dir == SouthWest)
            {
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
            }
            else if(mouse_dir ==  NorthWest)
            {
                next_pos.x = next_pos.x - 1;
                next_pos.y = next_pos.y + 1;
            }
        }

        else if(next_pos.dir == east)
        {
            if(mouse_dir ==  NorthEast)
            {
                next_pos.x = next_pos.x + 1;
                next_pos.y = next_pos.y + 1;
            }
			else if( mouse_dir ==  SouthEast)
            {
                next_pos.x = next_pos.x + 1;
                next_pos.y = next_pos.y - 1;
            }
            else if( mouse_dir ==  SouthWest)
            {
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
            }
            else if( mouse_dir ==  NorthWest)
            {
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
            }
        }

        else if( next_pos.dir == south)
        {
            if (mouse_dir ==  NorthEast)
			{
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
			}
			else if( mouse_dir ==  SouthEast)
			{
                next_pos.x = next_pos.x + 1;
                next_pos.y = next_pos.y - 1;
			}
			else if( mouse_dir ==  SouthWest)
			{
                next_pos.x = next_pos.x - 1;
                next_pos.y = next_pos.y - 1;
			}
            else if( mouse_dir ==  NorthWest)
            {
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
            }
        }
        else if( next_pos.dir == west)
        {
            if(mouse_dir ==  NorthEast)
            {
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
            }
            else if( mouse_dir ==  SouthEast)
            {
                next_pos.x = next_pos.x;
                next_pos.y = next_pos.y;
            }
            else if( mouse_dir ==  SouthWest)
            {
                next_pos.x = next_pos.x - 1;
                next_pos.y = next_pos.y - 1;
            }
			else if( mouse_dir ==  NorthWest)
			{
                next_pos.x = next_pos.x - 1;
                next_pos.y = next_pos.y + 1 ;
			}
        }
        next_pos.dir = (next_pos.dir + 2*(next_pos.dir%2) + (8 + 4) - mouse_dir)%4;

        if(wall_check(next_pos, 0x03) == false && i != 0)
        {
        	int set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, i);
			if(pos_run_time(set_pos) > set_run_t && is_determine(set_pos) == false){
				close_list_set(set_pos,parent_pos,mouse_dir,set_run_t,run_pt,i,false);
			}
        }

        //if(self.is_goal(Pos(next_pos_x,next_pos_y,next_pos_d),gx,gy) == False and self.is_goal(Pos(set_pos_x,set_pos_y,set_pos_d),gx,gy) == True):
           // break
        if(is_goal_dijkstra(set_pos,gx,gy,goal_size) == true && is_goal_dijkstra(next_pos,gx,gy,goal_size) == false)
        {
        	break;
        }

	}
	run_cnt = run_cnt - 1;
	int set_run_t = pos_run_time(parent_pos) + calc_time(run_pt, run_cnt);
	if(pos_run_time(set_pos) > set_run_t)
	{
		close_list_set(set_pos,parent_pos,mouse_dir,set_run_t,run_pt,run_cnt,false);
	}
	return set_pos;
}

t_position last_expand(t_position pos, int *gx,int *gy,int goal_size)
{
	t_position last_pos = make_position(pos.x, pos.y, pos.dir);
	switch(pos.dir)
	{
		case center:
			last_pos = last_straight_expand(pos, gx, gy, goal_size);
			break;
		case north:
		case east:
		case south:
		case west:
			last_pos = last_diagonal_expand(pos, gx, gy, goal_size);
			break;
	}
	return last_pos;
}

/*
 * dijkstra.c
 *
 *  Created on: 2022/08/25
 *      Author: sato1
 */
#include "index.h"
#include "glob_var.h"
#include "run_param.h"
#include "dijkstra.h"
#include "queue.h"
#include "kalman_filter.h"

t_position make_position(uint8_t x,uint8_t y,t_direction dir)
{
	t_position pos;
	pos.x	= x;
	pos.y   = y;
	pos.dir = dir;
	return pos;
}

t_node_el set_node(t_position pos,t_position parent,t_eight_dir mouse_dir,uint16_t time,t_run_pattern run_pattern,uint16_t acc_cnt,t_bool determine)
{
	t_node_el node;
	node.pos = pos;
	node.parent = parent;
	node.mouse_dir = mouse_dir;
	node.time = time;
	node.run_pt = run_pattern;
	node.acc_cnt = acc_cnt;
	node.determine = determine;
	return node;
}

t_bool wall_check(t_position pos,int mask)
{
	t_bool is_wall = true;
	switch(pos.dir){
		case north:
			if((wall[pos.x][pos.y].north & mask) == NOWALL)
			{
				is_wall = false;
			}
			break;
		case east:
			if((wall[pos.x][pos.y].east & mask) == NOWALL)
			{
				is_wall = false;
			}
			break;
		case south:
			if((wall[pos.x][pos.y].south & mask) == NOWALL)
			{
				is_wall = false;
			}
			break;
		case west:
			if((wall[pos.x][pos.y].west & mask) == NOWALL)
			{
				is_wall = false;
			}
			break;
		case center:
			break;
	}
	return is_wall;
}

void init_dijkstra_map(){
	for(int i = 0;i < MAZE_SIZE_X;i++){
		for(int j = 0;j < MAZE_SIZE_Y;j++){
			for(int d = 0; d < 5;d++){
				switch(d){
					case north:
						closed_list[i][j].north = set_node(make_position(i, j, north), make_position(i, j, north), Dir_None, UINT16_MAX-1, run_pt_none, 0, false);
						break;
					case east:
						closed_list[i][j].east = set_node(make_position(i, j, east), make_position(i, j, east), Dir_None, UINT16_MAX-1, run_pt_none, 0, false);
						break;
					case south:
						closed_list[i][j].south = set_node(make_position(i, j, south), make_position(i, j, south), Dir_None, UINT16_MAX-1, run_pt_none, 0, false);
						break;
					case west:
						closed_list[i][j].west = set_node(make_position(i, j, west), make_position(i, j, west), Dir_None, UINT16_MAX-1, run_pt_none, 0, false);
						break;
					case center:
						closed_list[i][j].center = set_node(make_position(i, j, center), make_position(i, j, center), Dir_None, UINT16_MAX-1, run_pt_none, 0, false);
						break;
				}
			}
		}
	}
}

void start_node_setup(t_position pos,t_eight_dir mouse_dir){
	switch(pos.dir){
		case north:
			closed_list[pos.x][pos.y].north 	= set_node(pos,pos,mouse_dir,0,run_pt_none,0,false);
			if(pos.y + 1 < MAZE_SIZE_Y){
				closed_list[pos.x][pos.y + 1].south = set_node(make_position(pos.x,pos.y+1,south),make_position(pos.x,pos.y+1,south),mouse_dir,0,run_pt_none,0,false);
			}
			break;
		case east:
			closed_list[pos.x][pos.y].east  	= set_node(pos,pos,mouse_dir,0,run_pt_none,0,false);
			if(pos.x + 1 < MAZE_SIZE_X){
				closed_list[pos.x + 1][pos.y].west = set_node(make_position(pos.x+1,pos.y,west),make_position(pos.x+1,pos.y,west),mouse_dir,0,run_pt_none,0,false);
			}
			break;
		case south:
			closed_list[pos.x][pos.y].south 	= set_node(pos,pos,mouse_dir,0,run_pt_none,0,false);
			if(pos.y - 1 >= 0){
				closed_list[pos.x][pos.y - 1].north = set_node(make_position(pos.x,pos.y-1,north),make_position(pos.x,pos.y-1,north),mouse_dir,0,run_pt_none,0,false);
			}
			break;
		case west:
			closed_list[pos.x][pos.y].west  	= set_node(pos,pos,mouse_dir,0,run_pt_none,0,false);
			if(pos.x - 1 >= 0){
				closed_list[pos.x - 1][pos.y].east = set_node(make_position(pos.x-1,pos.y,east),make_position(pos.x-1,pos.y,east),mouse_dir,0,run_pt_none,0,false);
			}
			break;
		case center:
			closed_list[pos.x][pos.y].center	= set_node(pos,pos,mouse_dir,0,run_pt_none,0,false);
			break;
	}
}

void close_list_set(t_position pos,t_position parent,t_eight_dir mouse_dir,uint16_t time,t_run_pattern run_pt,uint16_t acc_cnt,t_bool determine)
{
	switch(pos.dir){
		case north:
			closed_list[pos.x][pos.y].north 	= set_node(pos,parent,mouse_dir,time,run_pt,acc_cnt,determine);
			if(pos.y + 1 < MAZE_SIZE_Y){
				closed_list[pos.x][pos.y + 1].south = set_node(make_position(pos.x,pos.y+1,south),parent,mouse_dir,time,run_pt,acc_cnt,determine);
			}
			break;
		case east:
			closed_list[pos.x][pos.y].east  	= set_node(pos,parent,mouse_dir,time,run_pt,acc_cnt,determine);
			if(pos.x + 1 < MAZE_SIZE_X){
				closed_list[pos.x+1][pos.y].west = set_node(make_position(pos.x+1,pos.y,west),parent,mouse_dir,time,run_pt,acc_cnt,determine);
			}
			break;
		case south:
			closed_list[pos.x][pos.y].south 	= set_node(pos,parent,mouse_dir,time,run_pt,acc_cnt,determine);
			if(pos.y - 1 >= 0)
			{
				closed_list[pos.x][pos.y-1].north = set_node(make_position(pos.x,pos.y-1,north),parent,mouse_dir,time,run_pt,acc_cnt,determine);
			}
			break;
		case west:
			closed_list[pos.x][pos.y].west  	= set_node(pos,parent,mouse_dir,time,run_pt,acc_cnt,determine);
			if(pos.y - 1 >= 0)
			{
				closed_list[pos.x-1][pos.y].east = set_node(make_position(pos.x-1,pos.y,east),parent,mouse_dir,time,run_pt,acc_cnt,determine);
			}
			break;
		case center:
			closed_list[pos.x][pos.y].center 	= set_node(pos,parent,mouse_dir,time,run_pt,acc_cnt,determine);
			break;
	}
}

void set_determine(t_position pos){
	switch(pos.dir){
		case north:
			closed_list[pos.x][pos.y].north.determine 			= true;
			if(pos.y + 1 < MAZE_SIZE_Y)
			{
				closed_list[pos.x][pos.y + 1].south.determine 	= true;
			}
			break;
		case east:
			closed_list[pos.x][pos.y].east.determine 			= true;
			if(pos.x + 1 < MAZE_SIZE_X)
			{
				closed_list[pos.x + 1][pos.y].west.determine 	= true;
			}
			break;
		case south:
			closed_list[pos.x][pos.y].south.determine 			= true;
			if(pos.y - 1 >= 0)
			{
				closed_list[pos.x][pos.y-1].north.determine 	= true;
			}
			break;
		case west:
			closed_list[pos.x][pos.y].west.determine 			= true;
			if(pos.x - 1 >= 0)
			{
				closed_list[pos.x-1][pos.y].east.determine 		= true;
			}
			break;
		case center:
			closed_list[pos.x][pos.y].center.determine			= true;
			break;
	}
}

t_position min_search(){
	t_position min_pos;
	uint16_t   time = 60000;
	min_pos.x 	= 0;
	min_pos.y 	= 0;
	min_pos.dir	= center;
	for(int i = 0;i < MAZE_SIZE_X;i++){
		for(int j = 0;j < MAZE_SIZE_Y;j++){
			for(int d = 0; d < 5;d++){
				switch(d){
					case north:
						if(closed_list[i][j].north.time < time && closed_list[i][j].north.determine == false){
							min_pos = closed_list[i][j].north.pos;
							time 	= closed_list[i][j].north.time;
						}
						break;
					case east:
						if(closed_list[i][j].east.time < time && closed_list[i][j].east.determine == false){
							min_pos = closed_list[i][j].east.pos;
							time 	= closed_list[i][j].east.time;
						}
						break;
					case south:
						if(closed_list[i][j].south.time < time && closed_list[i][j].south.determine == false){
							min_pos = closed_list[i][j].south.pos;
							time 	= closed_list[i][j].south.time;
						}
						break;
					case west:
						if(closed_list[i][j].west.time < time && closed_list[i][j].west.determine == false){
							min_pos = closed_list[i][j].west.pos;
							time 	= closed_list[i][j].west.time;
						}
						break;
					case center:
						if(closed_list[i][j].center.time < time &&  closed_list[i][j].center.determine == false){
							min_pos = closed_list[i][j].center.pos;
							time 	= closed_list[i][j].center.time;
						}
						break;
				}
			}
		}
	}
	return min_pos;
}

t_bool is_goal_dijkstra(t_position pos,int *gx,int *gy,int goal_size){
	t_bool flag = false;
	uint8_t x = pos.x;
	uint8_t y = pos.y;
	t_direction d = pos.dir;
	for(int i = 0;i < goal_size;i++){
		for(int j = 0;j < goal_size;j++){
			if(flag == true) continue;

			if((int)x == gx[i] && (int)y == gy[j]){
				flag = true;
			}
			else if(d == north && (int)y < (MAZE_SIZE_Y - 1)){
				if((int)x == gx[i] && (int)(y + 1) == gy[j])	flag = true;
			}
			else if(d == east && (int)x < (MAZE_SIZE_X - 1)){
				if((int)(x + 1) == gx[i] && (int)y == gy[j])	flag = true;
			}
			else if(d == south && (int)y > 0){
				if((int)x == gx[i] && (int)(y - 1) == gy[j])	flag = true;
			}
			else if(d == west && x > 0){
				if((int)(x - 1) == gx[i] && (int)y == gy[j])	flag = true;
			}
		}
	}
	return flag;
}

void print_closed_list_info(t_position pos)
{
	switch(pos.dir)
	{
		case north:
			printf("n:time:%d\n",closed_list[pos.x][pos.y].center.time);
			break;
		case east:
			printf("e:time:%d\n",closed_list[pos.x][pos.y].center.time);
			break;
		case south:
			printf("s:time:%d\n",closed_list[pos.x][pos.y].center.time);
			break;
		case west:
			printf("w:time:%d\n",closed_list[pos.x][pos.y].center.time);
			break;
		case center:
			printf("c:time:%d\n",closed_list[pos.x][pos.y].center.time);
			break;
	}
}

t_position make_map_dijkstra(t_position start_pos,t_eight_dir start_mouse_dir,int *gx,int *gy,int goal_size){
	t_position min_pos;
	init_dijkstra_map();
#if DEBUG_MODE == 1
		printf("start:x:%d,y:%d,dir:%d\n",start_pos.x,start_pos.y,start_pos.dir);
		HAL_Delay(1);
		print_closed_list_info(start_pos);
		HAL_Delay(1);
#endif
	start_node_setup(start_pos, start_mouse_dir);
#if DEBUG_MODE == 1
		printf("start:x:%d,y:%d,dir:%d\n",start_pos.x,start_pos.y,start_pos.dir);
		print_closed_list_info(start_pos);
		HAL_Delay(1);
#endif
	for(int i = 0;i < 1000;i++){
		min_pos = min_search();
		set_determine(min_pos);
		if(is_goal_dijkstra(min_pos, gx, gy, goal_size) == true)
		{
			min_pos = last_expand(min_pos, gx,gy,goal_size);
			break;
		}
#if DEBUG_MODE == 1
		t_node_el node_el = return_close_list_el(min_pos);
		printf("x:%d,y:%d,dir:%d,run_pt:%d,Dir:%d,Acc_Cnt:%d\n",min_pos.x,min_pos.y,min_pos.dir,pos_run_pt(min_pos),return_mouse_dir(min_pos),node_el.acc_cnt);
		HAL_Delay(1);
		print_closed_list_info(min_pos);
#endif
		expand_dijkstra(min_pos);
	}
	return min_pos;
}



void run_print(const t_straight_param *const *st_mode,uint16_t size_st_mode,
			   const t_straight_param *const *di_mode,uint16_t size_di_mode,
			   const t_param *const *turn_mode,
			   t_position start_pos,t_eight_dir start_mouse_dir,int *gx,int *gy,int goal_size)
{

	Initialize_st_param_for_time(st_mode, size_st_mode);
	Initialize_di_param_for_time(di_mode, size_di_mode);
	Initialize_turn_time(turn_mode);

	t_position p_pos = make_map_dijkstra(start_pos,start_mouse_dir,gx,gy,goal_size);

	for(int i = 0;i<100;i++){
#if DEBUG_MODE == 1
		t_node_el node_el = return_close_list_el(p_pos);
		printf("x:%d,y:%d,dir:%d,run_pt:%d,acc_cnt:%d\n",p_pos.x,p_pos.y,p_pos.dir,pos_run_pt(p_pos),node_el.acc_cnt);
#endif
		HAL_Delay(3);
		if(pos_run_pt(p_pos) == run_pt_none) break;
		p_pos = pos_parent(p_pos);
	}

}

void run_dijkstra(const t_straight_param *const *st_mode,uint16_t size_st_mode,
			   const t_straight_param *const *di_mode,uint16_t size_di_mode,
			   const t_param *const *turn_mode,
			   t_position start_pos,t_eight_dir start_mouse_dir,int *gx,int *gy,int goal_size)
{

	Initialize_st_param_for_time(st_mode, size_st_mode);
	Initialize_di_param_for_time(di_mode, size_di_mode);
	Initialize_turn_time(turn_mode);

	t_position p_pos = make_map_dijkstra(start_pos,start_mouse_dir,gx,gy,goal_size);

	int16_t log_run_tail = 0;
	//t_position log_run_pos[MAZE_SIZE];

	for(int i = 0;i < MAZE_SIZE;i++)
	{
		log_run_pos[i] = start_pos;
	}

	for(int i = 0;i<MAZE_SIZE;i++){
		log_run_pos[i] = p_pos;
		log_run_tail++;
		if(pos_run_pt(p_pos) == run_pt_none) break;
		p_pos = pos_parent(p_pos);
	}

	Sp_Param_Initialize(&machine);
	Sp_Param_Initialize(&target);
	Sp_Param_Initialize(&max_set);

	Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
	wall_controll.is_controll = true;
	//float st_acc = st_mode[0]->param->max_velo*st_mode[0]->param->max_velo/(2*0.020);
	straight(20.0, 10.0, 0.3,0.3);
	log_flag = true;time = 0;
	run_mode = NON_CON_MODE;
	for(int i = log_run_tail ;i >= 0;i--)
	{ 	filter_init();
		t_node_el node_el = return_close_list_el(log_run_pos[i]);
		t_straight_param velo_param;
		switch(node_el.run_pt)
		{
			case Straight:
				wall_controll.is_controll = true;
				//velo_param = calc_straight_max_velo(st_mode, size_st_mode,SECTION*(float)node_el.acc_cnt);
				//Set_Omega_PID_Gain(velo_param.om_gain->Kp, velo_param.om_gain->Ki, velo_param.om_gain->Kd);
				//Set_Velo_PID_Gain(velo_param.sp_gain->Kp, velo_param.sp_gain->Ki, velo_param.sp_gain->Kd);
				if(i == 0)
				{
					velo_param = calc_end_straight_max_velo(st_mode, size_st_mode,SECTION*(float)node_el.acc_cnt);
					Set_Omega_PID_Gain(velo_param.om_gain->Kp, velo_param.om_gain->Ki, velo_param.om_gain->Kd);
					Set_Velo_PID_Gain(velo_param.sp_gain->Kp, velo_param.sp_gain->Ki, velo_param.sp_gain->Kd);
					straight(SECTION*(float)node_el.acc_cnt, velo_param.param->acc, velo_param.param->max_velo, 0.0);
					break;
				}
				else
				{
					velo_param = calc_straight_max_velo(st_mode, size_st_mode,SECTION*(float)node_el.acc_cnt);
					Set_Omega_PID_Gain(velo_param.om_gain->Kp, velo_param.om_gain->Ki, velo_param.om_gain->Kd);
					Set_Velo_PID_Gain(velo_param.sp_gain->Kp, velo_param.sp_gain->Ki, velo_param.sp_gain->Kd);
					straight(SECTION*(float)node_el.acc_cnt, velo_param.param->acc, velo_param.param->max_velo, st_mode[0]->param->max_velo);
				}
				break;
			case Diagonal:
				wall_controll.is_controll = true;
				//velo_param = calc_straight_max_velo(di_mode, size_di_mode,DIAG_SECTION*(float)node_el.acc_cnt);
				//Set_Omega_PID_Gain(velo_param.om_gain->Kp, velo_param.om_gain->Ki, velo_param.om_gain->Kd);
				//Set_Velo_PID_Gain(velo_param.sp_gain->Kp, velo_param.sp_gain->Ki, velo_param.sp_gain->Kd);
				if(i == 0)
				{
					velo_param = calc_end_straight_max_velo(di_mode, size_di_mode,DIAG_SECTION*(float)node_el.acc_cnt);
					Set_Omega_PID_Gain(velo_param.om_gain->Kp, velo_param.om_gain->Ki, velo_param.om_gain->Kd);
					Set_Velo_PID_Gain(velo_param.sp_gain->Kp, velo_param.sp_gain->Ki, velo_param.sp_gain->Kd);
					diagonal(DIAG_SECTION*(float)node_el.acc_cnt, velo_param.param->acc, velo_param.param->max_velo, 0.0);
				}
				else
				{
					velo_param = calc_straight_max_velo(di_mode, size_di_mode,DIAG_SECTION*(float)node_el.acc_cnt);
					Set_Omega_PID_Gain(velo_param.om_gain->Kp, velo_param.om_gain->Ki, velo_param.om_gain->Kd);
					Set_Velo_PID_Gain(velo_param.sp_gain->Kp, velo_param.sp_gain->Ki, velo_param.sp_gain->Kd);
					diagonal(DIAG_SECTION*(float)node_el.acc_cnt, velo_param.param->acc, velo_param.param->max_velo, di_mode[0]->param->max_velo);
				}
				break;
			case Long_turnL90:
			case Long_turnR90:
			case Long_turnL180:
			case Long_turnR180:
				//wall_controll.is_controll = false;
				long_turn(turn_mode[node_el.run_pt],st_mode[0]);
				break;
			case Turn_in_L45:
			case Turn_in_R45:
			case Turn_in_L135:
			case Turn_in_R135:
				//wall_controll.is_controll = false;
				turn_in(turn_mode[node_el.run_pt],st_mode[0],di_mode[0]);
				break;
			case Turn_out_L45:
			case Turn_out_R45:
			case Turn_out_L135:
			case Turn_out_R135:
				//wall_controll.is_controll = false;
				turn_out(turn_mode[node_el.run_pt],st_mode[0],di_mode[0]);
				break;
			case Turn_RV90:
			case Turn_LV90:
				//wall_controll.is_controll = false;
				turn_v90(turn_mode[node_el.run_pt],di_mode[0]);
				break;
			case Diagonal_L:
			case Diagonal_R:
			case run_pt_none:
				break;
		}
	}
	//straight(5.0, 6.0, 0.3, 0.0);
	run_mode = NON_CON_MODE;
	wall_controll.is_controll = false;

}


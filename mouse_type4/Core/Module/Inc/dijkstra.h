/*
 * dijkstra.h
 *
 *  Created on: 2022/08/25
 *      Author: sato1
 */

#ifndef MODULE_INC_DIJKSTRA_H_
#define MODULE_INC_DIJKSTRA_H_

#include "index.h"
#include "macro.h"
#include "run_param.h"

t_position make_position(uint8_t x,uint8_t y,t_direction dir);
t_node_el set_node(t_position pos,t_position parent,t_eight_dir mouse_dir,uint16_t time,t_run_pattern run_pattern,uint16_t acc_cnt,t_bool determine);
t_bool wall_check(t_position pos,int mask);
void init_dijkstra_map();
void start_node_setup(t_position pos,t_eight_dir mouse_dir);
void close_list_set(t_position pos,t_position parent,t_eight_dir mouse_dir,uint16_t time,t_run_pattern run_pt,uint16_t acc_cnt,t_bool determine);
void set_determine(t_position pos);
t_position min_search();
t_bool is_goal_dijkstra(t_position pos,int *gx,int *gy,int goal_size);
t_position make_map_dijkstra(t_position start_pos,t_eight_dir start_mouse_dir,int *gx,int *gy,int goal_size);
void expand_dijkstra(t_position pos);
t_position last_expand(t_position pos, int *gx,int *gy,int goal_size);

t_position pos_parent(t_position pos);
t_run_pattern pos_run_pt(t_position pos);
t_eight_dir return_mouse_dir(t_position pos);
t_node_el return_close_list_el(t_position pos);

void Initialize_turn_time(const t_param *const *mode);
void Initialize_st_param_for_time(const t_straight_param *const *st_mode,uint16_t size_st_mode);
void Initialize_di_param_for_time(const t_straight_param *const *di_mode,uint16_t size_di_mode);
void run_print(const t_straight_param *const *st_mode,uint16_t size_st_mode,
			   const t_straight_param *const *di_mode,uint16_t size_di_mode,
			   const t_param *const *turn_mode,
			   t_position start_pos,t_eight_dir start_mouse_dir,int *gx,int *gy,int goal_size);

void run_dijkstra(const t_straight_param *const *st_mode,uint16_t size_st_mode,
			   const t_straight_param *const *di_mode,uint16_t size_di_mode,
			   const t_param *const *turn_mode,
			   t_position start_pos,t_eight_dir start_mouse_dir,int *gx,int *gy,int goal_size);

uint16_t calc_straight_time(const t_straight_param *const *mode,uint16_t mode_size,float length);
t_straight_param calc_straight_max_velo(const t_straight_param *const *mode,uint16_t mode_size,float length);
t_straight_param calc_end_straight_max_velo(const t_straight_param *const *mode,uint16_t mode_size,float length);
#endif /* MODULE_INC_DIJKSTRA_H_ */

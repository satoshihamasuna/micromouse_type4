/*
 * operation_check.c
 *
 *  Created on: 2022/08/14
 *      Author: sato1
 */
#include "index.h"
#include "run_param.h"
#include "glob_var.h"
void slalom_check_R90()
{
		 log_flag = true;time = 0;
		 Sp_Param_Initialize(&machine);
		 Sp_Param_Initialize(&target);
		 Sp_Param_Initialize(&max_set);
		 straight(45.0f, 6.0f, 0.6f, 0.6f);
		 search_turn90_table(&param_R90_ext_search);
		 straight(45.0f, 6.0f, 0.6f, 0.0f);
		 log_flag = false;
		 run_mode = NON_CON_MODE;
}
void slalom_check_L90()
{
	 	 log_flag = true;time = 0;
	 	 Sp_Param_Initialize(&machine);
	 	 Sp_Param_Initialize(&target);
	 	 Sp_Param_Initialize(&max_set);
	 	 straight(45.0f, 4.0f, 0.3f, 0.3f);
	 	 search_turn90_table(&param_L90_search);
	 	 straight(45.0f, 4.0f, 0.3f, 0.0f);
	 	 log_flag = false;
	 	 run_mode = NON_CON_MODE;
}

void slalom_fast_L90()
{
	 	 log_flag = true;time = 0;
	 	 Sp_Param_Initialize(&machine);
	 	 Sp_Param_Initialize(&target);
	 	 Sp_Param_Initialize(&max_set);
	 	 straight(90.0f, 10.0f, 1.0f, 1.0f);
	 	 //turn90(&slalom_L90_v3);
	 	 turn90_table(&slalom_L90_table_v3);
	 	 straight(90.0f, 10.0f, 1.0f, 0.0f);
	 	 log_flag = false;
}

void fast_run_check()
{
		 log_flag = true;time = 0;
		 Sp_Param_Initialize(&machine);
		 Sp_Param_Initialize(&target);
		 Sp_Param_Initialize(&max_set);
		 straight(270.0f, 15.0f, 2.0f, 0.0f);
		 log_flag = false;
}

void slalom_check(const t_straight_param* velo_param,const t_param *const *mode,t_run_pattern run_pt)
{
	 log_flag = true;time = 0;
	 Sp_Param_Initialize(&machine);
	 Sp_Param_Initialize(&target);
	 Sp_Param_Initialize(&max_set);
	 Set_Velo_PID_Gain(velo_param->sp_gain->Kp,velo_param->sp_gain->Ki,velo_param->sp_gain->Kd);
	 Set_Omega_PID_Gain(velo_param->om_gain->Kp,velo_param->om_gain->Ki,velo_param->om_gain->Kd);

	 float acc = velo_param->param->acc;
	 float velo = velo_param->param->max_velo;
	 switch(run_pt)
	 {
	 	 case Straight:
	 		 straight(90.0f, acc, velo, velo);
	 		 straight(90.0f, acc, velo, 0.0f);
	 	 case Diagonal:
	 		 diagonal(DIAG_SECTION*4, acc, velo, velo);
	 		 diagonal(DIAG_SECTION*1, acc, velo, 0.0);
	 	 case Diagonal_L:
	 	 case Diagonal_R:
	 	 case run_pt_none:
	 		 break;
	 	 case Long_turnL90:
	 	 case Long_turnR90:
	 	 case Long_turnL180:
	 	 case Long_turnR180:
	 		 straight(90.0f, acc, velo, velo);
	 		 long_turn(mode[run_pt],velo_param);
	 		 straight(90.0f, acc, velo, 0.0f);
	 		 break;
	 	 case Turn_in_L45:
	 	 case Turn_in_R45:
	 	 case Turn_in_L135:
	 	 case Turn_in_R135:
	 		 straight(90.0f, acc, velo, velo);
	 		 turn_in(mode[run_pt],velo_param,velo_param);
	 		 diagonal(DIAG_SECTION*2, acc, velo, 0.0);
	 		 break;
	 	 case Turn_out_L45:
	 	 case Turn_out_R45:
	 	 case Turn_out_L135:
	 	 case Turn_out_R135:
	 		 diagonal(DIAG_SECTION*2, acc, velo, velo);
	 		 turn_out(mode[run_pt],velo_param,velo_param);
	 		 straight(90.0f, acc, velo, 0.0f);
	 		 break;
	 	 case Turn_LV90:
	 	 case Turn_RV90:
	 		 diagonal(DIAG_SECTION*2, acc, velo, velo);
	 		 turn_v90(mode[run_pt],velo_param);
	 		 diagonal(DIAG_SECTION*2, acc, velo, 0.0);
	 		 break;
	 }

 	 //straight(90.0f, acc, velo, 0.0f);
 	 log_flag = false;
}

void enkaigei(uint16_t time)
{
	uint16_t time_cnt;
	run_mode = STRAIGHT_MODE;
	Sp_Param_Initialize(&machine);
	Sp_Param_Initialize(&target);
	Sp_Param_Initialize(&max_set);
	for(time_cnt = 0; time_cnt < time;time_cnt++) HAL_Delay(2);
}


void wall_controll_check(uint16_t time)
{
	run_mode = STRAIGHT_MODE;
	Sp_Param_Initialize(&machine);
	Sp_Param_Initialize(&target);
	Sp_Param_Initialize(&max_set);
	Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.1, 0.0, 0.0);
	wall_controll.is_controll = true;
	for(int time_cnt = 0; time_cnt < time;time_cnt++){
		printf("%.2f\n",target.rad_velo);
		HAL_Delay(2);
	}
	wall_controll.is_controll = false;
	run_mode = NON_CON_MODE;
}

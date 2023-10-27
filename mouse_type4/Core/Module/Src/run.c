/*
 * run.c
 *
 *  Created on: Aug 13, 2022
 *      Author: sato1
 */

#include "index.h"
#include "glob_var.h"
#include "run_param.h"
#include "rad_accel_table.h"


void Sp_Param_I_Initialize(t_sp_param * sp_param){
	sp_param->I_velo 		= 0.0f;
	sp_param->I_rad_velo 	= 0.0f;
	//sp_param->radian		= 0.0f;
	//sp_param->length        = 0.0f;
	rad_Integral_controller = 0.0f;
}

void Sp_Param_rad_Initialize(t_sp_param * sp_param){
	sp_param->radian = 0.0f;
}


void Sp_Param_Initialize(t_sp_param *sp_param){
	sp_param->accel			= 0.0f;
	sp_param->velo 			= 0.0f;
	sp_param->rad_accel 	= 0.0f;
	sp_param->rad_velo 		= 0.0f;
	sp_param->I_velo		= 0.0f;
	sp_param->I_rad_velo	= 0.0f;
	sp_param->length 		= 0.0f;
	sp_param->radian 		= 0.0f;
}

void Machine_Param_Initialize(){
	machine.accel		= 0.0f;
	machine.velo 		= 0.0f;
    machine.rad_accel 	= 0.0f;
    machine.rad_velo 	= 0.0f;
    machine.I_velo		= 0.0f;
    machine.I_rad_velo	= 0.0f;
    machine.length 		= 0.0f;
    machine.radian 		= 0.0f;
}

void Target_Param_Initialize(){
	target.accel		= 0.0f;
	target.velo 		= 0.0f;
	target.rad_accel 	= 0.0f;
	target.rad_velo 	= 0.0f;
    target.I_velo		= 0.0f;
    target.I_rad_velo	= 0.0f;
	target.length 		= 0.0f;
	target.radian 		= 0.0f;
}

void MAX_Param_Initialize(){
	max_set.accel		= 0.0f;
	max_set.velo 		= 0.0f;
	max_set.rad_accel 	= 0.0f;
	max_set.rad_velo 	= 0.0f;
    max_set.I_velo		= 0.0f;
    max_set.I_rad_velo	= 0.0f;
	max_set.length 		= 0.0f;
	max_set.radian 		= 0.0f;
}

void Set_Velo_PID_Gain(float Kp,float Ki,float Kd){
	velo_g.Kp = Kp;
	velo_g.Ki = Ki;
	velo_g.Kd = Kd;
}

void Set_Omega_PID_Gain(float Kp,float Ki,float Kd){
	omega_g.Kp = Kp;
	omega_g.Ki = Ki;
	omega_g.Kd = Kd;
}

void Set_PID_Gain(t_pid_gain *pid_gain,float Kp,float Ki,float Kd)
{
	pid_gain->Kp = Kp;
	pid_gain->Ki = Ki;
	pid_gain->Kd = Kd;
}

void straight(float len_target,float acc,float max_sp,float end_sp){
	//Machine_Param_Initialize();
	//Target_Param_Initialize();
	//MAX_Param_Initialize();

	Sp_Param_I_Initialize(&machine);
	Sp_Param_I_Initialize(&target);
	if(end_sp == 0.0f)
	{
		run_mode = STRAIGHT_MODE;

		max_set.velo = max_sp;
		int wait_time = (int)(max_sp/3.0*1000.0);
		max_set.accel = acc;
		target.accel = acc;
		max_set.length = len_target;

		while(max_set.length- machine.length > 1000.0 * (max_set.velo *max_set.velo- end_sp*end_sp)/(2.0*max_set.accel));
		target.accel = -acc;
		while(machine.length < max_set.length){
			if(target.velo < 0.05){
				target.accel = 0.0;target.velo = 0.05;
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

		while(max_set.length - OFF_SET_LENGTH - machine.length > 1000.0 * (max_set.velo *max_set.velo- end_sp*end_sp)/(2.0*max_set.accel));
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

void diagonal(float len_target,float acc,float max_sp,float end_sp){
	//Machine_Param_Initialize();
	//Target_Param_Initialize();
	//MAX_Param_Initialize();

	Sp_Param_I_Initialize(&machine);
	Sp_Param_I_Initialize(&target);
	//Set_Omega_PID_Gain(0.6f, 0.01f, 0.0f);
	//Set_Velo_PID_Gain(12.0f, 0.1f, 0.0f);
	machine.radian		= 0.0f;
	target.radian		= 0.0f;
	diag_predict_xr = diag_predict_xl = DIAG_HALF_SECTION;
	if(end_sp == 0.0f)
	{
		run_mode = DIAG_MODE;

		max_set.velo = max_sp;
		int wait_time = (int)(max_sp/3.0*1000.0);
		max_set.accel = acc;
		target.accel = acc;
		max_set.length = len_target;

		while(max_set.length - machine.length > 1000.0 * (max_set.velo *max_set.velo- end_sp*end_sp)/(2.0*max_set.accel));
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
		run_mode = DIAG_MODE;

		max_set.velo = max_sp;
		max_set.accel = acc;
		target.accel = acc;
		max_set.length = len_target;

		while(max_set.length - machine.length > 1000.0 * (max_set.velo *max_set.velo- end_sp*end_sp)/(2.0*max_set.accel));
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

void search_straight(float len_target,float acc,float max_sp,float end_sp){
	//Machine_Param_Initialize();
	//Target_Param_Initialize();
	//MAX_Param_Initialize();

	Sp_Param_I_Initialize(&machine);
	Sp_Param_I_Initialize(&target);

	if(end_sp == 0.0f)
	{
		run_mode = STRAIGHT_MODE;

		max_set.velo = max_sp;
		int wait_time = (int)(max_sp/3.0*1000.0);
		max_set.accel = acc;
		target.accel = acc;
		max_set.length = len_target;

		while(max_set.length - machine.length > 1000.0 * (max_set.velo *max_set.velo - end_sp*end_sp)/(2.0*max_set.accel));
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

void Spin_turn(float rad_target,float rad_acc,float max_rad_velo,t_turn_dir turn_dir){

	Machine_Param_Initialize();
	Target_Param_Initialize();
	MAX_Param_Initialize();

	if(turn_dir == turn_left)
	{
		run_mode = TURN_MODE;
		max_set.rad_velo = max_rad_velo;
		target.rad_accel = rad_acc;
		max_set.rad_accel= rad_acc;
		max_set.radian   = rad_target;
	}
	else if(turn_dir == turn_right)
	{
		run_mode = TURN_MODE;
		max_set.rad_velo = -max_rad_velo;
		target.rad_accel = -rad_acc;
		max_set.rad_accel= -rad_acc;
		max_set.radian   = -rad_target;
	}

	while(ABS(max_set.radian) - ABS(machine.radian) > ABS((max_set.rad_velo*max_set.rad_velo)/(2.0*max_set.rad_accel)));

	if(turn_dir == turn_left)
	{
		target.rad_accel = -rad_acc;
		while(ABS(max_set.radian) > ABS(machine.radian))
		{
			if(target.rad_velo <= 0.0 ){
				max_set.rad_velo = 0.0f;
				target.rad_accel = 0.0f;
				break;
			}
		}
		max_set.rad_velo = 0.0f;
		target.rad_accel = 0.0f;
	}
	else if(turn_dir == turn_right)
	{
		target.rad_accel = rad_acc;
		while(ABS(max_set.radian) > ABS(machine.radian))
		{
			if(target.rad_velo >= 0.0 ){
				max_set.rad_velo = 0.0f;
				target.rad_accel = 0.0f;
				break;
			}
		}
		max_set.rad_velo = 0.0f;
		target.rad_accel = 0.0f;
	}

	HAL_Delay(100);
	Machine_Param_Initialize();
	Target_Param_Initialize();
	MAX_Param_Initialize();
	machine.radian = 0.0f;
	run_mode = NON_CON_MODE;

}

void turn90(const t_turn_param* param)
{

	Sp_Param_I_Initialize(&target);
	Sp_Param_I_Initialize(&machine);
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);

	run_mode = STRAIGHT_MODE;

	max_set.velo = param->velo;
	target.velo  = param->velo;
	max_set.length = param->Lstart;
	while(machine.length < param->Lstart );


	run_mode = TURN_MODE;
	max_set.rad_velo = param->omega;
	target.rad_accel = param->omega_acc;
	max_set.rad_accel= param->omega_acc;
	max_set.radian   = DEG2RAD(param->degree);
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);

	while(ABS(max_set.radian) - ABS(target.radian) > ABS((max_set.rad_velo*max_set.rad_velo)/(2.0*max_set.rad_accel)));

	target.rad_accel = (-1.0f)*param->omega_acc;

	while(ABS(max_set.radian) >= ABS(target.radian))
	{
		if(param->turn_dir == turn_left)
		{
			if(target.rad_velo <= 0.0f)
			{
				max_set.rad_velo = 0.0f;
				target.rad_velo = 0.0f;
				target.rad_accel = 0.0f;
				break;
			}
		}
		else if(param->turn_dir == turn_right)
		{
			if(target.rad_velo >= 0.0f)
			{
				max_set.rad_velo = 0.0f;
				target.rad_velo = 0.0f;
				target.rad_accel = 0.0f;
				break;
			}
		}

	}


	run_mode = STRAIGHT_MODE;
	machine.length	= target.length	= 0.0f;
	max_set.velo	= target.velo	= param->velo;
	max_set.length = param->Lend;
	while(machine.length < max_set.length );

	machine.length = target.length  = 0.0f;


}

void turn90_table(const t_turn_param_table* param)
{

	Sp_Param_I_Initialize(&target);
	Sp_Param_I_Initialize(&machine);
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);

	run_mode = STRAIGHT_MODE;

	max_set.velo = param->velo;
	target.velo  = param->velo;
	max_set.length = param->Lstart;
	while(machine.length < param->Lstart ){
		if((90.0 - sen_fr.distance) > param->Lstart &&  (90.0 - sen_fl.distance) > param->Lstart) break;
	}


	run_mode = TURN_MODE_TABLE;
	max_set.rad_velo = param->velo/(param->r_min/1000);							//rad\s
	set_turn_time = DEG2RAD(param->degree)/(accel_Integral*max_set.rad_velo);	//s

	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	turn_time = 0;

	while((float)turn_time < set_turn_time * 1000);
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	max_set.rad_velo = 0.0f;
	target.rad_velo = 0.0f;
	target.rad_accel = 0.0f;

	run_mode = STRAIGHT_MODE;
	machine.length	= target.length	= 0.0f;
	max_set.velo	= target.velo	= param->velo;
	max_set.length = param->Lend;
	while(machine.length < max_set.length );

	machine.length = target.length  = 0.0f;


}

void search_turn90_table(const t_param* parameter)
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


	if(sen_fr.is_wall == true && sen_fl.is_wall == true)
	{
		while((90.0 + 12.0 - ((sen_fr.distance+sen_fl.distance)/2 )) < parameter->param->Lstart) ;
	}
	else{
		while(machine.length < parameter->param->Lstart ){
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
	while((float)turn_time < set_turn_time * 1000);
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	max_set.rad_velo = 0.0f;
	target.rad_velo = 0.0f;
	target.rad_accel = 0.0f;

	run_mode = STRAIGHT_MODE;
	machine.length	= target.length	= 0.0f;
	max_set.velo	= target.velo	= parameter->param->velo;
	max_set.length = parameter->param->Lend;
	while(machine.length < max_set.length );

	machine.length = target.length  = 0.0f;
	wall_controll.is_controll = true;

}

void long_turn(const t_param* parameter,const t_straight_param* st_param)
{

	Sp_Param_I_Initialize(&target);
	Sp_Param_I_Initialize(&machine);
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	init_W_parameters();
	Set_Omega_PID_Gain(st_param->om_gain->Kp, st_param->om_gain->Ki, st_param->om_gain->Kd);
	Set_Velo_PID_Gain(st_param->sp_gain->Kp, st_param->sp_gain->Ki, st_param->sp_gain->Kd);
	run_mode = STRAIGHT_MODE;

	max_set.velo = parameter->param->velo;
	target.velo  = parameter->param->velo;
	max_set.length = parameter->param->Lstart;
	t_bool hosei_flag = false;
	/*
	t_bool r_wall = sen_r.is_wall;
	t_bool l_wall = sen_r.is_wall;
	uint16_t r_wall_cnt = 0;
	uint16_t l_wall_cnt = 0;
	*/
	while(machine.length < parameter->param->Lstart ){
		/*
		if(r_wall == true)	r_wall_cnt++;
		else				r_wall_cnt = 0;

		if(l_wall == true)	l_wall_cnt++;
		else				l_wall_cnt = 0;

		if((r_wall == true && sen_r.is_wall == false) && machine.length > 0.0 && r_wall_cnt > 100)
		{
			if(hosei_flag == false)
			{
				machine.length = SEARCH_HOSEI-45.0;
				hosei_flag = true;
			}
		}
		if((l_wall == true && sen_l.is_wall == false)&& machine.length > 0.0 && l_wall_cnt > 100)
		{
			if(hosei_flag == false)
			{
				machine.length = SEARCH_HOSEI-45.0;
				hosei_flag = true;
			}
		}
		r_wall = sen_r.is_wall;
		l_wall = sen_l.is_wall;
		*/
		if(hosei_flag == false && st_r_hosei_check == true)
		{
			machine.length = SEARCH_HOSEI-45.0;;
			hosei_flag = true;
		}
		if(hosei_flag == false && st_l_hosei_check == true)
		{
			machine.length = SEARCH_HOSEI-45.0;;
			hosei_flag = true;
		}
	}

	Sp_Param_I_Initialize(&target);
	Sp_Param_I_Initialize(&machine);

	Set_PID_Gain(&velo_g, parameter->sp_gain->Kp, parameter->sp_gain->Ki, parameter->sp_gain->Kd);
	Set_PID_Gain(&omega_g, parameter->om_gain->Kp, parameter->om_gain->Ki, parameter->om_gain->Kd);
	run_mode = TURN_MODE_TABLE;
	max_set.rad_velo = parameter->param->velo/(parameter->param->r_min/1000);							//rad\s
	set_turn_time = DEG2RAD(parameter->param->degree)/(accel_Integral*max_set.rad_velo);	//s

	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	turn_time = 0;

	while((float)turn_time < set_turn_time * 1000);

	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	max_set.rad_velo = 0.0f;
	target.rad_velo = 0.0f;
	target.rad_accel = 0.0f;
	Set_Omega_PID_Gain(st_param->om_gain->Kp, st_param->om_gain->Ki, st_param->om_gain->Kd);
	Set_Velo_PID_Gain(st_param->sp_gain->Kp, st_param->sp_gain->Ki, st_param->sp_gain->Kd);
	run_mode = STRAIGHT_MODE;
	machine.length	= target.length	= 0.0f;
	max_set.velo	= target.velo	= parameter->param->velo;
	max_set.length = parameter->param->Lend;
	hosei_flag = false;
	while(machine.length < max_set.length ){
		if(hosei_flag == false && st_r_hosei_check == true)
		{
			machine.length = max_set.length;
			hosei_flag = true;
		}
		if(hosei_flag == false && st_l_hosei_check == true)
		{
			machine.length = max_set.length;
			hosei_flag = true;
		}
	}

	machine.length = target.length  = 0.0f;
}


void turn_v90(const t_param* parameter,const t_straight_param* di_param)
{

	Sp_Param_I_Initialize(&target);
	Sp_Param_I_Initialize(&machine);
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	init_W_parameters();
	Set_Omega_PID_Gain(di_param->om_gain->Kp, di_param->om_gain->Ki, di_param->om_gain->Kd);
	Set_Velo_PID_Gain(di_param->sp_gain->Kp, di_param->sp_gain->Ki, di_param->sp_gain->Kd);
	run_mode = DIAG_MODE;

	max_set.velo = parameter->param->velo;
	target.velo  = parameter->param->velo;
	max_set.length = parameter->param->Lstart;
	t_bool hosei_flag = false;
	while(machine.length < parameter->param->Lstart )
	{
		if(parameter->param->turn_dir == LEFT)
		{
			if(hosei_flag == false && diag_l_hosei_check == true)
			{
				machine.length = -2.0;
				hosei_flag = true;
			}
		}
		if(parameter->param->turn_dir == RIGHT)
		{
			if(hosei_flag == false && diag_r_hosei_check == true)
			{
				machine.length = -2.0;
				hosei_flag = true;
			}
		}
	}

	Set_PID_Gain(&velo_g, parameter->sp_gain->Kp, parameter->sp_gain->Ki, parameter->sp_gain->Kd);
	Set_PID_Gain(&omega_g, parameter->om_gain->Kp, parameter->om_gain->Ki, parameter->om_gain->Kd);
	run_mode = TURN_MODE_TABLE;
	max_set.rad_velo = parameter->param->velo/(parameter->param->r_min/1000);							//rad\s
	set_turn_time = DEG2RAD(parameter->param->degree)/(accel_Integral*max_set.rad_velo);	//s

	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	turn_time = 0;

	while((float)turn_time < set_turn_time * 1000);
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	init_W_parameters();
	max_set.rad_velo = 0.0f;
	target.rad_velo = 0.0f;
	target.rad_accel = 0.0f;
	Set_Omega_PID_Gain(di_param->om_gain->Kp, di_param->om_gain->Ki, di_param->om_gain->Kd);
	Set_Velo_PID_Gain(di_param->sp_gain->Kp, di_param->sp_gain->Ki, di_param->sp_gain->Kd);
	run_mode = DIAG_MODE;
	machine.length	= target.length	= 0.0f;
	max_set.velo	= target.velo	= parameter->param->velo;
	max_set.length = parameter->param->Lend;
	hosei_flag = false;
	while(machine.length < max_set.length ){
		if(parameter->param->turn_dir == LEFT)
		{
			if(hosei_flag == false && diag_r_hosei_check == true)
			{
				machine.length = max_set.length-1.0;
				hosei_flag = true;
			}
		}
		if(parameter->param->turn_dir == RIGHT)
		{
			if(hosei_flag == false && diag_l_hosei_check == true)
			{
				machine.length = max_set.length-1.0;
				hosei_flag = true;
			}
		}
	}

	machine.length = target.length  = 0.0f;
}

void turn_in(const t_param* parameter,const t_straight_param* st_param,const t_straight_param* di_param)
{

	Sp_Param_I_Initialize(&target);
	Sp_Param_I_Initialize(&machine);
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	init_W_parameters();
	Set_Omega_PID_Gain(st_param->om_gain->Kp, st_param->om_gain->Ki, st_param->om_gain->Kd);
	Set_Velo_PID_Gain(st_param->sp_gain->Kp, st_param->sp_gain->Ki, st_param->sp_gain->Kd);
	run_mode = STRAIGHT_MODE;

	max_set.velo = parameter->param->velo;
	target.velo  = parameter->param->velo;
	max_set.length = parameter->param->Lstart;

	t_bool hosei_flag = false;
	while(machine.length < parameter->param->Lstart ){
		if(hosei_flag == false && st_r_hosei_check == true)
		{
			machine.length = SEARCH_HOSEI-45.0;;
			hosei_flag = true;
		}
		if(hosei_flag == false && st_l_hosei_check == true)
		{
			machine.length = SEARCH_HOSEI-45.0;;
			hosei_flag = true;
		}
	}

	if(parameter->param->turn_dir == LEFT && sen_l.is_wall == true)
	{
		machine.length	= target.length	= 0.0f;
		hosei_flag = false;
		while(machine.length < parameter->param->Lstart){
			if(hosei_flag == false && st_l_hosei_check == true)
			{
				machine.length = SEARCH_HOSEI-45.0;;
				hosei_flag = true;
			}
		}
	}
	else if(parameter->param->turn_dir == RIGHT && sen_r.is_wall == true)
	{
		machine.length	= target.length	= 0.0f;
		hosei_flag = false;
		while(machine.length < parameter->param->Lstart){
			if(hosei_flag == false && st_r_hosei_check == true)
			{
				machine.length = SEARCH_HOSEI-45.0;;
				hosei_flag = true;
			}
		}
	}

	Set_PID_Gain(&velo_g, parameter->sp_gain->Kp, parameter->sp_gain->Ki, parameter->sp_gain->Kd);
	Set_PID_Gain(&omega_g, parameter->om_gain->Kp, parameter->om_gain->Ki, parameter->om_gain->Kd);
	run_mode = TURN_MODE_TABLE;
	max_set.rad_velo = parameter->param->velo/(parameter->param->r_min/1000);							//rad\s
	set_turn_time = DEG2RAD(parameter->param->degree)/(accel_Integral*max_set.rad_velo);	//s

	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	turn_time = 0;
	if(parameter->param->degree == 45.0f || parameter->param->degree == -45.0f) enable_lsm = false;
	while((float)turn_time < set_turn_time * 1000);
	max_set.rad_velo = 0.0f;
	target.rad_velo = 0.0f;
	target.rad_accel = 0.0f;
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	Set_Omega_PID_Gain(di_param->om_gain->Kp, di_param->om_gain->Ki, di_param->om_gain->Kd);
	Set_Velo_PID_Gain(di_param->sp_gain->Kp, di_param->sp_gain->Ki, di_param->sp_gain->Kd);
	run_mode = DIAG_MODE;
	machine.length	= target.length	= 0.0f;
	max_set.velo	= target.velo	= parameter->param->velo;
	max_set.length = parameter->param->Lend;
	hosei_flag = false;
	while(machine.length < max_set.length ){
		if(parameter->param->turn_dir == LEFT)
		{
			if(hosei_flag == false && diag_r_hosei_check == true)
			{
				machine.length = max_set.length-1.0;
				hosei_flag = true;
			}
		}
		if(parameter->param->turn_dir == RIGHT)
		{
			if(hosei_flag == false && diag_l_hosei_check == true)
			{
				machine.length = max_set.length-1.0;
				hosei_flag = true;
			}
		}
	}

	machine.length = target.length  = 0.0f;
	if(parameter->param->degree == 45.0f || parameter->param->degree == -45.0f) enable_lsm = true;
}

void turn_out(const t_param* parameter,const t_straight_param* st_param,const t_straight_param* di_param)
{

	Sp_Param_I_Initialize(&target);
	Sp_Param_I_Initialize(&machine);
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	init_W_parameters();
	Set_Omega_PID_Gain(di_param->om_gain->Kp, di_param->om_gain->Ki, di_param->om_gain->Kd);
	Set_Velo_PID_Gain(di_param->sp_gain->Kp, di_param->sp_gain->Ki, di_param->sp_gain->Kd);
	run_mode = DIAG_MODE;

	max_set.velo = parameter->param->velo;
	target.velo  = parameter->param->velo;
	max_set.length = parameter->param->Lstart;

	t_bool hosei_flag = false;
	while(machine.length < parameter->param->Lstart ){
		if(parameter->param->turn_dir == LEFT)
		{
			if(hosei_flag == false && diag_l_hosei_check == true)
			{
				machine.length = -2.0;
				hosei_flag = true;
			}
		}
		if(parameter->param->turn_dir == RIGHT)
		{
			if(hosei_flag == false && diag_r_hosei_check == true)
			{
				machine.length = -2.0;
				hosei_flag = true;
			}
		}
	}

	Set_PID_Gain(&velo_g, parameter->sp_gain->Kp, parameter->sp_gain->Ki, parameter->sp_gain->Kd);
	Set_PID_Gain(&omega_g, parameter->om_gain->Kp, parameter->om_gain->Ki, parameter->om_gain->Kd);
	run_mode = TURN_MODE_TABLE;
	max_set.rad_velo = parameter->param->velo/(parameter->param->r_min/1000);							//rad\s
	set_turn_time = DEG2RAD(parameter->param->degree)/(accel_Integral*max_set.rad_velo);	//s

	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	turn_time = 0;

	while((float)turn_time < set_turn_time * 1000);
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	max_set.rad_velo = 0.0f;
	target.rad_velo = 0.0f;
	target.rad_accel = 0.0f;
	Set_Omega_PID_Gain(di_param->om_gain->Kp, di_param->om_gain->Ki, di_param->om_gain->Kd);
	Set_Velo_PID_Gain(di_param->sp_gain->Kp, di_param->sp_gain->Ki, di_param->sp_gain->Kd);
	run_mode = STRAIGHT_MODE;
	machine.length	= target.length	= 0.0f;
	max_set.velo	= target.velo	= parameter->param->velo;
	max_set.length = parameter->param->Lend;
	hosei_flag = false;
	while(machine.length < max_set.length )
	{
		if(hosei_flag == false && st_r_hosei_check == true)
		{
			machine.length = max_set.length;
			hosei_flag = true;
		}
		if(hosei_flag == false && st_l_hosei_check == true)
		{
			machine.length = max_set.length;
			hosei_flag = true;
		}
	}

	machine.length = target.length  = 0.0f;
}

void set_stop_wall(int millis)
{
	Sp_Param_I_Initialize(&target);
	Sp_Param_I_Initialize(&machine);
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	Sp_Param_Initialize(&target);
	Sp_Param_Initialize(&machine);
	run_mode = TURN_MODE;

	for(int i = 0; i < millis;i++)
	{
		if(sen_fr.distance < 70.0 && sen_fl.distance < 70.0)
		{
			float sp_err = ((sen_fr.distance - 45.0) + (sen_fl.distance - 45.0))/2.0f;
			float om_err = ((sen_fr.distance - 45.0) - (sen_fl.distance - 45.0))/2.0f;

			target.accel = (1.0 * sp_err - 100.0*target.velo);
			//target.velo = 0.05 * sp_err;//veloだったら0.05
			max_set.velo = 0.3;
			if(target.velo >=  max_set.velo)
			{
				target.accel = 0.0;
				target.velo = max_set.velo;
			}
			else if(target.velo <= -max_set.velo){
				target.accel = 0.0;
				target.velo = -max_set.velo;
			}

			target.rad_accel = (5.0*om_err - 20.0*target.rad_velo);
			//target.rad_velo = 0.1*om_err;////veloだったら0.5
			max_set.rad_velo = 10.0;
			if(target.rad_velo >= max_set.rad_velo)
			{
				target.rad_accel = 0.0;
				target.rad_velo = max_set.velo;
			}
			else if(target.rad_velo <= -max_set.rad_velo)
			{
				target.rad_accel = 0.0;
				target.rad_velo = -max_set.velo;
			}

		}
		else
		{
			target.accel = 0.0f;
			target.velo = 0.0f;
			max_set.velo = 0.0f;

			target.rad_velo = 0.0f;
			target.rad_accel = 0.0f;
			max_set.rad_velo = 0.0f;
		}
		HAL_Delay(1);
	}
	Sp_Param_rad_Initialize(&target);
	Sp_Param_rad_Initialize(&machine);
	Sp_Param_Initialize(&target);
	Sp_Param_Initialize(&machine);
	Sp_Param_Initialize(&max_set);

	HAL_Delay(100);
	run_mode = NON_CON_MODE;
}

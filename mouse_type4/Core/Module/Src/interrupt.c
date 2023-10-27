/*
 * interrupt.c
 *
 *  Created on: Aug 13, 2022
 *      Author: sato1
 */

#include "index.h"
#include "glob_var.h"
#include "rad_accel_table.h"
#include "sensor_table.h"
#include "kalman_filter.h"
float enc_sp;
float motor_r_ampere,motor_l_ampere;
float FF_Vr_LPF,FF_Vl_LPF;
float disturb_Vr_LPF,disturb_Vl_LPF;
float diff_low_pass;
const float k = 0.95;
const float km = 0.95;
static int prev_buff;
static int i;


float W_yaw;
const float gain_W_yaw = 0.0;
const float input_constant = 0.1;


float W_Vr[4],W_Vl[4];
float u_Vr[4],u_Vl[4];
float predict_Vr,predict_Vl;
float error_Vr,error_Vl;
const float lambda_r = 0.1;
const float lambda_l = 0.1;

const float varep = 1/1000000;

float fb_rad_velo = 0.0;

float W_yaw_kr;
const float gain_W_yaw_kr = 0.0001;
/*
float W_yaw_kr_acc;
const float gain_W_yaw_kr_acc = 0.5/1000000.0f;

float W_yaw_dis;
const float gain_W_yaw_dis = 0.005;

float W_yaw_dis_acc;
const float gain_W_yaw_dis_acc = 0.5/1000000.0f;
*/
float W_rad_sp;
const float gain_W_rad_sp = 0.05;
float W_rad_acc = 0.0;//0.004;
const float gain_W_rad_acc = 0.01/1000.0f;//0.01/1000.0f;
float W_sp;

float W_rad_sp_rls;
float P_rad_sp_rls;
const float lambda_rls = 1.0;

const float gain_W_sp = 1.0;
float W_spr;
const float gain_W_spr = 0.0;
float W_acc = 0.0;//MOTOR_R/(MOTOR_K_TR*GEAR_N)*(WEIGHT/1000*TIRE_RADIUS/2);
const float gain_W_acc = 1.0;

const float interrupt_time = 0.001;
const float inv_interrupt_time = 1000.0f;
const int   m_dt = 1;
float model_predict_motor_r_ampere,model_predict_motor_l_ampere;


void init_W_parameters()
{
	W_rad_sp = 0.0;
	W_rad_acc = 0.004;
	W_sp = 0.0;
	W_acc = 0.0;
	W_yaw = 0.0;
	prev_V_l = 0.0;
	prev_V_r = 0.0;
	W_yaw_kr = 0.0;

	W_rad_sp_rls = 0.0f;
	P_rad_sp_rls = 1.0f;
	//W_yaw_dis = W_yaw_dis_acc =0.0;
	//W_yaw_kr_acc = W_yaw_dis_acc = 0.0;
	//MOTOR_R/(MOTOR_K_TR*GEAR_N)*(WEIGHT/1000*TIRE_RADIUS/2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim == &htim5){
        Interrupt_PreProcess();
        Interrupt_Main();
        Interrupt_PostProcess();

    }
}

void Interrupt_Initialize(){
	HAL_TIM_Base_Start_IT(&htim5);
}


void Interrupt_PreProcess(){
	//get & calc encoder pulse
	Interrupt_Get_Irsensor();
	Interrupt_Get_Speed();
	Interrupt_Set_Target_Speed();
}
/*
void Interrupt_Main(){
	//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	V_r = V_l = 0.0f;
	motor_out_r = motor_out_l = 0;

	if(run_mode == STRAIGHT_MODE || run_mode == TURN_MODE || run_mode == DIAG_MODE || run_mode == TURN_MODE_TABLE )
	{


	  	//determine FF
		float motor_r_rpm = RAD_2_RPM*GEAR_N*(target.velo*1000/TIRE_RADIUS + 1.0f*TREAD_WIDTH*target.rad_velo/(2*TIRE_RADIUS));
	  	float motor_l_rpm = RAD_2_RPM*GEAR_N*(target.velo*1000/TIRE_RADIUS - 1.0f*TREAD_WIDTH*target.rad_velo/(2*TIRE_RADIUS));
	  	float prev_motor_r_amp = motor_r_ampere;
	  	float prev_motor_l_amp = motor_l_ampere;
	  	motor_r_ampere = 1/(MOTOR_K_TR*GEAR_N)*(WEIGHT*target.accel/1000*TIRE_RADIUS/2 + 1.0f*MOUSE_INERTIA*target.rad_accel*TIRE_RADIUS/TREAD_WIDTH) + MOTOR_BR*motor_r_rpm/MOTOR_K_TR;
	  	motor_l_ampere = 1/(MOTOR_K_TR*GEAR_N)*(WEIGHT*target.accel/1000*TIRE_RADIUS/2 - 1.0f*MOUSE_INERTIA*target.rad_accel*TIRE_RADIUS/TREAD_WIDTH) + MOTOR_BR*motor_l_rpm/MOTOR_K_TR;

	  	float FF_Vr = (L_BAR_DT*(motor_r_ampere-prev_motor_r_amp) + MOTOR_R*motor_r_ampere + MOTOR_K_ER*motor_r_rpm/1000);
	  	float FF_Vl = (L_BAR_DT*(motor_l_ampere-prev_motor_l_amp) + MOTOR_R*motor_l_ampere + MOTOR_K_ER*motor_l_rpm/1000);
	  	FF_Vr_LPF = (1-k)*FF_Vr_LPF + k*FF_Vr;
	  	FF_Vl_LPF = (1-k)*FF_Vl_LPF + k*FF_Vl;

	  	//calculate observe voltage
	  	float real_motor_r_rpm = RAD_2_RPM*GEAR_N*(machine.velo*1000/TIRE_RADIUS + 1.0f*TREAD_WIDTH*machine.rad_velo/(2*TIRE_RADIUS));
	  	float real_motor_l_rpm = RAD_2_RPM*GEAR_N*(machine.velo*1000/TIRE_RADIUS - 1.0f*TREAD_WIDTH*machine.rad_velo/(2*TIRE_RADIUS));
	  	float prev_model_predict_motor_r_ampere = model_predict_motor_r_ampere;
	  	float prev_model_predict_motor_l_ampere = model_predict_motor_l_ampere;

	  	model_predict_motor_r_ampere = 1/(MOTOR_K_TR*GEAR_N)*(WEIGHT*machine.accel/1000*TIRE_RADIUS/2 + 1.0f*MOUSE_INERTIA*machine.rad_accel*TIRE_RADIUS/TREAD_WIDTH) + MOTOR_BR*real_motor_r_rpm/MOTOR_K_TR;
	  	model_predict_motor_l_ampere = 1/(MOTOR_K_TR*GEAR_N)*(WEIGHT*machine.accel/1000*TIRE_RADIUS/2 - 1.0f*MOUSE_INERTIA*machine.rad_accel*TIRE_RADIUS/TREAD_WIDTH) + MOTOR_BR*real_motor_l_rpm/MOTOR_K_TR;

	  	float model_predict_motor_V_r = (L_BAR_DT*(model_predict_motor_r_ampere-prev_model_predict_motor_r_ampere) + MOTOR_R*model_predict_motor_r_ampere + MOTOR_K_ER*real_motor_r_rpm/1000);
		float model_predict_motor_V_l = (L_BAR_DT*(model_predict_motor_l_ampere-prev_model_predict_motor_l_ampere) + MOTOR_R*model_predict_motor_l_ampere + MOTOR_K_ER*real_motor_l_rpm/1000);

		//disturb_Vr_LPF = (1 - km)*disturb_Vr_LPF + km*model_predict_motor_V_r;
		//disturb_Vl_LPF = (1 - km)*disturb_Vl_LPF + km*model_predict_motor_V_l;

		//V_r += FF_Vr_LPF + (-1.0)*disturb_Vr_LPF;
		//V_l -= FF_Vl_LPF + (-1.0)*disturb_Vl_LPF;
/*
		//LMS.P[行たて][列よこ]
		u_Vr[0] = u_Vl[0] = machine.velo;
		u_Vr[1] = u_Vl[1] = machine.accel;
		u_Vr[2] = u_Vl[2] = machine.rad_velo;
		u_Vr[3] = machine.rad_accel/1000.0f;
		u_Vl[3] = machine.rad_accel/1000.0f;


		predict_Vr = W_Vr[0]*u_Vr[0]+W_Vr[1]*u_Vr[1]+W_Vr[2]*u_Vr[2]+W_Vr[3]*u_Vr[3];
		predict_Vl = W_Vl[0]*u_Vl[0]+W_Vl[1]*u_Vl[1]+W_Vl[2]*u_Vl[2]+W_Vl[3]*u_Vl[3];

		error_Vr = prev_V_r-predict_Vr;
		error_Vl = prev_V_l-predict_Vl;

		for(int i = 0; i < 4 ;i++){
			W_Vr[i] = W_Vr[i] + lambda_r*error_Vr*u_Vr[i]/1000.0f;
			W_Vl[i] = W_Vl[i] + lambda_r*error_Vl*u_Vl[i]/1000.0f;
		}
*/

        //V_r += (W_Vr[0]*target.velo+W_Vr[1]*target.accel+W_Vr[2]*target.rad_velo+W_Vr[3]*target.rad_accel/10000.0f);
        //V_l += (W_Vl[0]*target.velo+W_Vl[1]*target.accel+W_Vl[2]*target.rad_velo+W_Vr[3]*target.rad_accel/10000.0f);
/*
	  	V_r += FF_Vr_LPF;
	  	V_l -= FF_Vl_LPF;

		//speed FB
		V_r += (target.velo - machine.velo)*velo_g.Kp;
		V_l -= (target.velo - machine.velo)*velo_g.Kp;

		V_r += (target.I_velo - machine.I_velo)*velo_g.Ki;
		V_l -= (target.I_velo - machine.I_velo)*velo_g.Ki;

		diff_low_pass = 0.9*diff_low_pass + 0.1*(((target.velo - target.prev_velo) - (machine.velo - machine.prev_velo))*velo_g.Kd);

		//V_r += ((target.velo - target.prev_velo) - (machine.velo - machine.prev_velo))*velo_g.Kd;
		//V_l -= ((target.velo - target.prev_velo) - (machine.velo - machine.prev_velo))*velo_g.Kd;
		V_r += diff_low_pass;
		V_l -= diff_low_pass;


		//
		W_sp 	= W_sp 	+ gain_W_sp		* (target.prev_velo - machine.velo) * target.prev_velo	*interrupt_time;
		W_acc 	= W_acc + gain_W_acc 	* (target.prev_velo - machine.velo) * target.prev_accel	*interrupt_time;
		W_spr 	= W_spr + gain_W_spr	* (target.prev_velo - machine.velo) * machine.prev_velo	*interrupt_time;
		V_r += (W_sp*target.velo + W_acc*target.accel + W_spr*machine.velo);
		V_l -= (W_sp*target.velo + W_acc*target.accel + W_spr*machine.velo);

		//rad_sp FB
		V_r += (target.rad_velo - machine.rad_velo)*omega_g.Kp;
		V_l += (target.rad_velo - machine.rad_velo)*omega_g.Kp;

		V_r += (target.I_rad_velo - machine.I_rad_velo)*omega_g.Ki;
		V_l += (target.I_rad_velo - machine.I_rad_velo)*omega_g.Ki;

		V_r += ((target.rad_velo-target.prev_rad_velo) - (machine.rad_velo-machine.prev_rad_velo))*omega_g.Kd;
		V_l += ((target.rad_velo-target.prev_rad_velo) - (machine.rad_velo-machine.prev_rad_velo))*omega_g.Kd;

		//
		/*
		W_rad_sp 	= W_rad_sp 	  + gain_W_rad_sp		* (target.prev_rad_velo - machine.rad_velo)  * target.prev_rad_velo	*interrupt_time;
		W_rad_acc 	= W_rad_acc   + gain_W_rad_acc 		* (target.prev_rad_velo - machine.rad_velo)  * target.prev_rad_accel*interrupt_time;
		W_yaw		= W_yaw		  + gain_W_yaw			* (target.prev_rad_velo - machine.rad_velo)  * input_constant*interrupt_time;
		W_yaw_kr	= W_yaw_kr    + gain_W_yaw_kr  		* (target.prev_rad_velo - machine.rad_velo)  * (-1.0)*machine.prev_rad_velo*interrupt_time;

		float err_rls = target.prev_rad_velo - machine.rad_velo;
		float g_rad_sp_rls = (lambda_rls*P_rad_sp_rls*(target.prev_rad_accel))/(1.0f+lambda_rls*P_rad_sp_rls*(target.prev_rad_accel)*(target.prev_rad_accel));
		P_rad_sp_rls = lambda_rls*(P_rad_sp_rls - g_rad_sp_rls*(target.prev_rad_accel)*P_rad_sp_rls);
		W_rad_sp_rls = W_rad_sp_rls + err_rls*g_rad_sp_rls*interrupt_time;

*/
		//V_r += target.rad_accel *(W_rad_sp_rls);
		//V_l += target.rad_accel *(W_rad_sp_rls);
/*
		W_yaw_dis	= W_yaw_dis + gain_W_yaw_dis * (target.rad_velo - machine.rad_velo) * (target.rad_velo - machine.rad_velo)/1000.0f;

		W_yaw_kr_acc	= W_yaw_kr_acc + gain_W_yaw_kr_acc * (target.rad_velo - machine.rad_velo) * (machine.rad_accel)/1000.0f;
		W_yaw_dis_acc	= W_yaw_dis_acc + gain_W_yaw_dis_acc * (target.rad_velo - machine.rad_velo) * (target.rad_accel - machine.rad_accel)/1000.0f;
*/
/*
		if( enable_lsm == true)
		{
			V_r += (W_rad_sp*target.rad_velo + W_rad_acc* target.rad_accel + W_yaw * input_constant);
			V_l += (W_rad_sp*target.rad_velo + W_rad_acc* target.rad_accel + W_yaw * input_constant);
		}
		*/
		//prev_V_l = V_l;
		//prev_V_r = V_r;

		//V_r- += W_yaw_kr*(-1.0)*machine.rad_velo;
		//V_l- += W_yaw_kr*(-1.0)*machine.rad_velo;

		/*
		V_r += (W_yaw_kr*machine.rad_velo-W_yaw_dis*(target.rad_velo - machine.rad_velo) );
		V_l += (W_yaw_kr*machine.rad_velo-W_yaw_dis*(target.rad_velo - machine.rad_velo) );
		V_r += (W_yaw_kr_acc*machine.rad_accel-W_yaw_dis_acc*(target.rad_accel - machine.rad_accel) );
		V_l += (W_yaw_kr_acc*machine.rad_accel-W_yaw_dis_acc*(target.rad_accel - machine.rad_accel) );
		 *//*
	}
	else
	{
		init_W_parameters();
	}

	if(run_mode == STRAIGHT_MODE || run_mode == TURN_MODE || run_mode == DIAG_MODE || run_mode == TURN_MODE_TABLE )
	{
		float duty_r = V_r/Battery_GetVoltage();
		float duty_l = V_l/Battery_GetVoltage();
		if(ABS(duty_r) > 1.0){
			motor_out_r = (int)(SIGN(duty_r) * 4.0f * 250.0f);
		}else{
			motor_out_r = (int)(duty_r * 1000.0f);
		}
		Motor_SetDuty_Right(motor_out_r);

		if(ABS(duty_l) > 1.0){
			motor_out_l = (int)(SIGN(duty_l) * 4.0f * 250.0f);
		}else{
			motor_out_l = (int)(duty_l * 1000.0f);
		}
		Motor_SetDuty_Left(motor_out_l);
	}
	else if(run_mode == NON_CON_MODE)
	{
		Motor_SetDuty_Left(0);
		Motor_SetDuty_Right(0);
	}
}
*/

void Interrupt_Main(){
	//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	V_r = V_l = 0.0f;
	motor_out_r = motor_out_l = 0;

	if(run_mode == STRAIGHT_MODE || run_mode == TURN_MODE || run_mode == DIAG_MODE || run_mode == TURN_MODE_TABLE )
	{


	  	//determine FF
		float motor_r_rpm = RAD_2_RPM*GEAR_N*(target.velo*1000/TIRE_RADIUS + 1.0f*TREAD_WIDTH*target.rad_velo/(2*TIRE_RADIUS));
	  	float motor_l_rpm = RAD_2_RPM*GEAR_N*(target.velo*1000/TIRE_RADIUS - 1.0f*TREAD_WIDTH*target.rad_velo/(2*TIRE_RADIUS));
	  	float prev_motor_r_amp = motor_r_ampere;
	  	float prev_motor_l_amp = motor_l_ampere;
	  	motor_r_ampere = 1/(MOTOR_K_TR*GEAR_N)*(WEIGHT*target.accel/1000*TIRE_RADIUS/2 + 1.0f*MOUSE_INERTIA*target.rad_accel*TIRE_RADIUS/TREAD_WIDTH) + MOTOR_BR*motor_r_rpm/MOTOR_K_TR;
	  	motor_l_ampere = 1/(MOTOR_K_TR*GEAR_N)*(WEIGHT*target.accel/1000*TIRE_RADIUS/2 - 1.0f*MOUSE_INERTIA*target.rad_accel*TIRE_RADIUS/TREAD_WIDTH) + MOTOR_BR*motor_l_rpm/MOTOR_K_TR;

	  	float FF_Vr = (L_BAR_DT*(motor_r_ampere-prev_motor_r_amp) + MOTOR_R*motor_r_ampere + MOTOR_K_ER*motor_r_rpm/1000);
	  	float FF_Vl = (L_BAR_DT*(motor_l_ampere-prev_motor_l_amp) + MOTOR_R*motor_l_ampere + MOTOR_K_ER*motor_l_rpm/1000);
	  	FF_Vr_LPF = (1-k)*FF_Vr_LPF + k*FF_Vr;
	  	FF_Vl_LPF = (1-k)*FF_Vl_LPF + k*FF_Vl;

	  	//


	  	V_r += FF_Vr_LPF;
	  	V_l -= FF_Vl_LPF;

		float ff_turn = (FF_Vr - FF_Vl_LPF)/2.0;
		float ff_st   = (FF_Vr + FF_Vl_LPF)/2.0;

	  	float speed_fb = 0.0;
	  	float omega_fb = 0.0;


	  	speed_fb += (target.velo - machine.velo)*velo_g.Kp;
	  	speed_fb += (target.I_velo - machine.I_velo)*velo_g.Ki;
	  	diff_low_pass = 0.9*diff_low_pass + 0.1*(((target.velo - target.prev_velo) - (machine.velo - machine.prev_velo))*velo_g.Kd);
	  	speed_fb += diff_low_pass;


	  	omega_fb += (target.rad_velo - machine.rad_velo)*omega_g.Kp;;
	  	omega_fb += (target.I_rad_velo - machine.I_rad_velo)*omega_g.Ki;
	  	omega_fb += ((target.rad_velo-target.prev_rad_velo) - (machine.rad_velo-machine.prev_rad_velo))*omega_g.Kd;




	  	float ctrl_limit = ABS(ff_turn) + ABS(ff_st);
	  	float sp_limit = ABS(Battery_GetVoltage() - ctrl_limit);
		float sp_operation = speed_fb;
	  	if(ABS(sp_operation) > sp_limit)
		{
			float diff =  sp_operation - SIGN(sp_operation) * sp_limit;
			if(velo_g.Kp != 0.0f)
			{
				target.I_velo  = target.I_velo  - 1/velo_g.Kp*diff/2.0;
				machine.I_velo = machine.I_velo + 1/velo_g.Kp*diff/2.0;
			}
			sp_operation = SIGN(sp_operation) * sp_limit;
		}

	  	if(run_mode == STRAIGHT_MODE || run_mode == DIAG_MODE )
	  	{
			//om_fb_controll = ct.omega_ctrl.Anti_windup_2(om_fb_controll + (sp_FF_controll_r-sp_FF_controll_l)/2.0, 1.0);
		  	if(ABS(ff_turn + omega_fb) > 1.0)
			{
				float diff =  (ff_turn + omega_fb) - SIGN(ff_turn + omega_fb) * 1.0;
				if(omega_g.Kp != 0.0f)
				{
					target.I_rad_velo  = target.I_rad_velo  - 1/omega_g.Kp*diff/2.0;
					machine.I_rad_velo = machine.I_rad_velo + 1/omega_g.Kp*diff/2.0;
				}
				omega_fb = omega_fb - ff_turn;
			}
	  	}

	  	float omega_limit = ABS(Battery_GetVoltage() - ctrl_limit);
		float omega_operation = omega_fb;
	  	if(ABS(omega_operation) > omega_limit)
		{
			float diff =  omega_operation - SIGN(omega_operation) * omega_limit;
			if(omega_g.Kp != 0.0f)
			{
				target.I_rad_velo  = target.I_rad_velo  - 1/omega_g.Kp*diff/2.0;
				machine.I_rad_velo = machine.I_rad_velo + 1/omega_g.Kp*diff/2.0;
			}
			omega_operation = SIGN(sp_operation) * omega_limit;
		}

		V_r += sp_operation;
		V_l -= sp_operation;

		V_r += omega_fb;
		V_l += omega_fb;

	}
	else
	{
		init_W_parameters();
	}

	if(run_mode == STRAIGHT_MODE || run_mode == TURN_MODE || run_mode == DIAG_MODE || run_mode == TURN_MODE_TABLE )
	{
		float duty_r = V_r/Battery_GetVoltage();
		float duty_l = V_l/Battery_GetVoltage();
		if(ABS(duty_r) > 1.0){
			motor_out_r = (int)(SIGN(duty_r) * 4.0f * 250.0f);
		}else{
			motor_out_r = (int)(duty_r * 1000.0f);
		}
		Motor_SetDuty_Right(motor_out_r);

		if(ABS(duty_l) > 1.0){
			motor_out_l = (int)(SIGN(duty_l) * 4.0f * 250.0f);
		}else{
			motor_out_l = (int)(duty_l * 1000.0f);
		}
		Motor_SetDuty_Left(motor_out_l);
	}
	else if(run_mode == NON_CON_MODE)
	{
		Motor_SetDuty_Left(0);
		Motor_SetDuty_Right(0);
	}
}
void Interrupt_PostProcess(){
	if(log_flag)
	{
		log_data[0][time/SCALE] = target.velo;//machine.length;//target.velo;
		log_data[1][time/SCALE] = machine.velo;
		log_data[2][time/SCALE] = target.rad_velo;//machine.length;//target.velo;
		log_data[3][time/SCALE] = machine.rad_velo;
		log_data[4][time/SCALE] = machine.length;//sen_fr.value;//target.rad_velo;
		log_data[5][time/SCALE] = (read_accel_y_axis());//sen_fl.value;//machine.rad_velo;
		log_data[6][time/SCALE] = target.rad_accel;//machine.length;//sen_r.distance;
		log_data[7][time/SCALE] = machine.accel;
		time++;
		if(time/SCALE >= LOG_COUNT) log_flag = false;
	}
	IMU_read_DMA_Start();



	//error process
	//static int error_cnt;

	if((ABS(machine.I_velo) > 5000.0 || ABS(machine.I_rad_velo) > 5000.0) && run_mode != NON_CON_MODE){
		//Motor_Stop();
		//save_data();
		//write_save_data();
		//NVIC_SystemReset();
	}

	else if((ABS(target.velo - machine.velo) > 2.0 || ABS(target.rad_velo - machine.rad_velo) > DEG2RAD(1300.0)) && run_mode != NON_CON_MODE){
		Motor_Stop();
		//save_data();
		write_save_data();
		NVIC_SystemReset();
	}
	else if(ABS(read_accel_z_axis()) >= 35.0 && run_mode != NON_CON_MODE){
		//Motor_Stop();
		//save_data();
		//write_save_data();
		//NVIC_SystemReset();
	}

}

void Interrupt_Get_Irsensor(){
	//----------sensor_setup----------//
	static float r_threshold = SIDE_THRESHOLD;
	static float l_threshold = SIDE_THRESHOLD;
	prev_buff = 20;
	static uint8_t r_cnt= 0,l_cnt = 0;
	sen_r.value = Sensor_GetValue(sensor_sr);
	sen_r.buff[i%50] = sen_r.value;
	sen_r.distance = Sensor_CalcDistance(sensor_sr,sen_r.value);
	if(sen_r.distance <= SIDE_THRESHOLD)
	{
		//sen_r.is_controll 	= true;
		sen_r.is_wall 		= true;
	}
	else
	{
		sen_r.is_controll 	= false;
		sen_r.is_wall 		= false;
	}

	if(sen_r.distance <= r_threshold && sen_r.is_wall == true)
	{
		sen_r.is_controll 	= true;
	}
	else
	{
		sen_r.is_controll 	= false;
	}


	sen_l.value = Sensor_GetValue(sensor_sl);
	sen_l.distance = Sensor_CalcDistance(sensor_sl,sen_l.value);
	sen_l.buff[i%50] = sen_l.value;
	if(sen_l.distance  <= SIDE_THRESHOLD)
	{
		//sen_l.is_controll 	= true;
		sen_l.is_wall 		= true;
	}
	else
	{
		sen_l.is_controll 	= false;
		sen_l.is_wall 		= false;
	}


	if(sen_l.distance <= l_threshold && sen_l.is_wall == true)
	{
		sen_l.is_controll 	= true;
	}
	else
	{
		sen_l.is_controll 	= false;
	}


	if(ABS(Sensor_CalcDistance(sensor_sr,sen_r.buff[(i-prev_buff + 50)%50]) - sen_r.distance) >= 3.0)
	{
		//sen_l.error = 0.0;
		r_threshold = 45.0;
		//sen_r.is_controll = false;
		r_cnt = 0;
	}
	else
	{
		r_threshold = SIDE_THRESHOLD;
	}

	if(ABS(Sensor_CalcDistance(sensor_sl,sen_l.buff[(i-prev_buff + 50)%50]) - sen_l.distance) >= 3.0)
	{
		//sen_l.error = 0.0;
		l_threshold = 45.0;
		//sen_l.is_controll = false;
		l_cnt = 0;
	}
	else
	{
		l_threshold = SIDE_THRESHOLD;
	}


	sen_fr.value = Sensor_GetValue(sensor_fr);
	sen_fr.distance = Sensor_CalcDistance(sensor_fr,sen_fr.value);
	sen_fr.buff[i%50] = sen_fr.value;
	if(sen_fr.distance <= FRONT_THRESHOLD)
	{
		sen_fr.is_controll 	= true;
		sen_fr.is_wall 		= true;
	}
	else
	{
		sen_fr.is_controll 	= false;
		sen_fr.is_wall 		= false;
	}
	if(sen_fr.distance < 80.0)	sen_r.is_controll = false;


	sen_fl.value = Sensor_GetValue(sensor_fl);
	sen_fl.distance = Sensor_CalcDistance(sensor_fl,sen_fl.value);
	sen_fl.buff[i%50] = sen_fl.value;
	if(sen_fl.distance <= FRONT_THRESHOLD)
	{
		sen_fl.is_controll 	= true;
		sen_fl.is_wall 		= true;
	}
	else
	{
		sen_fl.is_controll 	= false;
		sen_fl.is_wall 		= false;
	}
	if(sen_fl.distance < 80.0)	sen_l.is_controll = false;

	//----------set_indicator----------//


	r_cnt++;l_cnt++;
	if(run_mode != NON_CON_MODE)
	{
		switch(sen_r.is_controll)
		{
			case true:	Right_Side_On_LED(); 	break;
			case false:	Right_Side_Off_LED(); 	break;
		}
		if(r_cnt > 100) r_cnt = 100;
		switch(sen_l.is_controll)
		{
			case true:	Left_Side_On_LED(); 	break;
			case false:	Left_Side_Off_LED(); 	break;
		}
		if(l_cnt > 100) l_cnt = 100;
		switch(sen_fr.is_wall|sen_fl.is_wall)
		{
			case true:	Front_On_LED(); 	break;
			case false:	Front_Off_LED(); 	break;
		}
	}
	//----------set_controll----------//


	if(target.velo >= 0.20 && target.velo < 0.7){
		prev_buff = (int)(6.0/target.velo);
	}
	else if(target.velo >= 0.7)
	{
		prev_buff = 3;
	}
	else
	{
		prev_buff = 20;
	}


	if(run_mode == STRAIGHT_MODE && wall_controll.is_controll == true)
	{
		if(sen_r.is_controll == true){
			sen_r.error = sen_r.distance - 45.0;
			if(ABS(Sensor_CalcDistance(sensor_sr,sen_r.buff[(i-prev_buff + 50)%50]) - sen_r.distance) >= 3.0 && sen_r.error > 0.0)
			{
				sen_r.error = 0.0;
				r_cnt = 0;
			}
		}
		else{
			sen_r.error = 0.0;
		}

		if(sen_l.is_controll == true){
			sen_l.error = sen_l.distance - 45.0;
			if(ABS(Sensor_CalcDistance(sensor_sl,sen_l.buff[(i-prev_buff + 50)%50]) - sen_l.distance) >= 3.0 && sen_l.error  > 0.0)
			{
				sen_l.error = 0.0;
				l_cnt = 0;
			}

		}
		else
		{
			sen_l.error = 0.0;
		}


		wall_controll.prev_om_error = wall_controll.om_error;
		if(sen_r.is_controll == true && sen_l.is_controll == true)
		{
			wall_controll.om_error = (sen_l.error - sen_r.error)/2.0;
		}
		else
		{
			wall_controll.om_error = (sen_l.error - sen_r.error);
		}

/*
		target.rad_velo = 0.0;

		target.rad_velo += wall_controll.om_error*wall_controll.side_om_wall_gain.Kp;
		target.rad_velo	+=  (wall_controll.om_error-wall_controll.prev_om_error)*wall_controll.side_om_wall_gain.Kd;
		max_set.rad_velo = target.rad_velo;
*/

		//target.rad_velo = 0.0;


		target.rad_accel = (3.0)*wall_controll.om_error*1.0;
	    target.rad_accel = target.rad_accel-(machine.velo*target.rad_velo*10.00+target.rad_velo*30.0);
		//target.rad_accel = target.rad_accel-(target.velo*target.radian*10.00+target.rad_velo*10.0);
		//target.rad_accel = target.rad_accel-(machine.radian * machine.accel * 0.00);
	    target.rad_accel = target.rad_accel-(target.velo*target.radian*100.00);
		//max_set.rad_velo = target.rad_velo + target.rad_accel/1000.0f;

		if((Sensor_CalcDistance(sensor_sr,sen_r.buff[(i-prev_buff + 50)%50]) - sen_r.distance) <= -12.0)
		{
			st_r_hosei_check = true;
		}
		else
		{
			st_r_hosei_check = false;
		}

		if((Sensor_CalcDistance(sensor_sl,sen_l.buff[(i-prev_buff + 50)%50]) - sen_l.distance) <= -12.0)
		{
			st_l_hosei_check = true;
		}
		else
		{
			st_l_hosei_check = false;
		}

	}
	else if(run_mode == DIAG_MODE && wall_controll.is_controll == true)
	{

		if(sen_r.is_controll == true){
			sen_r.error = sen_r.distance - DIAG_HALF_SECTION;
			if(sen_r.error > 0.0) {
				sen_r.error = 0.0;
			}
			else{
				diag_predict_xr = sen_r.distance;
				machine.radian = 0.0f;
			}
		}

		else if(target.length  > DIAG_SECTION && machine.length > DIAG_HALF_SECTION ){

			diag_predict_xr = diag_predict_xr + machine.velo*machine.radian;
			sen_r.error = diag_predict_xr - DIAG_HALF_SECTION;
			if(sen_r.error > 0.0) sen_r.error = 0.0;
			sen_r.is_controll = true;
		}

		if(sen_l.is_controll == true){
			sen_l.error = sen_l.distance - DIAG_HALF_SECTION;
			if(sen_l.error > 0.0){
				sen_l.error = 0.0;
			}
			else{
				diag_predict_xl = sen_l.distance;
				machine.radian = 0.0f;
			}
		}

		else if(target.length > DIAG_SECTION && machine.length > DIAG_HALF_SECTION ){
			diag_predict_xl = diag_predict_xl - machine.velo*machine.radian;
			sen_l.error = diag_predict_xl - DIAG_HALF_SECTION;
			if(sen_l.error > 0.0){
				sen_l.error = 0.0;
			}
			sen_l.is_controll = true;
		}

		if((Sensor_CalcDistance(sensor_sr,sen_r.buff[(i-prev_buff + 50)%50]) - sen_r.distance) <= -10.0)
		{
			diag_r_hosei_check = true;
		}
		else
		{
			diag_r_hosei_check = false;
		}

		if((Sensor_CalcDistance(sensor_sl,sen_l.buff[(i-prev_buff + 50)%50]) - sen_l.distance) <= -10.0)
		{
			diag_l_hosei_check = true;
		}
		else
		{
			diag_l_hosei_check = false;
		}

		target.rad_velo = 0.0;
		wall_controll.prev_om_error = wall_controll.om_error;
		if(sen_r.is_controll == true && sen_l.is_controll == true)
		{
			wall_controll.om_error = (sen_l.error - sen_r.error)/2.0;
		}
		else
		{
			wall_controll.om_error = (sen_l.error - sen_r.error);
		}


		if(max_set.length - machine.length > 110.0)
		{
			if(sen_fr.value >= 300)//340
			{
				sen_fr.is_controll = true;
				sen_fr.error = sen_fr.value - 300;
			}
			else
			{
				sen_fr.is_controll = false;
				sen_fr.error = 0.0;
			}

			if(sen_fl.value >= 260)//200
			{
				sen_fl.is_controll = true;
				sen_fl.error = sen_fl.value - 260;
			}
			else
			{
				sen_fl.is_controll = false;
				sen_fl.error = 0.0;
			}

			if(sen_fr.is_controll == true && sen_fl.is_controll == true)
			{
				wall_controll.om_error += (sen_fr.error-sen_fl.error)/2.0*0.02;
			}
			else
			{
				wall_controll.om_error += (sen_fr.error-sen_fl.error)*0.02;
			}
		}

		target.rad_velo += wall_controll.om_error*wall_controll.side_om_wall_gain.Kp;
		target.rad_velo	+=  (wall_controll.om_error-wall_controll.prev_om_error)*wall_controll.side_om_wall_gain.Kd;
		max_set.rad_velo = target.rad_velo;
	}
	else if(run_mode == STRAIGHT_MODE || run_mode == DIAG_MODE)
	{
		max_set.rad_velo = 0.0;
		target.rad_velo  = 0.0;
	}

	i++;
}

void Interrupt_Get_Speed(){
	enc_R.prev_sp_pulse = enc_R.sp_pulse;
	enc_L.prev_sp_pulse = enc_L.sp_pulse;

	enc_R.sp_pulse = Encoder_GetPosition_Right();		Encoder_ResetPosition_Right();
	enc_L.sp_pulse = Encoder_GetPosition_Left();		Encoder_ResetPosition_Left();

	enc_R.prev_wheel_speed = enc_R.wheel_speed;
	enc_L.prev_wheel_speed = enc_L.wheel_speed;

	enc_R.wheel_speed =  (float)enc_R.sp_pulse * MMPP * m_dt; //計測はmm mm/ms-> m/s
	enc_L.wheel_speed =  (float)enc_L.sp_pulse * MMPP * m_dt;

	enc_sp = (enc_R.wheel_speed - enc_L.wheel_speed)/2.0f;
	machine.prev_velo = machine.velo;
	machine.prev_accel = machine.accel;
	//machine.velo = (enc_R.wheel_speed - enc_L.wheel_speed)/2.0;
	machine.velo = calc_speed_filter((-1.0f)*(read_accel_x_axis()), enc_sp);
	machine.length += enc_sp*interrupt_time*1000.0f;;
	machine.I_velo += machine.velo ;
    machine.accel = (-1.0f)*(read_accel_x_axis())*1.0 + machine.accel * 0.0;

	machine.prev_rad_velo = machine.rad_velo;
	machine.prev_rad_accel = machine.rad_accel;
	machine.rad_velo = (-1.0f)*read_gyro_z_axis()*PI/180.0f;
	machine.rad_accel = (machine.rad_velo-machine.prev_rad_velo)*inv_interrupt_time;
	machine.radian += machine.rad_velo*interrupt_time;
	machine.I_rad_velo += machine.rad_velo;
	fb_rad_velo = 0.95*fb_rad_velo + 0.05*machine.rad_velo;
}

void Interrupt_Set_Target_Speed(){

	static float turn_slip_acc = 0.0;
	target.prev_velo = target.velo;
	//target.accel = target.accel - turn_slip_acc;
	target.prev_accel = target.accel;
	target.velo += target.accel*interrupt_time;
	if(target.velo >= max_set.velo)
	{
		target.velo = max_set.velo;
		target.accel = 0.0f;
	}
	target.length += target.velo*interrupt_time*1000.0f;
	target.I_velo += target.velo;

	if(run_mode == TURN_MODE_TABLE){
		turn_time = turn_time + 1.0;
		if((float)turn_time < set_turn_time * 1000)
		{
			target.prev_rad_velo = target.rad_velo;
			target.prev_rad_accel = target.rad_accel;
			float m =((float)(turn_time))/set_turn_time - (float)((int)(((float)(turn_time))/set_turn_time));
			float n =(float)(((int)(((float)(turn_time))/set_turn_time))+1)- ((float)(turn_time))/set_turn_time;
			target.rad_velo = max_set.rad_velo*(n*accel_table[(int)(((float)(turn_time))/set_turn_time)] + m*accel_table[(int)(((float)(turn_time))/set_turn_time) + 1]);
			target.rad_accel = (target.rad_velo - target.prev_rad_velo)*1000.0;
		}
		else
		{
			target.rad_velo = 0.0;
		}
		target.radian += target.rad_velo*interrupt_time;
		target.I_rad_velo += target.rad_velo;
		//we neeed to think beta

		turn_slip_acc  = (1.0)*ABS(read_accel_y_axis()*read_accel_y_axis()/180.0);
		target.accel = turn_slip_acc;
	}
	else if(run_mode == STRAIGHT_MODE)
	{
		target.prev_rad_velo = target.rad_velo;
		target.prev_rad_accel = target.rad_accel;
		target.rad_velo = target.rad_velo + target.rad_accel*interrupt_time;
		if(ABS(target.rad_velo) >= ABS(max_set.rad_velo))
		{
			//target.rad_velo = max_set.rad_velo;
			//if(run_mode == TURN_MODE) target.rad_accel = 0.0f;
		}
		target.radian += target.rad_velo*interrupt_time;
		target.I_rad_velo += target.rad_velo;


		//turn_slip_acc  = (1.0)*ABS(read_accel_y_axis()*read_accel_y_axis()/60.0);
		//target.accel += turn_slip_acc;
	}
	else
	{
		target.prev_rad_velo = target.rad_velo;
		target.prev_rad_accel = target.rad_accel;
		target.rad_velo += target.rad_accel*interrupt_time;
		if(ABS(target.rad_velo) >= ABS(max_set.rad_velo))
		{
			target.rad_velo = max_set.rad_velo;
			if(run_mode == TURN_MODE) target.rad_accel = 0.0f;
		}
		target.radian += target.rad_velo*interrupt_time;
		target.I_rad_velo += target.rad_velo;

	}
}

/*
 * motor.c
 *
 *  Created on: Aug 12, 2022
 *      Author: sato1
 */

#include "index.h"

#define PCLK1			(HAL_RCC_GetPCLK1Freq())//25,000,000
#define PCLK2			(HAL_RCC_GetPCLK2Freq())//50,000,000
#define PWMFREQ			(200000)
#define FANPWMFREQ		(200000)
#define MOT_DUTY_MIN	(30)
#define MOT_DUTY_MAX	(950)

#define MOT_SET_COMPARE_L_FORWARD(x)	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, x)
#define MOT_SET_COMPARE_L_REVERSE(x)	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, x)
#define MOT_SET_COMPARE_R_FORWARD(x)	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, x)
#define MOT_SET_COMPARE_R_REVERSE(x)	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, x)

void Motor_Initialize()
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void Motor_Stop(){
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
	FAN_Motor_Stop();
}

void FAN_Motor_Initialize()
{
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
}

void FAN_Motor_Stop(){
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
}

void Motor_SetDuty_Left( int16_t duty_l )
{
	uint32_t	pulse_l;

	if( ABS(duty_l) > MOT_DUTY_MAX ) {
		pulse_l = (uint32_t)((PCLK1*2) / PWMFREQ * MOT_DUTY_MAX / 1000) - 1;
	} else if( ABS(duty_l) < MOT_DUTY_MIN ) {
		pulse_l = (uint32_t)((PCLK1*2) / PWMFREQ * MOT_DUTY_MIN / 1000) - 1;
	} else {
		pulse_l = (uint32_t)((PCLK1*2) / PWMFREQ * ABS(duty_l) / 1000) - 1;
	}

	if( duty_l > 0 ) {
		MOT_SET_COMPARE_L_FORWARD( pulse_l );
		MOT_SET_COMPARE_L_REVERSE( 0 );
	} else if( duty_l < 0 ) {
		MOT_SET_COMPARE_L_FORWARD( 0 );
		MOT_SET_COMPARE_L_REVERSE( pulse_l );
	} else {
		MOT_SET_COMPARE_L_FORWARD( 0 );
		MOT_SET_COMPARE_L_REVERSE( 0 );
	}
}

void Motor_SetDuty_Right( int16_t duty_r )
{
	uint32_t	pulse_r;

	if( ABS(duty_r) > MOT_DUTY_MAX ) {
		pulse_r = (uint32_t)((PCLK1*2) / PWMFREQ * MOT_DUTY_MAX / 1000) - 1;
	} else if( ABS(duty_r) < MOT_DUTY_MIN ) {
		pulse_r = (uint32_t)((PCLK1*2) / PWMFREQ * MOT_DUTY_MIN / 1000) - 1;
	} else {
		pulse_r = (uint32_t)((PCLK1*2) / PWMFREQ * ABS(duty_r) / 1000) - 1;
	}

	if( duty_r > 0 ) {
		MOT_SET_COMPARE_R_FORWARD( pulse_r );
		MOT_SET_COMPARE_R_REVERSE( 0 );
	} else if( duty_r < 0 ) {
		MOT_SET_COMPARE_R_FORWARD( 0 );
		MOT_SET_COMPARE_R_REVERSE( pulse_r );
	} else {
		MOT_SET_COMPARE_R_FORWARD( 0 );
		MOT_SET_COMPARE_R_REVERSE( 0 );
	}
}

void FAN_Motor_Test()
{
	int32_t max = 900;
	for(int i = 0; i < max;i++){
		FAN_Motor_SetDuty(i);
		HAL_Delay(2);
	}
	FAN_Motor_SetDuty(max);
	while(Mode_Start_photo_Sens() != true){
		HAL_Delay(2);
	}
	Check_LED_Toggle(5);
	for(int i = max; i >= 0; i--){
		FAN_Motor_SetDuty(i);
		HAL_Delay(2);
	}
	FAN_Motor_SetDuty(0);
	HAL_Delay(200);
}

void FAN_Motor_SetDuty(int16_t duty_f)
{
	uint32_t	pulse_f;

	if( ABS(duty_f) > MOT_DUTY_MAX ) {
		pulse_f = (uint32_t)((PCLK2)/FANPWMFREQ * MOT_DUTY_MAX / 1000) - 1;
	}else if(duty_f == 0){
		pulse_f = 0;
	}else if( ABS(duty_f) < MOT_DUTY_MIN ) {
		pulse_f = (uint32_t)((PCLK2)/FANPWMFREQ * MOT_DUTY_MIN / 1000) - 1;
	}else {
		pulse_f = (uint32_t)((PCLK2)/FANPWMFREQ * ABS(duty_f) / 1000) - 1;
	}
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pulse_f);
}


void Suction_start(int16_t max_duty)
{
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	for(int i = 0; i < max_duty;i++)
	{
		FAN_Motor_SetDuty(i);
		HAL_Delay(1);
	}
	FAN_Motor_SetDuty(max_duty);
	HAL_Delay(100);
}

void Suction_Stop()
{
	uint32_t pulse_f = __HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL_1);
	for(int i = pulse_f;i >=0;i--)
	{
		FAN_Motor_SetDuty(i);
		HAL_Delay(0);
	}
	FAN_Motor_SetDuty(0);
	HAL_Delay(100);
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
}



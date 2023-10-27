/*
 * encoder.c
 *
 *  Created on: Aug 12, 2022
 *      Author: sato1
 */

#include "index.h"

#define ENC_CNT_L 		(TIM4 -> CNT)
#define ENC_CNT_R 		(TIM3 -> CNT)

#define ENC_ZERO		(20000)


void Encoder_Initialize(){
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	Encoder_ResetPosition_Left();
	Encoder_ResetPosition_Right();
}

void Encoder_ResetPosition_Left(){
	ENC_CNT_L = ENC_ZERO;
}

void Encoder_ResetPosition_Right(){
	ENC_CNT_R = ENC_ZERO;
}

uint32_t Encoder_Counts_Left(){
	return (uint32_t)ENC_CNT_L;
}

uint32_t Encoder_Counts_Right(){
	return (uint32_t)ENC_CNT_R;
}

int32_t Encoder_GetPosition_Right(){
	return ((int32_t)ENC_CNT_R - (int32_t)ENC_ZERO);
}

int32_t Encoder_GetPosition_Left(){
	return -((int32_t)ENC_CNT_L - (int32_t)ENC_ZERO);
}

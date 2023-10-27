/*
 * indicator.c
 *
 *  Created on: Aug 13, 2022
 *      Author: sato1
 */


#include "index.h"

void Mode_LED(uint8_t led_num)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (led_num >> 4) & 0x01);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, (led_num >> 3) & 0x01);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, (led_num >> 2) & 0x01);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, (led_num >> 1) & 0x01);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, (led_num >> 0) & 0x01);
}

void Check_LED_Toggle(uint8_t count)
{
	for(int i = 0 ;i < count*2 ; i++){
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(50);
	}
}

void Right_Side_On_LED()
{
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
}

void Right_Side_Off_LED()
{
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
}

void Left_Side_On_LED()
{
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
}

void Left_Side_Off_LED()
{
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}

void Front_On_LED()
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
}

void Front_Off_LED()
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
}

void All_On_LED()
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
}

void All_Off_LED()
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
}

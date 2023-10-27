/*
 * interface.c
 *
 *  Created on: Aug 13, 2022
 *      Author: sato1
 */


#include "index.h"
#include "glob_var.h"

#define MAX_MODE_NUM 0x0f

void Mode_Change_ENC()
{
	if(enc_R.wheel_speed > 0.1){
		if(is_mode_enable == true) is_mode_enable = false;
		else mouse_mode = (mouse_mode == 0x0f) ? 0 : mouse_mode + 1 ;
		HAL_Delay(100);
	}
	else if(enc_R.wheel_speed < -0.1){
		if(is_mode_enable == true) is_mode_enable = false;
		else mouse_mode = (mouse_mode == 0) ? MAX_MODE_NUM : mouse_mode - 1 ;
		HAL_Delay(100);
	}

	if(enc_L.wheel_speed > 0.1){
		if(is_mode_enable == false) is_mode_enable = true;
		HAL_Delay(100);
	}

	Mode_LED(((uint8_t)is_mode_enable) << 4 |mouse_mode);
}


t_bool Mode_Start_photo_Sens()
{
	if((Sensor_GetValue(sensor_fl) + Sensor_GetValue(sensor_fr) + Sensor_GetValue(sensor_sl) + Sensor_GetValue(sensor_sr)) > 2000 * 4){
		return true;
	}
	return false;
}

t_bool Button_Read()
{
	t_bool button_data;
	if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin))
	{
		button_data = true;
	}
	else
	{
		button_data = false;
	}
	return button_data;
}

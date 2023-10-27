/*
 * ir_sensor.c
 *
 *  Created on: 2022/08/12
 *      Author: sato1
 */

#include "index.h"
#include "sensor_table.h"

#define NUM_ADC				(10)
#define GET_ADC_DATA(x)		adc_value[x-1]

#define	SENSOR_ALL_PATTERN		((photo1_Pin|photo2_Pin|photo3_Pin|photo4_Pin))


typedef  enum {
	LED_FL_ON 	= 5,
	LED_FL_OFF 	= 6,
	LED_SL_ON 	= 1,
	LED_SL_OFF 	= 2,
	LED_SR_ON 	= 3,
	LED_SR_OFF 	= 4,
	LED_FR_ON 	= 7,
	LED_FR_OFF 	= 8,
}t_sensor_mode;

static uint32_t		led_on_pattern[NUM_ADC]  	= {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};				// LED点灯コマンド

//static uint16_t		led_off_pattern[NUM_ADC] 	= {0x0000, 0x0000, 0x0000, 0x0000, 0x0000,0x0000, 0x0000, 0x0000, 0x0000, 0x0000};	// LED消灯コマンド


static uint32_t		led_off_pattern[] = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
										 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000};

static uint16_t		adc_value[NUM_ADC];		// AD変換値

void Sensor_TurnOffLED()
{
	for( int8_t i = 0; i < NUM_ADC; i++ ) {
		led_on_pattern[i] = 0x00000000;
		led_off_pattern[i] = (uint32_t)SENSOR_ALL_PATTERN << 16;
	}
}

void Sensor_TurnOnLED()
{
	Sensor_TurnOffLED();
	led_on_pattern[LED_SL_ON] = photo4_Pin;
	led_on_pattern[LED_SR_ON] = photo3_Pin;
	led_on_pattern[LED_FL_ON] = photo2_Pin;
	led_on_pattern[LED_FR_ON] = photo1_Pin;
	//led_on_pattern[LED_SL_OFF] = (uint32_t)LED1_Pin;
	//led_on_pattern[LED_FL_OFF] = (uint32_t)LED4_Pin;
}

void Sensor_Initialize()
{
	htim1.Instance->DIER |= TIM_DIER_CC1DE | TIM_DIER_CC2DE;
	__HAL_TIM_MOE_ENABLE(&htim1);
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
	Sensor_TurnOnLED();
	HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_CC1], (uint32_t)led_on_pattern,  (uint32_t)(&(GPIOC->BSRR)), NUM_ADC);//ODR
	HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_CC2], (uint32_t)led_off_pattern, (uint32_t)(&(GPIOC->BSRR)), NUM_ADC);//ODR
	Sensor_StartADC();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void Sensor_StartADC()
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_value, NUM_ADC);
}

void Sensor_StopADC()
{
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(GPIOC, SENSOR_ALL_PATTERN, GPIO_PIN_RESET);
}

int16_t ADC_get_value(int num)
{
	return adc_value[num];
}

int16_t Sensor_GetValue(t_sensor_dir dir)
{
	switch(dir)
	{
		case sensor_fl:
			return ((int16_t)adc_value[LED_FL_ON] - (int16_t)adc_value[LED_FL_OFF]);
			break;
		case sensor_fr:
			return ((int16_t)adc_value[LED_FR_ON] - (int16_t)adc_value[LED_FR_OFF]);
			break;
		case sensor_sr:
			return ((int16_t)adc_value[LED_SR_ON] - (int16_t)adc_value[LED_SR_OFF]);
			break;
		case sensor_sl:
			return ((int16_t)adc_value[LED_SL_ON] - (int16_t)adc_value[LED_SL_OFF]);
			break;
	}
	return 0;
}

float Sensor_CalcDistance(t_sensor_dir dir,int16_t value)
{
	float distance = 0.0f;
	int array_length = 0;
	int count = 0;
	float m,n;
	switch(dir)
	{
		case sensor_fr:
			array_length = sizeof(sens_front_length_table) / sizeof(uint16_t);
			if(value >= sens_fr_table[0]) distance = (float)sens_front_length_table[0];
			else if (value <= sens_fr_table[array_length-1]) distance = (float)sens_front_length_table[array_length-1];
			else{
				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_fr_table[count] && value > sens_fr_table[count+1]) break;
				}
				m = (float)(sens_fr_table[count] - value);
				n = (float)(value - sens_fr_table[count+1]);
				distance = (n*(float)sens_front_length_table[count] + m*(float)sens_front_length_table[count+1])/(m+n);
			}
			break;
		case sensor_fl:
			array_length = sizeof(sens_front_length_table) / sizeof(uint16_t);
			if(value >= sens_fl_table[0]) distance = (float)sens_front_length_table[0];
			else if (value <= sens_fl_table[array_length-1]) distance = (float)sens_front_length_table[array_length-1];
			else
			{
				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_fl_table[count] && value > sens_fl_table[count+1]) break;
				}
				m = (float)(sens_fl_table[count] - value);
				n = (float)(value - sens_fl_table[count+1]);
				distance = (n*(float)sens_front_length_table[count] + m*(float)sens_front_length_table[count+1])/(m+n);
			}
			break;
		case sensor_sr:
			array_length = sizeof(sens_side_length_table) / sizeof(uint16_t);
			if(value >= sens_sr_table[0]) distance = (float)sens_side_length_table[0];
			else if (value <= sens_sr_table[array_length-1]) distance = (float)sens_side_length_table[array_length-1];
			else
			{
				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_sr_table[count] && value > sens_sr_table[count+1]) break;
				}
				m = (float)(sens_sr_table[count] - value);
				n = (float)(value - sens_sr_table[count+1]);
				distance = (n*(float)sens_side_length_table[count] + m*(float)sens_side_length_table[count+1])/(m+n);
			}
			break;
		case sensor_sl:
			array_length = sizeof(sens_side_length_table) / sizeof(uint16_t);
			if(value >= sens_sl_table[0]) distance = (float)sens_side_length_table[0];
			else if (value <= sens_sl_table[array_length-1]) distance = (float)sens_side_length_table[array_length-1];
			else
			{
				for(count = 0; count < array_length-1;count++)
				{
					if(value <=sens_sl_table[count] && value > sens_sl_table[count+1]) break;
				}
				m = (float)(sens_sl_table[count] - value);
				n = (float)(value - sens_sl_table[count+1]);
				distance = (n*(float)sens_side_length_table[count] + m*(float)sens_side_length_table[count+1])/(m+n);
			}
			break;
	}
	return distance;
}

int16_t Sensor_GetBatteryValue(){
	return (adc_value[0] + adc_value[9])/2 ;
}

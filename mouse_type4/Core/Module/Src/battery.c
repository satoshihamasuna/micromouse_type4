/*
 * battery.c
 *
 *  Created on: Aug 12, 2022
 *      Author: sato1
 */

#include "index.h"

#define BATTRY_REFERENCE	(3.25f)
#define BATTERY_LIMIT		(7.2f)


float Battery_GetVoltage(){
	return (BATTRY_REFERENCE * (47.0f+10.0f)/(10.0f) * (float)Sensor_GetBatteryValue())/4096.f;
}

void Battery_LimiterVoltage()
{
	volatile int	i;
	volatile float	battery_voltage_average;

	for( i = 0; i < 10; i++) {
		HAL_Delay(5);
		battery_voltage_average += Battery_GetVoltage();
	}
	battery_voltage_average /= 10;

	if( battery_voltage_average < BATTERY_LIMIT ) {
		while( 1 ) {
			All_On_LED();
			HAL_Delay(200);
			All_Off_LED();
			HAL_Delay(200);
		}
	} else;
}

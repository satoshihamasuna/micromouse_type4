/*
 * imu.c
 *
 *  Created on: Aug 11, 2022
 *      Author: sato1
 */

#include "index.h"
#include "macro.h"
#include "icm_20648.h"

uint8_t imu_address = ACCEL_XOUT_H|0x80; //ACCEL_X_HIGH_BYTE
uint8_t imu_value[13];

int16_t accel_data[3];
int16_t gyro_data[3];

uint8_t read_byte(uint8_t reg){
	uint8_t val = 0x00;
	uint8_t dammy = 0x00;
	reg = reg | 0x80; //mask

	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &reg, 1 , 100);
	HAL_SPI_TransmitReceive(&hspi2, &dammy, &val, 1, 100);

	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin,GPIO_PIN_SET);

	return val;
}

void write_byte(uint8_t reg, uint8_t data){
	reg = reg & 0x7F;

	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &reg, 1 , 100);
	HAL_SPI_Transmit(&hspi2, &data, 1 , 100);
	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin,GPIO_PIN_SET);

}

void imu_initialize(){
	write_byte( PWR_MGMT_1    , 0x81 ); 	//Reset all register
	HAL_Delay(50);
	write_byte( USER_CTRL     , 0x10 );
	HAL_Delay(50);
	write_byte( PWR_MGMT_1    , 0x01 );		//set default value of 0x06
	HAL_Delay(50);

	write_byte( REG_BANK_SEL  , 0x20 );		//change user bank from bank0 to bank2
	HAL_Delay(50);
	write_byte( GYRO_CONFIG_1 , 0x3f );		//set low pass filter, Gyro_sensor_FS = 2000dps,
	//write_byte( GYRO_CONFIG_1 , 0x06 );		//set low pass filter, Gyro_sensor_FS = 2000dps,
	HAL_Delay(50);
	write_byte( ACCEL_CONFIG  , 0x23 );		//set low pass filter, Accel_sensor_FS = ,
	//write_byte( ACCEL_CONFIG  , 0x23 );		//set low pass filter, Accel_sensor_FS = ,
	HAL_Delay(50);
	write_byte( ACCEL_CONFIG_2, 0x00 );		//set low pass filter, Accel_sensor_FS = ,
	HAL_Delay(50);

	write_byte( REG_BANK_SEL  , 0x00 );		//change user bank from bank2 to bank0
	HAL_Delay(50);
	write_byte( PWR_MGMT_1    , 0x01 );		//set default value of 0x06,
	HAL_Delay(50);
}


void IMU_read_DMA_Start(){
	HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(&hspi2, &imu_address, imu_value, sizeof(imu_value)/sizeof(uint8_t));
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi){
		HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin,GPIO_PIN_SET);

		accel_data[x_axis] = (((int16_t)imu_value[1]<<8 ) | ( (int16_t)imu_value[2]&0x00ff ) );
		accel_data[y_axis] = (((int16_t)imu_value[3]<<8 ) | ( (int16_t)imu_value[4]&0x00ff ) );
		accel_data[z_axis] = (((int16_t)imu_value[5]<<8 ) | ( (int16_t)imu_value[6]&0x00ff ) );
		gyro_data[x_axis] = (((int16_t)imu_value[7]<<8 ) | ( (int16_t)imu_value[8]&0x00ff ) );
		gyro_data[y_axis] = (((int16_t)imu_value[9]<<8 ) | ( (int16_t)imu_value[10]&0x00ff ) );
		gyro_data[z_axis] = (((int16_t)imu_value[11]<<8 ) | ( (int16_t)imu_value[12]&0x00ff ) );

		//IMU_read_DMA_Start();
}

float read_gyro_x_axis(){
	return  (float)gyro_data[x_axis]*(1.0f) / GYRO_FS_SEL_3;
}

float read_gyro_y_axis(){
	return  (float)gyro_data[y_axis]*(1.0f) / GYRO_FS_SEL_3;
}

float read_gyro_z_axis(){
	return  (float)gyro_data[z_axis]*(1.0f) / GYRO_FS_SEL_3;
}

float read_accel_x_axis(){
	return  (float)accel_data[x_axis] / ACCEL_FS_SEL_1 * G;
}

float read_accel_y_axis(){
	return  (float)accel_data[y_axis] / ACCEL_FS_SEL_1 * G;
}

float read_accel_z_axis(){
	return  (float)accel_data[z_axis] / ACCEL_FS_SEL_1 * G;
}


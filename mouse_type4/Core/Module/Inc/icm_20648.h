/*
 * icm_20648.h
 *
 *  Created on: Aug 11, 2022
 *      Author: sato1
 */

#ifndef MODULE_INC_ICM_20648_H_
#define MODULE_INC_ICM_20648_H_

//USER BANK 0
#define WHO_AM_I		0x00
#define USER_CTRL		0x03
#define LP_CONFIG		0x05
#define PWR_MGMT_1		0x06
#define PWR_MGMT_2		0x07
#define ACCEL_XOUT_H	0x2D
#define ACCEL_XOUT_L	0x2E
#define ACCEL_YOUT_H	0x2F
#define ACCEL_YOUT_L	0x30
#define ACCEL_ZOUT_H	0x31
#define ACCEL_ZOUT_L	0x32
#define	GYRO_XOUT_H		0x33
#define GYRO_XOUT_L 	0x34
#define GYRO_YOUT_H		0x35
#define GYRO_YOUT_L		0x36
#define GYRO_ZOUT_H		0x37
#define GYRO_ZOUT_L		0x38
#define REG_BANK_SEL	0x7F

//USER BANK 2
#define GYRO_SMPLRT_DIV	0x00
#define GYRO_CONFIG_1	0x01
#define GYRO_CONFIG_2	0x02
#define ACCEL_CONFIG	0x14
#define ACCEL_CONFIG_2	0x15

#define GYRO_FS_SEL_0 (131.0f) 	//LSB/(dps) 250dps
#define GYRO_FS_SEL_1 (65.5f)  	//LSB/(dps) 500dps
#define GYRO_FS_SEL_2 (32.8f)  	//LSB/(dps) 1000dps
#define GYRO_FS_SEL_3 (16.364f)  //LSB/(dps) 2000dps

#define ACCEL_FS_SEL_0 (16384.0f) //LSB/(g) 2g
#define ACCEL_FS_SEL_1 (8192.0f)  //LSB/(g) 4g
#define ACCEL_FS_SEL_2 (4096.0f)  //LSB/(g) 8g
#define ACCEL_FS_SEL_3 (2048.0f)  //LSB/(g) 16g


#endif /* MODULE_INC_ICM_20648_H_ */

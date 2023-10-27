/*
 * index.h
 *
 *  Created on: Aug 11, 2022
 *      Author: sato1
 */

#ifndef MODULE_INC_INDEX_H_
#define MODULE_INC_INDEX_H_

#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "typedef.h"
#include "macro.h"


//IMU
uint8_t read_byte(uint8_t reg);
void write_byte(uint8_t reg, uint8_t data);
void imu_initialize();
void IMU_read_DMA_Start();
float read_gyro_x_axis();
float read_gyro_y_axis();
float read_gyro_z_axis();
float read_accel_x_axis();
float read_accel_y_axis();
float read_accel_z_axis();

//encoder
void Encoder_Initialize();
void Encoder_ResetPosition_Left();
void Encoder_ResetPosition_Right();
uint32_t Encoder_Counts_Left();
uint32_t Encoder_Counts_Right();
int32_t Encoder_GetPosition_Right();
int32_t Encoder_GetPosition_Left();

//ir_sensors
void Sensor_TurnOffLED();
void Sensor_TurnOnLED();
void Sensor_Initialize();
void Sensor_StartADC();
void Sensor_StopADC();
int16_t ADC_get_value(int num);
int16_t Sensor_GetValue(t_sensor_dir dir);
int16_t Sensor_GetBatteryValue();

//battery
float Battery_GetVoltage();
void Battery_LimiterVoltage();

//Motor
void Motor_Initialize();
void Motor_SetDuty_Left( int16_t duty_l );
void Motor_SetDuty_Right( int16_t duty_r );
void FAN_Motor_Initialize();
void FAN_Motor_SetDuty(int16_t duty_f);
void FAN_Motor_Test();
void Motor_Stop();
void FAN_Motor_Stop();
void Suction_start(int16_t max_duty);
void Suction_Stop();

//Interrupt
void Interrupt_Initialize();
void Interrupt_PreProcess();
void Interrupt_PostProcess();
void Interrupt_Main();
void Interrupt_Get_Irsensor();
void Interrupt_Get_Speed();
void Interrupt_Set_Target_Speed();
float Sensor_CalcDistance(t_sensor_dir dir,int16_t value);
void init_W_parameters();
//Indicator
void Mode_LED(uint8_t led_num);
void Check_LED_Toggle(uint8_t count);
void Right_Side_On_LED();
void Right_Side_Off_LED();
void Left_Side_On_LED();
void Left_Side_Off_LED();
void Front_On_LED();
void Front_Off_LED();
void All_On_LED();
void All_Off_LED();

//Interface
void Mode_Change_ENC();
t_bool Mode_Start_photo_Sens();
t_bool Button_Read();
//run
void Machine_Param_Initialize();
void Target_Param_Initialize();
void MAX_Param_Initialize();
void Sp_Param_I_Initialize(t_sp_param * sp_param);
void Sp_Param_Initialize(t_sp_param *sp_param);
void Sp_Param_rad_Initialize(t_sp_param * sp_param);
void Set_Velo_PID_Gain(float Kp,float Ki,float Kd);
void Set_Omega_PID_Gain(float Kp,float Ki,float Kd);
void Set_PID_Gain(t_pid_gain *pid_gain,float Kp,float Ki,float Kd);
void search_straight(float len_target,float acc,float max_sp,float end_sp);
void straight(float len_target,float acc,float max_sp,float end_sp);
void diagonal(float len_target,float acc,float max_sp,float end_sp);
void Spin_turn(float rad_target,float rad_acc,float max_rad_velo,t_turn_dir turn_dir);
void long_turn(const t_param* parameter,const t_straight_param* st_param);
void turn_v90(const t_param* parameter,const t_straight_param* di_param);
void turn_in(const t_param* parameter,const t_straight_param* st_param,const t_straight_param* di_param);
void turn_out(const t_param* parameter,const t_straight_param* st_param,const t_straight_param* di_param);
void turn90(const t_turn_param* param);
void turn90_table(const t_turn_param_table* param);
void search_turn90_table(const t_param* param);
void set_stop_wall(int millis);
void search_straight_update_maze(float len_target,float acc,float max_sp,float end_sp,int *x, int *y,int goal_size ,int mask);
void search_turn90_table_update_maze(const t_param* parameter,int *x, int *y,int goal_size ,int mask);
void search_straight_update_maze_zenmen(float len_target,float acc,float max_sp,float end_sp,int *x, int *y,int goal_size ,int mask);
void search_turn90_table_update_maze_zenmen(const t_param* parameter,int *x, int *y,int goal_size ,int mask);



//search
void init_maze();
void goal_set_vwall(int *gx,int *gy,int goal_size);
void goal_clear_vwall(int *gx,int *gy,int goal_size);
void search_adachi(int *gx,int *gy,int goal_size);
void search_adachi2(int *gx,int *gy,int goal_size);
void search_adachi3(int *gx,int *gy,int goal_size);
void search_adachi4(int *gx,int *gy,int goal_size,
		            const t_straight_param * base_straight_param,const t_straight_param * accel_straight_param,
					const t_param * turn_l_param				,const t_param * turn_r_param					);
void search_adachi_zenmen(int *gx,int *gy,int goal_size);
void search_adachi_zenmen2(int *gx,int *gy,int goal_size);
void search_adachi_zenmen3(int *gx,int *gy,int goal_size);
int get_nextdir(int *x, int *y,int goal_size ,int mask, t_direction *dir);
//flash
void write_save_data();
void read_save_data();
void save_data();

void disp_map();
void make_map_queue(int *x, int *y,int size,int mask);
void make_map_queue_zenmen(int *x, int *y,int size,int mask);
#endif /* MODULE_INC_INDEX_H_ */

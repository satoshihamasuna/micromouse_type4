/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "index.h"
#include "glob_var.h"
#include "run_param.h"
#include "icm_20648.h"
#include "run_param.h"
#include "operation_check.h"
#include "dijkstra.h"
#include "kalman_filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(200);
  setbuf(stdout, NULL);
  HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, 1);
  imu_initialize();
  IMU_read_DMA_Start();
  Encoder_Initialize();		//tim3,4
  Sensor_Initialize();		//tim1
  Interrupt_Initialize();	//tim5
  Motor_Initialize();		//tim2
  FAN_Motor_Initialize();	//tim8
  init_maze();

  int sx[1] = {0};		int sy[1] = {0};
  int gx[MAZE_GOAL_SIZE];	int gy[MAZE_GOAL_SIZE];
  for(int i = 0;i < MAZE_GOAL_SIZE;i++){
	  gx[i] = MAZE_GOAL_X + i;
	  gy[i] = MAZE_GOAL_Y + i;
  }

  enable_lsm = true;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(10);
	  Mode_Change_ENC();
	  Battery_LimiterVoltage();
	  switch((uint8_t)is_mode_enable << 4|mouse_mode){
	  	  case (MOUSE_ENABLE|0x00):
				//Suction_start((int16_t)((5.0)/Battery_GetVoltage()*1000.0));
	  		  	//Suction_start((int16_t)((5.0)/Battery_GetVoltage()*1000.0));
	  			while(Button_Read() != true){
	  				printf("fr:%4d,fl:%4d,sr:%4d,sl%4d,battery:%.2f\n",Sensor_GetValue(sensor_fr),Sensor_GetValue(sensor_fl),Sensor_GetValue(sensor_sr),Sensor_GetValue(sensor_sl),Battery_GetVoltage());
	  				printf("fr:%f,fl:%f\n",sen_fr.distance,sen_fl.distance);
		  		  	//if(Mode_Start_photo_Sens()) Suction_start((int16_t)((5.0)/Battery_GetVoltage()*1000.0));
	  				HAL_Delay(10);
	  			}
	  			Suction_Stop();

	  			is_mode_enable = false;
	  			break;
	  	  case (MOUSE_ENABLE|0x01):
	  			if(Mode_Start_photo_Sens())
	  			{
	  				Check_LED_Toggle(5);
	  				Set_Velo_PID_Gain(12.0,0.2,0.0);//14.0,0.1,0.0;
	  				Set_Omega_PID_Gain(0.4f, 0.001f, 0.0f);
	  				Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.15, 0.0, 0.02);
	  				Sp_Param_Initialize(&machine);
	  				Sp_Param_Initialize(&target);
	  				Sp_Param_Initialize(&max_set);
	  				Suction_start((int16_t)((8.0)/Battery_GetVoltage()*1000.0));HAL_Delay(300);
	  				wall_controll.is_controll = true;
	  				//wall_controll.is_controll = false;
	  				log_flag = true;time = 0;
	  				//slalom_check(&st_param_1000,mode_1000,Turn_in_L45);
	  				//straight(90.0*7+15.0,25.0,4.0,0.0);
	  				//slalom_check(&st_param_1200,mode_1200,Long_turnR180);
	  				//long_turn(mode_1200[Long_turnR180], &st_param_1200);
	  				//straight(90.0*1,4.0,0.3,0.0);
	  				straight(90.0*8,25.0,3.8,0.0);
	  				//slalom_check_R90();
	  				//set_stop_wall(2000);

	  				wall_controll.is_controll = false;
	  				Suction_Stop();
	  				log_flag = false;
	  				is_mode_enable = false;
	  			}
	  			break;
	  	  case (MOUSE_ENABLE|0x02):
	  			if(Mode_Start_photo_Sens())
	  			{
	  				Check_LED_Toggle(5);

	  				printf("target_velo,velo,target_rad_velo,rad_velo\n");
	  				HAL_Delay(2);
	  				for(int i = 0;i < LOG_COUNT;i++){
	  					printf("%.3f,%.3f,",log_data[0][i],log_data[1][i]);
	  					HAL_Delay(3);
	  					printf("%.3f,%.3f,",log_data[2][i],log_data[3][i]);
	  					HAL_Delay(3);
	  					printf("%.3f,%.3f,",log_data[4][i],log_data[5][i]);
	  					HAL_Delay(3);
	  					printf("%.3f,%.3f,\n",log_data[6][i],log_data[7][i]);
	  					HAL_Delay(3);
	  				}

	  				//disp_map();

	  				is_mode_enable = false;
	  			}
	  			break;
	  	  case (MOUSE_ENABLE|0x03):
	  			 if(Mode_Start_photo_Sens())
	  			 {

		  				Check_LED_Toggle(5);
		  				Set_Velo_PID_Gain(12.0,0.05,0.0);//14.0,0.1,0.0;
		  				Set_Omega_PID_Gain(0.1f, 0.01f, 0.0f);
		  				Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.15, 0.0, 0.02);
		  				Sp_Param_Initialize(&machine);
		  				Sp_Param_Initialize(&target);
		  				Sp_Param_Initialize(&max_set);
		  				Suction_start((int16_t)((8.0)/Battery_GetVoltage()*1000.0));HAL_Delay(300);
		  				wall_controll.is_controll = true;
		  				log_flag = true;time = 0;
		  				slalom_check(&st_param_1200,mode_1200,Turn_in_R45);
		  				//slalom_check_L90();
		  				wall_controll.is_controll = false;
		  				Suction_Stop();
		  				log_flag = false;
		  				is_mode_enable = false;
	  			 }
	  			break;
	  	  case (MOUSE_ENABLE|0x04):
	  		  	 if(Mode_Start_photo_Sens())
	  			 {
		  				Check_LED_Toggle(5);
		  				Set_Velo_PID_Gain(12.0,0.05,0.0);//14.0,0.1,0.0;
		  				Set_Omega_PID_Gain(0.1f, 0.01f, 0.0f);
		  				Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.15, 0.0, 0.02);
		  				Sp_Param_Initialize(&machine);
		  				Sp_Param_Initialize(&target);
		  				Sp_Param_Initialize(&max_set);
		  				Suction_start((int16_t)((9.0)/Battery_GetVoltage()*1000.0));HAL_Delay(300);
		  				wall_controll.is_controll = true;
		  				log_flag = true;time = 0;
		  				//slalom_check_R90();
		  				slalom_check(&st_param_1200,mode_1200,Turn_in_R135);
		  				wall_controll.is_controll = false;
		  				Suction_Stop();
		  				log_flag = false;
		  				is_mode_enable = false;
	  		  	 }
	  	  	  	HAL_Delay(10);
	  			break;
	  	  case (MOUSE_ENABLE|0x05):

  		  		if(Mode_Start_photo_Sens())
  			  	{
	  				 Check_LED_Toggle(5);
		  			 Sp_Param_Initialize(&machine);
		  			 Sp_Param_Initialize(&target);
		  			 Sp_Param_Initialize(&max_set);
		  			 filter_init();
					 mypos.x = mypos.y = 0;
					 mypos.dir = north;
					 Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
					 Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
					 const t_straight_param *stmode = &st_param_300;
					 //Suction_start((int16_t)((5.0)/Battery_GetVoltage()*1000.0));HAL_Delay(300);
					 straight(15.0,stmode->param->acc,stmode->param->max_velo,stmode->param->max_velo);
					 goal_set_vwall(gx, gy,MAZE_GOAL_SIZE);
					 //search_adachi3(gx, gy,MAZE_GOAL_SIZE);
					 search_adachi4(gx,gy,MAZE_GOAL_SIZE,
							 	 	&st_param_300		,&st_param_600,
									&param_L90_search	,&param_R90_search);

					 //Suction_Stop();HAL_Delay(100);
					 save_data();
					 HAL_Delay(100);
					 search_adachi_zenmen3(sx, sy,1);
					 save_data();
					 HAL_Delay(100);
		 			 Set_Omega_PID_Gain(0.6f, 0.01f, 0.0f);
			  		 Set_Velo_PID_Gain(12.0f, 0.1f, 0.0f);
		  			 Spin_turn(DEG2RAD(180.0), 20.0*PI, 2.0*PI, turn_left);
		  			 mypos.dir = (mypos.dir + 2 + 4) % 4;
					 HAL_Delay(100);
					 run_mode = NON_CON_MODE;
					 goal_clear_vwall(gx, gy,MAZE_GOAL_SIZE);
			  		 is_mode_enable = false;
  			  	}

  			  			/*
	  			if(Mode_Start_photo_Sens())
	  			{
	   				Check_LED_Toggle(5);
	  				Set_Velo_PID_Gain(12.0,0.05,0.0);//14.0,0.1,0.0;
	  				Set_Omega_PID_Gain(0.1f, 0.01f, 0.0f);
	  			  	Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.15, 0.0, 0.02);
	  			  	Sp_Param_Initialize(&machine);
	  			  	Sp_Param_Initialize(&target);
	  			  	Sp_Param_Initialize(&max_set);
	  			  	Suction_start((int16_t)((4.2)/Battery_GetVoltage()*1000.0));HAL_Delay(300);
	  			  	wall_controll.is_controll = true;
	  			  	log_flag = true;time = 0;
	  			  	slalom_check(&st_param_1200,mode_1200,Turn_in_R45);
	  			  	wall_controll.is_controll = false;
	  			  	Suction_Stop();
	  			  	log_flag = false;
	  			  	is_mode_enable = false;
	  		  	}
	  		  	*/
	  	  	  	HAL_Delay(10);
	  			break;
	  	  case (MOUSE_ENABLE|0x06):
	  		  			if(Mode_Start_photo_Sens())
	  		  			{
	  		   				Check_LED_Toggle(5);
	  		  				Set_Velo_PID_Gain(12.0,0.05,0.0);//14.0,0.1,0.0;
	  		  				Set_Omega_PID_Gain(0.1f, 0.01f, 0.0f);
	  		  			  	Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.15, 0.0, 0.02);
	  		  			  	Sp_Param_Initialize(&machine);
	  		  			  	Sp_Param_Initialize(&target);
	  		  			  	Sp_Param_Initialize(&max_set);
	  		  			  	Suction_start((int16_t)((6.0)/Battery_GetVoltage()*1000.0));HAL_Delay(300);
	  		  			  	wall_controll.is_controll = true;
	  		  			  	log_flag = true;time = 0;
	  		  			  	slalom_check(&st_param_1200,mode_1200,Long_turnR180);
	  		  			  	wall_controll.is_controll = false;
	  		  			  	Suction_Stop();
	  		  			  	log_flag = false;
	  		  			  	is_mode_enable = false;
	  		  		  	}
	  		  	  	  	HAL_Delay(10);
	  			break;
	  	  case (MOUSE_ENABLE|0x07):
  		  		if(Mode_Start_photo_Sens())
  		  		{
  		  			Check_LED_Toggle(5);
  		  			t_position start_pos = make_position(0,0,center);
		  			Sp_Param_Initialize(&machine);
		  			Sp_Param_Initialize(&target);
		  			Sp_Param_Initialize(&max_set);
		  			Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.2, 0.0, 0.01);
	  				Set_Omega_PID_Gain(0.6f, 0.01f, 0.0f);
		  			Set_Velo_PID_Gain(15.0f, 0.1f, 0.0f);
		  			filter_init();
		  			Suction_start((int16_t)((7.5)/Battery_GetVoltage()*1000.0));
  		  			printf("%d\n",(int)(sizeof(st_mode_500_v1)/sizeof(t_straight_param *const)));
  		  		    //log_flag = true;time = 0;
  		  			run_dijkstra(	st_mode_1000_v2, (int)(sizeof(st_mode_1000_v2)/sizeof(t_straight_param *const)),
  		  							di_mode_1000_v1, (int)(sizeof(di_mode_1000_v1)/sizeof(t_straight_param *const)),
									mode_1000,
									start_pos, north, gx, gy, MAZE_GOAL_SIZE);
		  			HAL_Delay(100);
		  			Suction_Stop();
  		  			is_mode_enable = false;
  		  			Sp_Param_Initialize(&machine);
  		  			Sp_Param_Initialize(&target);
  		  			Sp_Param_Initialize(&max_set);
  		  		}
	  			break;
	  	  case (MOUSE_ENABLE|0x08):
  		  		if(Mode_Start_photo_Sens())
  		  		{
  		  			Check_LED_Toggle(5);
					write_save_data();
					NVIC_SystemReset();
					is_mode_enable = false;
  		  		}
	  			break;
	  	  case (MOUSE_ENABLE|0x09):
  		  		if(Mode_Start_photo_Sens())
  		  		{
  		  		  	Check_LED_Toggle(5);
  		  		  	read_save_data();
  		  		  	HAL_Delay(10);
  		  			is_mode_enable = false;
  		  		}
	  			break;
	  	  case (MOUSE_ENABLE|0x0A):
  		  		if(Mode_Start_photo_Sens())
  		  		{
  		  			HAL_Delay(10);
  		  			disp_map();
  		  			is_mode_enable = false;
  		  		}
	  			break;
	  	  case (MOUSE_ENABLE|0x0B):
  		  		if(Mode_Start_photo_Sens())
  		  		{
  		   			Check_LED_Toggle(5);
  		  			t_position start_pos = make_position(0,0,center);
  		  			Sp_Param_Initialize(&machine);
  		  			Sp_Param_Initialize(&target);
  		  			Sp_Param_Initialize(&max_set);
  					Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.15, 0.0, 0.0);
  			  		Set_Omega_PID_Gain(0.6f, 0.01f, 0.0f);
  				  	Set_Velo_PID_Gain(15.0f, 0.1f, 0.0f);
  				  	filter_init();
  				  	//Suction_start((int16_t)((6.0)/Battery_GetVoltage()*1000.0));
  		  		  	printf("%d\n",(int)(sizeof(st_mode_1000_v0)/sizeof(t_straight_param *const)));
  		  		  	run_dijkstra(	st_mode_700_no_suction, (int)(sizeof(st_mode_700_no_suction)/sizeof(t_straight_param *const)),
  		  		  					di_mode_700_no_suction, (int)(sizeof(di_mode_700_no_suction)/sizeof(t_straight_param *const)),
									mode_700_no_suction,
									start_pos, north, gx, gy, MAZE_GOAL_SIZE);
  				  	HAL_Delay(100);
  				  	//Suction_Stop();
  		  		  	is_mode_enable = false;
  		  			Sp_Param_Initialize(&machine);
  		  			Sp_Param_Initialize(&target);
  		  			Sp_Param_Initialize(&max_set);
  		  			HAL_Delay(300);
  		  		}
	  	  	  	HAL_Delay(1);
	  			break;
	  	  case (MOUSE_ENABLE|0x0C):
  		  		  		if(Mode_Start_photo_Sens())
  		  		  		{
  		  		   			Check_LED_Toggle(5);
  		  		  			t_position start_pos = make_position(0,0,center);
  		  		  			Sp_Param_Initialize(&machine);
  		  		  			Sp_Param_Initialize(&target);
  		  		  			Sp_Param_Initialize(&max_set);
  		  					Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.2, 0.0, 0.01);
  		  			  		Set_Omega_PID_Gain(0.6f, 0.01f, 0.0f);
  		  				  	Set_Velo_PID_Gain(15.0f, 0.1f, 0.0f);
  		  				  	filter_init();
  		  				  	Suction_start((int16_t)((9.0)/Battery_GetVoltage()*1000.0));
  		  		  		  	printf("%d\n",(int)(sizeof(st_mode_1200_v0)/sizeof(t_straight_param *const)));
  		  		  		  	run_dijkstra(	st_mode_1300_v1, (int)(sizeof(st_mode_1300_v1)/sizeof(t_straight_param *const)),
  		  		  		  					di_mode_1300_v1, (int)(sizeof(di_mode_1300_v1)/sizeof(t_straight_param *const)),
  											mode_1300,
  											start_pos, north, gx, gy, MAZE_GOAL_SIZE);
  		  				  	HAL_Delay(100);
  		  				  	Suction_Stop();
  		  		  		  	is_mode_enable = false;
  		  		  			Sp_Param_Initialize(&machine);
  		  		  			Sp_Param_Initialize(&target);
  		  		  			Sp_Param_Initialize(&max_set);
  		  		  			HAL_Delay(300);
  		  		  		}
  			  	  	  	HAL_Delay(1);
	  	  case (MOUSE_ENABLE|0x0D):
  		  		  		if(Mode_Start_photo_Sens())
  		  		  		{
  		  		  			Check_LED_Toggle(5);
  		  		  			t_position start_pos = make_position(0,0,center);
  				  			Sp_Param_Initialize(&machine);
  				  			Sp_Param_Initialize(&target);
  				  			Sp_Param_Initialize(&max_set);
  				  			Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.2, 0.0, 0.01);
  			  				Set_Omega_PID_Gain(0.6f, 0.01f, 0.0f);
  				  			Set_Velo_PID_Gain(15.0f, 0.1f, 0.0f);
  				  			filter_init();
  				  			Suction_start((int16_t)((7.5)/Battery_GetVoltage()*1000.0));
  		  		  			printf("%d\n",(int)(sizeof(st_mode_500_v1)/sizeof(t_straight_param *const)));
  		  		  		    //log_flag = true;time = 0;
  		  		  			run_dijkstra(	st_mode_1000_v1, (int)(sizeof(st_mode_1000_v1)/sizeof(t_straight_param *const)),
  		  		  							di_mode_1000_v1, (int)(sizeof(di_mode_1000_v1)/sizeof(t_straight_param *const)),
  											mode_1000,
  											start_pos, north, gx, gy, MAZE_GOAL_SIZE);
  				  			HAL_Delay(100);
  				  			Suction_Stop();
  		  		  			is_mode_enable = false;
  		  		  			Sp_Param_Initialize(&machine);
  		  		  			Sp_Param_Initialize(&target);
  		  		  			Sp_Param_Initialize(&max_set);
  		  		  			HAL_Delay(300);
  		  		  		}
  			  			break;
	  	  case (MOUSE_ENABLE|0x0E):
  		  		  		if(Mode_Start_photo_Sens())
  		  		  		{
  		  		  			Check_LED_Toggle(5);
  		  		  			t_position start_pos = make_position(0,0,center);
  		  		  			Sp_Param_Initialize(&machine);
  		  		  			Sp_Param_Initialize(&target);
   		  		  			Sp_Param_Initialize(&max_set);
   		  					Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.2, 0.0, 0.01);
  		  			  		Set_Omega_PID_Gain(0.6f, 0.01f, 0.0f);
   		  				  	Set_Velo_PID_Gain(15.0f, 0.1f, 0.0f);
  		  				  	filter_init();
  		  		  		  	Suction_start((int16_t)((9.0)/Battery_GetVoltage()*1000.0));
  		  		  		  	printf("%d\n",(int)(sizeof(st_mode_1200_v0)/sizeof(t_straight_param *const)));
  		   		  		  	run_dijkstra(	st_mode_1200_v1, (int)(sizeof(st_mode_1200_v1)/sizeof(t_straight_param *const)),
  		   		  		  					di_mode_1200_v1, (int)(sizeof(di_mode_1200_v1)/sizeof(t_straight_param *const)),
  		  									mode_1200,
											start_pos, north, gx, gy, MAZE_GOAL_SIZE);
  		  		  			HAL_Delay(100);
  		  		  		  	Suction_Stop();
  		  		   		  	is_mode_enable = false;
  		  		  			Sp_Param_Initialize(&machine);
  		  		  			Sp_Param_Initialize(&target);
  		  		  			Sp_Param_Initialize(&max_set);
  		  		  			HAL_Delay(300);
   		  		  		}
	  			break;
	  	  case (MOUSE_ENABLE|0x0F):
  		  		  		  		if(Mode_Start_photo_Sens())
  		  		  		  		{
  		  		  		  			Check_LED_Toggle(5);
  		  		  		  			t_position start_pos = make_position(0,0,center);
  		  		  		  			Sp_Param_Initialize(&machine);
  		  		  		  			Sp_Param_Initialize(&target);
  		   		  		  			Sp_Param_Initialize(&max_set);
  		   		  					Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.2, 0.0, 0.01);
  		  		  			  		Set_Omega_PID_Gain(0.6f, 0.01f, 0.0f);
  		   		  				  	Set_Velo_PID_Gain(15.0f, 0.1f, 0.0f);
  		  		  				  	filter_init();
  		  		  		  		  	Suction_start((int16_t)((9.0)/Battery_GetVoltage()*1000.0));
  		  		  		  		  	printf("%d\n",(int)(sizeof(st_mode_1200_v0)/sizeof(t_straight_param *const)));
  		  		   		  		  	run_dijkstra(	st_mode_1200_v3, (int)(sizeof(st_mode_1200_v3)/sizeof(t_straight_param *const)),
  		  		   		  		  					di_mode_1200_v3, (int)(sizeof(di_mode_1200_v3)/sizeof(t_straight_param *const)),
  		  		  									mode_1200,
  													start_pos, north, gx, gy, MAZE_GOAL_SIZE);
  		  		  		  			HAL_Delay(100);
  		  		  		  		  	Suction_Stop();
  		  		  		   		  	is_mode_enable = false;
  		  		  		  			Sp_Param_Initialize(&machine);
  		  		  		  			Sp_Param_Initialize(&target);
  		  		  		  			Sp_Param_Initialize(&max_set);
  		  		  		  			HAL_Delay(300);
  		   		  		  		}
  			  			break;
	  			break;

	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1,(uint8_t *)ptr,len,10);
  return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

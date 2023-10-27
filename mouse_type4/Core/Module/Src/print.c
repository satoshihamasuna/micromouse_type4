/*
 * print.c
 *
 *  Created on: 2022/09/06
 *      Author: sato1
 */

#include "index.h"
#include "glob_var.h"
#include "macro.h"

void disp_map(){
	for( int y = MAZE_SIZE_Y - 1 ; y >= 0 ; y-- ){
		for(int x = 0; x < MAZE_SIZE_X ; x++ ){
			if(wall[x][y].north == WALL || wall[x][y].north == VWALL)	{	printf("+---");	HAL_Delay(10);	}
			else							{	printf("+   "); HAL_Delay(10);	}
			//if(x == MAZE_SIZE_X - 1)		{	printf("+\n");	HAL_Delay(5);	}
		}
		printf("+\n");	HAL_Delay(10);

		for(int x = 0; x < MAZE_SIZE_X ; x++ ){
			if(wall[x][y].west == WALL || wall[x][y].west == VWALL)		{	printf("|%3x",map[x][y]);	HAL_Delay(10);	}
			else							{	printf(" %3x",map[x][y]);	HAL_Delay(10);	}
			//if(x == MAZE_SIZE_X - 1)		{	printf("|\n");				HAL_Delay(5);	}
		}
		printf("|\n");				HAL_Delay(5);
	}
	for(int x = 0; x < MAZE_SIZE_X ; x++)	{	printf("+---"); HAL_Delay(5);	}	printf("+\n");
}

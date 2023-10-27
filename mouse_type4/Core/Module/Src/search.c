/*
 * search.c
 *
 *  Created on: 2022/08/25
 *      Author: sato1
 */


#include "index.h"
#include "macro.h"
#include "glob_var.h"
#include "run_param.h"
#include "typedef.h"
#include "queue.h"
#include "kalman_filter.h"

#define FAR_PRIORITY	0x01
#define NEAR_PRIORITY 	0x03

void init_map(int *x, int *y,int goal_size);				//歩数マップの初期化
void set_wall(int x,int y);				//壁情報の保存
t_bool is_unknown(int x,int y);			//未探索の区間かの判定
t_bool i_am_goal(int x,int y,int *gx,int *gy,int goal_size);
int get_priority(int x, int y, t_direction dir);			//優先度を取得(未探索，前方優先)
int get_nextdir(int *x, int *y,int goal_size ,int mask, t_direction *dir);			//次に行く方向の取得
int get_nextdir_zenmen(int *x, int *y,int goal_size ,int mask, t_direction *dir);			//次に行く方向の取得
//void search_adachi(int gx,int gy,int size);		//足立法
int end_search = 0;

//迷路の初期化
void init_maze(){
	for( int i = 0 ; i < MAZE_SIZE_X ; i++ ){
		for( int j = 0 ; j < MAZE_SIZE_Y ; j++ ){
			wall[i][j].north = wall[i][j].east = wall[i][j].south = wall[i][j].west = UNKNOWN;
		}
	}

	for( int i = 0 ; i < MAZE_SIZE_X ; i++ ){
		wall[i][0].south = WALL;				//南側の壁を追加する
		wall[i][MAZE_SIZE_Y - 1].north = WALL;	//北側の壁を追加する
	}

	for( int j = 0 ; j < MAZE_SIZE_Y ; j++ ){
		wall[0][j].west = WALL;					//西側の壁を追加する
		wall[MAZE_SIZE_X - 1][j].east = WALL;	//東側の壁を追加する
	}

	wall[0][0].east = wall[1][0].west = WALL;				//スタートの東側の壁を追加

}

void init_map(int *x, int *y,int goal_size){
	for( int i = 0; i < MAZE_SIZE_X ; i++ ){
		for( int j = 0 ; j < MAZE_SIZE_Y ; j++ ){
			map[i][j] = MAZE_SIZE;
		}
	}

	for(int i = 0;i < goal_size;i++){
		for(int j = 0;j < goal_size;j++){
			map[x[i]][y[j]] = 0;
		}
	}

}

void goal_set_vwall(int *gx,int *gy,int goal_size){
	if(goal_size == 3)
	{
		wall[gx[1]][gy[1]].north = wall[gx[1]][gy[1]].east = wall[gx[1]][gy[1]].south = wall[gx[1]][gy[1]].west = VWALL;
		wall[gx[1]][gy[2]].south = wall[gx[2]][gy[1]].west = wall[gx[1]][gy[0]].north = wall[gx[0]][gy[1]].east = VWALL;
	}

}

void goal_clear_vwall(int *gx,int *gy,int goal_size){
	if(goal_size == 3)
	{
		wall[gx[1]][gy[1]].north = wall[gx[1]][gy[1]].east = wall[gx[1]][gy[1]].south = wall[gx[1]][gy[1]].west = NOWALL;
		wall[gx[1]][gy[2]].south = wall[gx[2]][gy[1]].west = wall[gx[1]][gy[0]].north = wall[gx[0]][gy[1]].east = NOWALL;
	}
}


t_bool i_am_goal(int x,int y,int *gx,int *gy,int goal_size){
	t_bool flag = false;
	for (int i = 0; i < goal_size;i++){
		for(int j = 0; j < goal_size;j++){
			if(x == gx[i] && y == gy[j]) flag = true;
		}
	}
	return flag;
}

void make_map(int *x, int *y,int size,int mask)	//歩数マップを作成する
{
//座標x,yをゴールとした歩数Mapを作成する。
//maskの値(MASK_SEARCH or MASK_SECOND)によって、
//探索用の歩数Mapを作るか、最短走行の歩数Mapを作るかが切り替わる
	t_bool change_flag;			//Map作成終了を見極めるためのフラグ
    init_map(x,y,size);				//Mapを初期化する

	do
	{
		change_flag = false;				//変更がなかった場合にはループを抜ける
		for(int i = 0; i < MAZE_SIZE_X; i++)			//迷路の大きさ分ループ(x座標)
		{
			for(int j = 0; j < MAZE_SIZE_Y; j++)		//迷路の大きさ分ループ(y座標)
			{
				if(map[i][j] == MAZE_SIZE)		//MAP_MAX_VALUEの場合は次へ
				{
					continue;
				}

				if(j < MAZE_SIZE_Y-1)					//範囲チェック
				{
					if( (wall[i][j].north & mask) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
					{
						if(map[i][j+1] == MAZE_SIZE)			//まだ値が入っていなければ
						{
							map[i][j+1] = map[i][j] + 1;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
				}

				if(i < MAZE_SIZE_X-1)					//範囲チェック
				{
					if( (wall[i][j].east & mask) == NOWALL)		//壁がなければ
					{
						if(map[i+1][j] == MAZE_SIZE)			//値が入っていなければ
						{
							map[i+1][j] = map[i][j] + 1;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
				}

				if(j > 0)						//範囲チェック
				{
					if( (wall[i][j].south & mask) == NOWALL)	//壁がなければ
					{
						if(map[i][j-1] == MAZE_SIZE)			//値が入っていなければ
						{
							map[i][j-1] = map[i][j] + 1;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}
					}
				}

				if(i > 0)						//範囲チェック
				{
					if( (wall[i][j].west & mask) == NOWALL)		//壁がなければ
					{
						if(map[i-1][j] == MAZE_SIZE)			//値が入っていなければ
						{
							map[i-1][j] = map[i][j] + 1;	//値を代入
							change_flag = true;		//値が更新されたことを示す
						}

					}

				}

			}

		}

	}while(change_flag == true);	//全体を作り終わるまで待つ

}

void expand(t_queue *queue,t_MapNode n,int mask){
	if(n.st_y < MAZE_SIZE_Y-1)					//範囲チェック
	{
		if( (wall[n.st_x][n.st_y].north & mask) == NOWALL)	//壁がなければ(maskの意味はstatic_parametersを参照)
		{
			if(map[n.st_x][n.st_y+1] == MAZE_SIZE)			//まだ値が入っていなければ
			{
				map[n.st_x][n.st_y+1] = n.cost + 1;	//値を代入
				heap_push(queue,node_set(n.st_x,n.st_y+1,map[n.st_x][n.st_y+1],0));
			}
		}
	}

	if(n.st_x < MAZE_SIZE_X-1)					//範囲チェック
	{
		if( (wall[n.st_x][n.st_y].east & mask) == NOWALL)		//壁がなければ
		{
			if(map[n.st_x+1][n.st_y] == MAZE_SIZE)			//値が入っていなければ
			{
				map[n.st_x+1][n.st_y] = n.cost + 1;	//値を代入
				heap_push(queue,node_set(n.st_x+1,n.st_y,map[n.st_x+1][n.st_y],0));
			}
		}
	}

	if(n.st_y > 0)						//範囲チェック
	{
		if( (wall[n.st_x][n.st_y].south & mask) == NOWALL)	//壁がなければ
		{
			if(map[n.st_x][n.st_y-1] == MAZE_SIZE)			//値が入っていなければ
			{
				map[n.st_x][n.st_y-1] = n.cost + 1;	//値を代入
				heap_push(queue,node_set(n.st_x,n.st_y-1,map[n.st_x][n.st_y-1],0));
			}
		}
	}

	if(n.st_x > 0)						//範囲チェック
	{
		if( (wall[n.st_x][n.st_y].west & mask) == NOWALL)		//壁がなければ
		{
			if(map[n.st_x-1][n.st_y] == MAZE_SIZE)			//値が入っていなければ
			{
				map[n.st_x-1][n.st_y] = n.cost + 1;	//値を代入
				heap_push(queue,node_set(n.st_x-1,n.st_y,map[n.st_x-1][n.st_y],0));
			}

		}
	}

}



void make_map_queue(int *x, int *y,int size,int mask)
{
	//mapの初期化
		init_map(x,y,size);
	//heapの初期化
		t_queue heapq;
		list_init(&heapq);
		for(int i = 0;i < size;i++){
			for(int j = 0;j < size;j++){
				push(&heapq,node_set(x[i],y[j],0,0));
			}
		}
		build_heap(&heapq);

	    t_MapNode n;
		while(heapq.tail != -1){
			n = heap_pop(&heapq);
			expand(&heapq,n,mask);
			if(mypos.x == n.st_x && mypos.y == n.st_y)
				break;

		}
}


void make_map_queue_zenmen(int *x, int *y,int size,int mask){

//mapの初期化
//heapの初期化
	t_queue heapq;
	list_init(&heapq);
	for(int i = 0;i < MAZE_SIZE_X;i++){
		for(int j = 0;j <  MAZE_SIZE_Y;j++){
			if( is_unknown(i,j) == true){
				push(&heapq,node_set(i,j,0,0));
				map[i][j] = 0;
			}else{
				map[i][j] = MAZE_SIZE;
			}
		}
	}
	build_heap(&heapq);
	//print_heap(&heapq);

    t_MapNode n;
	while(heapq.tail != -1){
		n = heap_pop(&heapq);
		expand(&heapq,n,mask);

		if(mypos.x == n.st_x && mypos.y == n.st_y)
			break;

	}

	if(map[mypos.x][mypos.y] == MAZE_SIZE){
		init_map(x,y,size);
		//heapの初期化
		list_init(&heapq);
		for(int i = 0;i < size;i++){
			for(int j = 0;j < size;j++){
				push(&heapq,node_set(x[i],y[j],0,0));
			}
		}
		build_heap(&heapq);
		while(heapq.tail != -1){
			n = heap_pop(&heapq);
			expand(&heapq,n,mask);

			if(mypos.x == n.st_x && mypos.y == n.st_y)
				break;

		}
	}
}

t_bool is_unknown(int x, int y)	//指定された区画が未探索か否かを判断する関数 未探索:true　探索済:false
{
	//座標x,yが未探索区間か否かを調べる

	if((wall[x][y].north == UNKNOWN) || (wall[x][y].east == UNKNOWN) || (wall[x][y].south == UNKNOWN) || (wall[x][y].west == UNKNOWN))
	{			//どこかの壁情報が不明のままであれば
		return true;	//未探索
	}
	else
	{
		return false;	//探索済
	}
}

void set_wall(int x, int y)	//壁情報を記録
{
//引数の座標x,yに壁情報を書き込む
	int n_write,s_write,e_write,w_write;
	n_write = 0;
	s_write = 0;
    e_write = 0;
    w_write = 0;
	//自分の方向に応じて書き込むデータを生成
	//CONV_SEN2WALL()はmacro.hを参照
	switch(mypos.dir){
		case north:	//北を向いている時

			n_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//　前壁の有無を判断
			e_write = CONV_SEN2WALL(sen_r.is_wall);				//右の有無を判断
			w_write = CONV_SEN2WALL(sen_l.is_wall);				//左壁の有無を判断
			s_write = NOWALL;						//後ろは必ず壁がない

			break;

		case east:	//東を向いているとき

			e_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//前壁の有無を判断
			s_write = CONV_SEN2WALL(sen_r.is_wall);				//右壁の有無を判断
			n_write = CONV_SEN2WALL(sen_l.is_wall);				//左壁の有無を判断
			w_write = NOWALL;						//後ろは必ず壁がない

			break;

		case south:	//南を向いているとき

			s_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//前壁の有無を判断
			w_write = CONV_SEN2WALL(sen_r.is_wall);				//右壁の有無を判断
			e_write = CONV_SEN2WALL(sen_l.is_wall);				//左壁の有無を判断
			n_write = NOWALL;						//後ろは必ず壁がない

			break;

		case west:	//西を向いているとき

			w_write = CONV_SEN2WALL(sen_fr.is_wall || sen_fl.is_wall);	//前壁の有無を判断
			n_write = CONV_SEN2WALL(sen_r.is_wall);				//右壁の有無を判断
			s_write = CONV_SEN2WALL(sen_l.is_wall);				//左壁の有無を判断
			e_write = NOWALL;						//後ろは必ず壁がない

			break;

		case center:

			break;


	}

	if(wall[x][y].north == UNKNOWN || wall[x][y].north == n_write){
		wall[x][y].north = n_write;	//実際に壁情報を書き込み
	}
	else
	{
		wall[x][y].north = VWALL;	//実際に壁情報を書き込み
		n_write			 = VWALL;
	}


	if(wall[x][y].south == UNKNOWN || wall[x][y].south == s_write){
		wall[x][y].south = s_write;	//実際に壁情報を書き込み
	}
	else
	{
		wall[x][y].south = VWALL;	//実際に壁情報を書き込み
		s_write			 = VWALL;
	}

	if(wall[x][y].east == UNKNOWN || wall[x][y].east == e_write){
		wall[x][y].east = e_write;	//実際に壁情報を書き込み
	}
	else
	{
		wall[x][y].east  = VWALL;	//実際に壁情報を書き込み
		e_write			 = VWALL;
	}

	if(wall[x][y].west == UNKNOWN || wall[x][y].west == w_write){
		wall[x][y].west = w_write;	//実際に壁情報を書き込み
	}
	else
	{
		wall[x][y].west  = VWALL;	//実際に壁情報を書き込み
		w_write			 = VWALL;
	}

	/*
	wall[x][y].north = n_write;
	wall[x][y].south = s_write;	//実際に壁情報を書き込み
	wall[x][y].east  = e_write;	//実際に壁情報を書き込み
	wall[x][y].west  = w_write;	//実際に壁情報を書き込み
	*/
	if(y < MAZE_SIZE_Y-1)	//範囲チェック
	{
		wall[x][y+1].south = n_write;	//反対側から見た壁を書き込み
	}

	if(x < MAZE_SIZE_X-1)	//範囲チェック
	{
		wall[x+1][y].west = e_write;	//反対側から見た壁を書き込み
	}

	if(y > 0)	//範囲チェック
	{
        wall[x][y-1].north = s_write;	//反対側から見た壁を書き込み
	}

	if(x > 0)	//範囲チェック
	{
		wall[x-1][y].east = w_write;	//反対側から見た壁を書き込み
	}

}

int get_priority(int x, int y, t_direction dir){	//そのマスの情報から、優先度を算出する

	int priority;	//優先度を記録する変数
	priority = 0;

	if(mypos.dir == dir){				//行きたい方向が現在の進行方向と同じ場合
		priority = 2;
	}else if( ((4+mypos.dir-dir)%4) == 2){		//行きたい方向が現在の進行方向と逆の場合
		priority = 0;
	}else{						//それ以外(左右どちらか)の場合
		priority = 1;
	}


	if(is_unknown(x,y) == true){
		priority += 4;				//未探索の場合優先度をさらに付加
	}

	return priority;				//優先度を返す

}

int get_priority2(int *x,int *y,int nx, int ny, t_direction dir ,unsigned char distance_priority){	//そのマスの情報から、優先度を算出する

	int priority;	//優先度を記録する変数
	priority = 0;

	if(mypos.dir == dir){				//行きたい方向が現在の進行方向と同じ場合
		priority = 2;
	}else if( ((4+mypos.dir-dir)%4) == 2){		//行きたい方向が現在の進行方向と逆の場合
		priority = 0;
	}else{						//それ以外(左右どちらか)の場合
		priority = 1;
	}


	if(is_unknown(nx,ny) == true){
		if(distance_priority == FAR_PRIORITY) priority = ((x[0]-nx)*(x[0]-nx)+(y[0]-ny)*(y[0]-ny));				//未探索の場合優先度をさらに付加
		else								  priority = 2*MAZE_SIZE - ((x[0]-nx)*(x[0]-nx)+(y[0]-ny)*(y[0]-ny));
	}

	return priority;				//優先度を返す

}

int get_nextdir(int *x, int *y,int goal_size ,int mask, t_direction *dir){
    int little,priority,tmp_priority;

    //make_map_queue(x,y,goal_size,mask);

    little = MAP_MAX_VALUE+1;

    priority = -1;

    if((wall[mypos.x][mypos.y].north & mask) == NOWALL){
        tmp_priority = get_priority(mypos.x,mypos.y + 1,north);
        if(map[mypos.x][mypos.y+1] < little){
            little = map[mypos.x][mypos.y+1];
            *dir = north;
            priority = tmp_priority;
        }else if(map[mypos.x][mypos.y+1] == little){
            if(priority < tmp_priority){
                *dir = north;
                priority = tmp_priority;
            }
        }
    }

    if((wall[mypos.x][mypos.y].east & mask) == NOWALL){
        tmp_priority = get_priority(mypos.x + 1,mypos.y,east);
        if(map[mypos.x + 1][mypos.y] < little){
            little = map[mypos.x+1][mypos.y];
            *dir = east;
            priority = tmp_priority;
        }else if(map[mypos.x + 1][mypos.y] == little){
            if(priority < tmp_priority){
                *dir = east;
                priority = tmp_priority;
            }
        }
    }

    if((wall[mypos.x][mypos.y].south & mask) == NOWALL){
        tmp_priority = get_priority(mypos.x,mypos.y-1,south);
        if(map[mypos.x][mypos.y-1] < little){
            little = map[mypos.x][mypos.y-1];
            *dir = south;
            priority = tmp_priority;
        }else if(map[mypos.x][mypos.y-1] == little){
            if(priority < tmp_priority){
                *dir = south;
                priority = tmp_priority;
            }
        }
    }

    if((wall[mypos.x][mypos.y].west & mask) == NOWALL){
        tmp_priority = get_priority(mypos.x-1,mypos.y,west);
        if(map[mypos.x-1][mypos.y] < little){
            little = map[mypos.x-1][mypos.y];
            *dir = west;
            priority = tmp_priority;
        }else if(map[mypos.x-1][mypos.y] == little){
            if(priority < tmp_priority){
                *dir = west;
                priority = tmp_priority;
            }
        }
    }

    return ((int)((4+*dir - mypos.dir)%4));
}

int get_nextdir_zenmen(int *x, int *y,int goal_size ,int mask, t_direction *dir){
    int little,priority,tmp_priority;

    //make_map_queue_zenmen(x,y,goal_size,mask);

    little = MAP_MAX_VALUE;

    priority = -1;

    if((wall[mypos.x][mypos.y].north & mask) == NOWALL){
        tmp_priority = get_priority(mypos.x,mypos.y + 1,north);
        if(map[mypos.x][mypos.y+1] < little){
            little = map[mypos.x][mypos.y+1];
            *dir = north;
            priority = tmp_priority;
        }else if(map[mypos.x][mypos.y+1] == little){
            if(priority < tmp_priority){
                *dir = north;
                priority = tmp_priority;
            }
        }
    }

    if((wall[mypos.x][mypos.y].east & mask) == NOWALL){
        tmp_priority = get_priority(mypos.x + 1,mypos.y,east);
        if(map[mypos.x + 1][mypos.y] < little){
            little = map[mypos.x+1][mypos.y];
            *dir = east;
            priority = tmp_priority;
        }else if(map[mypos.x + 1][mypos.y] == little){
            if(priority < tmp_priority){
                *dir = east;
                priority = tmp_priority;
            }
        }
    }

    if((wall[mypos.x][mypos.y].south & mask) == NOWALL){
        tmp_priority = get_priority(mypos.x,mypos.y-1,south);
        if(map[mypos.x][mypos.y-1] < little){
            little = map[mypos.x][mypos.y-1];
            *dir = south;
            priority = tmp_priority;
        }else if(map[mypos.x][mypos.y-1] == little){
            if(priority < tmp_priority){
                *dir = south;
                priority = tmp_priority;
            }
        }
    }

    if((wall[mypos.x][mypos.y].west & mask) == NOWALL){
        tmp_priority = get_priority(mypos.x-1,mypos.y,west);
        if(map[mypos.x-1][mypos.y] < little){
            little = map[mypos.x-1][mypos.y];
            *dir = west;
            priority = tmp_priority;
        }else if(map[mypos.x-1][mypos.y] == little){
            if(priority < tmp_priority){
                *dir = west;
                priority = tmp_priority;
            }
        }
    }

    return ((int)((4+*dir - mypos.dir)%4));
}

int get_tmp_nextdir(int *x, int *y,int goal_size ,int mask,t_position tmp_pos, t_direction *dir,unsigned char distance_priority){
    int little,priority,tmp_priority;

    //make_map_queue(x,y,goal_size,mask);

    little = MAP_MAX_VALUE;

    priority = -1;

    if((wall[tmp_pos.x][tmp_pos.y].north & mask) == NOWALL){
        tmp_priority = get_priority2(x,y,tmp_pos.x,tmp_pos.y + 1,north,distance_priority);
        if(map[tmp_pos.x][tmp_pos.y+1] < little){
            little = map[tmp_pos.x][tmp_pos.y+1];
            *dir = north;
            priority = tmp_priority;
        }else if(map[tmp_pos.x][tmp_pos.y+1] == little){
            if(priority < tmp_priority){
                *dir = north;
                priority = tmp_priority;
            }
        }
    }

    if((wall[tmp_pos.x][tmp_pos.y].east & mask) == NOWALL){
        tmp_priority = get_priority2(x,y,tmp_pos.x + 1,tmp_pos.y,east,distance_priority);
        if(map[tmp_pos.x + 1][tmp_pos.y] < little){
            little = map[tmp_pos.x+1][tmp_pos.y];
            *dir = east;
            priority = tmp_priority;
        }else if(map[tmp_pos.x + 1][tmp_pos.y] == little){
            if(priority < tmp_priority){
                *dir = east;
                priority = tmp_priority;
            }
        }
    }

    if((wall[tmp_pos.x][tmp_pos.y].south & mask) == NOWALL){
        tmp_priority = get_priority2(x,y,tmp_pos.x,tmp_pos.y-1,south,distance_priority);
        if(map[tmp_pos.x][tmp_pos.y-1] < little){
            little = map[tmp_pos.x][tmp_pos.y-1];
            *dir = south;
            priority = tmp_priority;
        }else if(map[tmp_pos.x][tmp_pos.y-1] == little){
            if(priority < tmp_priority){
                *dir = south;
                priority = tmp_priority;
            }
        }
    }

    if((wall[tmp_pos.x][tmp_pos.y].west & mask) == NOWALL){
        tmp_priority = get_priority2(x,y,tmp_pos.x-1,tmp_pos.y,west,distance_priority);
        if(map[tmp_pos.x-1][tmp_pos.y] < little){
            little = map[tmp_pos.x-1][tmp_pos.y];
            *dir = west;
            priority = tmp_priority;
        }else if(map[tmp_pos.x-1][tmp_pos.y] == little){
            if(priority < tmp_priority){
                *dir = west;
                priority = tmp_priority;
            }
        }
    }

    return ((int)((4+*dir - tmp_pos.dir)%4));
}

t_bool is_wall_dir(int x,int y,t_direction dir)
{
	t_bool is_wall = false;
	switch(dir)
	{
	case north:
		if( wall[x][y].north == WALL) is_wall = true;
		else 								is_wall = false;
		break;
	case east:
		if(wall[x][y].east == WALL) is_wall = true;
		else 								is_wall = false;
		break;
	case south:
		if(wall[x][y].south == WALL) is_wall = true;
		else 								is_wall = false;
		break;
	case west:
		if(wall[x][y].west  == WALL) is_wall = true;
		else 								is_wall = false;
		break;
	case center:
		is_wall = false;
		break;
	}
	return is_wall;
}

void search_adachi(int *gx,int *gy,int goal_size){
    t_direction glob_nextdir;
    int direction = get_nextdir(gx,gy,goal_size,MASK_SEARCH,&glob_nextdir);

	Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);

	Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.15, 0.0, 0.0);

	mypos.dir = glob_nextdir;
    switch(mypos.dir){
        case north:
            mypos.y++;
            break;
        case east:
            mypos.x++;
            break;
        case south:
            mypos.y--;
            break;
        case west:
            mypos.x--;
            break;
        case center:
        	break;
    }

    switch(direction){
        case front:
        	wall_controll.is_controll = true;
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            straight(HALF_SECTION,4.0,0.3,0.3);
            break;
        case right:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
            straight(HALF_SECTION,4.0,0.3,0.3);
            break;
        case left:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
            straight(HALF_SECTION,4.0,0.3,0.3);
            break;
        case rear:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(180.0), 20.0*PI, 2.0*PI, turn_left);
            straight(HALF_SECTION,4.0,0.3,0.3);
		    break;
    }


    while(i_am_goal(mypos.x,mypos.y,gx,gy,goal_size) == false){
    	t_position prev_pos = mypos;
    	set_wall(mypos.x,mypos.y);
        direction = get_nextdir(gx,gy,goal_size,MASK_SEARCH,&glob_nextdir);

        mypos.dir = glob_nextdir;

        switch(mypos.dir){
            case north:
                mypos.y++;
                break;
            case east:
                mypos.x++;
                break;
            case south:
                mypos.y--;
                break;
            case west:
                mypos.x--;
                break;
            case center:
            	break;
        }

        switch(direction){
            case front:
    			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
      			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
	  			wall_controll.is_controll = true;
	  			search_straight(SECTION,4.0,0.3,0.3);
            	break;
            case right:;
 				wall_controll.is_controll = true;
            	search_turn90_table(&param_R90_search);
    			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
      			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            	break;
            case left:
            	wall_controll.is_controll = true;
 				search_turn90_table(&param_L90_search);
 				Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
 	  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            	break;
            case rear:
            	wall_controll.is_controll = false;
    			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
      			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
                straight(HALF_SECTION,4.0,0.3,0.0);
                if(is_wall_dir(prev_pos.x, prev_pos.y, prev_pos.dir) == true) set_stop_wall(200);
                if(is_wall_dir(prev_pos.x, prev_pos.y, (prev_pos.dir + 1 + 4)%4) == true)
                {
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
                	set_stop_wall(200);
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
                }
                else if(is_wall_dir(prev_pos.x, prev_pos.y, (prev_pos.dir - 1 + 4)%4) == true)
                {
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
                	set_stop_wall(200);
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
                }
                else
                {
                	Spin_turn(DEG2RAD(180.0), 20.0*PI, 2.0*PI, turn_left);
                }
            	wall_controll.is_controll = false;
            	filter_init();
                straight(HALF_SECTION,4.0,0.3,0.3);
                break;
        }

    }
    set_wall(mypos.x,mypos.y);		//壁をセット
    wall_controll.is_controll = false;
    straight(HALF_SECTION,4.0,0.3,0.0);
    run_mode = NON_CON_MODE;
}

void search_adachi2(int *gx,int *gy,int goal_size){
    t_direction glob_nextdir;
    //init_map(gx, gy, goal_size);
    make_map_queue(gx, gy, goal_size, MASK_SEARCH);
    int direction = get_nextdir(gx,gy,goal_size,MASK_SEARCH,&glob_nextdir);

	Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);

	Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.15, 0.0, 0.0);

	mypos.dir = glob_nextdir;
    switch(mypos.dir){
        case north:
            mypos.y++;
            break;
        case east:
            mypos.x++;
            break;
        case south:
            mypos.y--;
            break;
        case west:
            mypos.x--;
            break;
        case center:
        	break;
    }

    switch(direction){
        case front:
        	wall_controll.is_controll = true;
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            straight(HALF_SECTION,4.0,0.3,0.3);
            break;
        case right:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
            straight(HALF_SECTION,4.0,0.3,0.3);
            break;
        case left:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
            straight(HALF_SECTION,4.0,0.3,0.3);
            break;
        case rear:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(180.0), 20.0*PI, 2.0*PI, turn_left);
            straight(HALF_SECTION,4.0,0.3,0.3);
		    break;
    }

    int segment_accel_cnt = 0;
    while(i_am_goal(mypos.x,mypos.y,gx,gy,goal_size) == false){
    	t_position prev_pos = mypos;

    	if(is_unknown(mypos.x, mypos.y)){
    		set_wall(mypos.x,mypos.y);
    	}

        direction = get_nextdir(gx,gy,goal_size,MASK_SEARCH,&glob_nextdir);
        mypos.dir = glob_nextdir;

        switch(mypos.dir){
            case north:
                mypos.y++;
                break;
            case east:
                mypos.x++;
                break;
            case south:
                mypos.y--;
                break;
            case west:
                mypos.x--;
                break;
            case center:
            	break;
        }

        uint8_t next_step_flag = 0x00;
        if(is_unknown(mypos.x, mypos.y) == false)
        {
        	next_step_flag = 0x80;
        }

        switch(direction|next_step_flag){
            case front:
    			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
      			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
	  			wall_controll.is_controll = true;
	  			if(segment_accel_cnt > 1)
	  			{
	  				FAN_Motor_SetDuty(300);
	    			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	      			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
		  			wall_controll.is_controll = true;
	  				for(int i = 0; i < segment_accel_cnt - 1;i++)
	  				{
	  					search_straight_update_maze(SECTION,5.0,0.6,0.6, gx,gy,goal_size,MASK_SEARCH);
						//search_straight(SECTION,5.0,0.6,0.6);

	  				}
	  				search_straight_update_maze(SECTION,5.0,0.6,0.3, gx,gy,goal_size,MASK_SEARCH);
	  				//search_straight(SECTION,5.0,0.6,0.3);
	  				FAN_Motor_SetDuty(0);
	  				segment_accel_cnt = 0;
	  			}
	  			else if(segment_accel_cnt == 1)
	  			{
	    			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	      			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
		  			wall_controll.is_controll = true;
		  			search_straight_update_maze(SECTION,4.0,0.3,0.3, gx,gy,goal_size,MASK_SEARCH);
		  			//search_straight(SECTION,4.0,0.3,0.3);
	  				segment_accel_cnt = 0;
	  			}
	  			search_straight_update_maze(SECTION,4.0,0.3,0.3, gx,gy,goal_size,MASK_SEARCH);
            	break;

            case right:
            case (right|0x80):
				if(segment_accel_cnt > 1)
				{
					//FAN_Motor_SetDuty(300);
					Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
					Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
					wall_controll.is_controll = true;
					for(int i = 0; i < segment_accel_cnt - 1;i++)
					{
						search_straight_update_maze(SECTION,5.0,0.6,0.6, gx,gy,goal_size,MASK_SEARCH);
						//search_straight(SECTION,5.0,0.6,0.6);
					}
					search_straight_update_maze(SECTION,5.0,0.6,0.3, gx,gy,goal_size,MASK_SEARCH);
					//search_straight(SECTION,5.0,0.6,0.3);
					FAN_Motor_SetDuty(0);
					segment_accel_cnt = 0;
				}
				else if(segment_accel_cnt == 1)
				{
					Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
					Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
					wall_controll.is_controll = true;
					search_straight_update_maze(SECTION,5.0,0.3,0.3, gx,gy,goal_size,MASK_SEARCH);
					//search_straight(SECTION,4.0,0.3,0.3);
					segment_accel_cnt = 0;
				}
 				wall_controll.is_controll = true;
            	//search_turn90_table(&param_R90_search);
            	search_turn90_table_update_maze(&param_R90_search, gx, gy, goal_size, MASK_SEARCH);
    			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
      			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            	break;
            case left:
            case (left|0x80):
				if(segment_accel_cnt > 1)
				{
					//FAN_Motor_SetDuty(300);
					Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
					Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
					wall_controll.is_controll = true;
					for(int i = 0; i < segment_accel_cnt - 1;i++)
					{
						search_straight_update_maze(SECTION,5.0,0.6,0.6, gx,gy,goal_size,MASK_SEARCH);
						//search_straight(SECTION,5.0,0.6,0.6);
					}
					search_straight_update_maze(SECTION,5.0,0.6,0.3, gx,gy,goal_size,MASK_SEARCH);
					//search_straight(SECTION,5.0,0.6,0.3);
					FAN_Motor_SetDuty(0);
					segment_accel_cnt = 0;
				}
				else if(segment_accel_cnt == 1)
				{
					Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
					Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
					wall_controll.is_controll = true;
					//search_straight(SECTION,4.0,0.3,0.3);
					search_straight_update_maze(SECTION,5.0,0.3,0.3, gx,gy,goal_size,MASK_SEARCH);
					segment_accel_cnt = 0;
				}
            	wall_controll.is_controll = true;
 				//search_turn90_table(&param_L90_search);
 				search_turn90_table_update_maze(&param_L90_search, gx, gy, goal_size, MASK_SEARCH);
 				Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
 	  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            	break;
            case rear:
            case (rear|0x80):
				if(segment_accel_cnt > 1)
				{
					//FAN_Motor_SetDuty(300);
					Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
					Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
					wall_controll.is_controll = true;
					for(int i = 0; i < segment_accel_cnt - 1;i++)
					{
						//search_straight(SECTION,5.0,0.6,0.6);
						search_straight_update_maze(SECTION,5.0,0.6,0.6, gx,gy,goal_size,MASK_SEARCH);
					}
					//search_straight(SECTION,5.0,0.6,0.3);
					search_straight_update_maze(SECTION,5.0,0.6,0.3, gx,gy,goal_size,MASK_SEARCH);
					FAN_Motor_SetDuty(0);
					segment_accel_cnt = 0;
				}
				else if(segment_accel_cnt == 1)
				{
					Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
					Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
					wall_controll.is_controll = true;
					//search_straight(SECTION,4.0,0.3,0.3);
					search_straight_update_maze(SECTION,5.0,0.3,0.3, gx,gy,goal_size,MASK_SEARCH);
					segment_accel_cnt = 0;
				}
            	wall_controll.is_controll = false;
    			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
      			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
                straight(HALF_SECTION,4.0,0.3,0.0);
                if(is_wall_dir(prev_pos.x, prev_pos.y, prev_pos.dir) == true) set_stop_wall(200);
                if(is_wall_dir(prev_pos.x, prev_pos.y, (prev_pos.dir + 1 + 4)%4) == true)
                {
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
                	set_stop_wall(200);
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
                }
                else if(is_wall_dir(prev_pos.x, prev_pos.y, (prev_pos.dir - 1 + 4)%4) == true)
                {
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
                	set_stop_wall(200);
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
                }
                else
                {
                	Spin_turn(DEG2RAD(180.0), 20.0*PI, 2.0*PI, turn_left);
                }
            	wall_controll.is_controll = false;
            	filter_init();
                //straight(HALF_SECTION,4.0,0.3,0.3);
            	search_straight_update_maze(HALF_SECTION,5.0,0.3,0.3, gx,gy,goal_size,MASK_SEARCH);
            	break;
            case (front|0x80):
            	segment_accel_cnt++;
            	break;
        }

    }

	if(segment_accel_cnt > 1)
	{
		//FAN_Motor_SetDuty(300);
		Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
		Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
		wall_controll.is_controll = true;
		for(int i = 0; i < segment_accel_cnt - 1;i++)
		{
			search_straight(SECTION,5.0,0.6,0.6);
		}
		search_straight(SECTION,5.0,0.6,0.3);
		segment_accel_cnt = 0;
		FAN_Motor_SetDuty(0);
	}
	else if(segment_accel_cnt == 1)
	{
		Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
		Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
		wall_controll.is_controll = true;
		search_straight(SECTION,4.0,0.3,0.3);
		segment_accel_cnt = 0;
	}
	if(is_unknown(mypos.x, mypos.y)){
		set_wall(mypos.x,mypos.y);//壁情報セット
	}
    wall_controll.is_controll = false;
    straight(HALF_SECTION,4.0,0.3,0.0);
    run_mode = NON_CON_MODE;
}


void search_adachi3(int *gx,int *gy,int goal_size){
    t_direction glob_nextdir;
    int direction = get_tmp_nextdir(gx,gy,goal_size,MASK_SEARCH,mypos,&glob_nextdir,NEAR_PRIORITY);
	Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
	Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.15, 0.0, 0.0);

	mypos.dir = glob_nextdir;
    switch(mypos.dir){
        case north:
            mypos.y++;
            break;
        case east:
            mypos.x++;
            break;
        case south:
            mypos.y--;
            break;
        case west:
            mypos.x--;
            break;
        case center:
        	break;
    }

    switch(direction){
        case front:
        	wall_controll.is_controll = true;
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
  			search_straight_update_maze(HALF_SECTION,4.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
            break;
        case right:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
        	search_straight_update_maze (HALF_SECTION,4.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
            break;
        case left:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
            //straight(HALF_SECTION,4.0,0.3,0.3);
        	search_straight_update_maze (HALF_SECTION,4.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
        	break;
        case rear:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(180.0), 20.0*PI, 2.0*PI, turn_left);
            //straight(HALF_SECTION,4.0,0.3,0.3);
        	search_straight_update_maze (HALF_SECTION,4.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
        	break;
    }

   // int segment_accel_cnt = 0;
    //int prev_run_direction = rear;
    while(i_am_goal(mypos.x,mypos.y,gx,gy,goal_size) == false){
    	t_position prev_pos = mypos;
    	if(is_unknown(mypos.x, mypos.y)){
    		set_wall(mypos.x,mypos.y);
    	}
        direction = get_tmp_nextdir(gx,gy,goal_size,MASK_SEARCH,mypos,&glob_nextdir,NEAR_PRIORITY);

        mypos.dir = glob_nextdir;

        switch(mypos.dir){
            case north:
                mypos.y++;
                break;
            case east:
                mypos.x++;
                break;
            case south:
                mypos.y--;
                break;
            case west:
                mypos.x--;
                break;
            case center:
            	break;
        }

        uint8_t next_step_flag = 0x00;
        t_direction tmp_next_dir = glob_nextdir;

        //predict next_next_pos
        int next_move_known_next = get_tmp_nextdir(gx,gy,goal_size,MASK_SEARCH,mypos,&tmp_next_dir,NEAR_PRIORITY);
        int tmp_next_move		 = rear;
        t_position next_position = mypos;
        next_position.dir = tmp_next_dir;
        if(is_unknown(mypos.x, mypos.y) == false)
        {
        	next_step_flag = 0x80;
        	if(i_am_goal(next_position.x,next_position.y,gx,gy,goal_size) == false){
                switch(next_position.dir){
                    case north:
                    	next_position.y++;
                        break;
                    case east:
                    	next_position.x++;
                        break;
                    case south:
                    	next_position.y--;
                        break;
                    case west:
                    	next_position.x--;
                        break;
                    case center:
                    	break;
                }
                tmp_next_move = get_tmp_nextdir(gx,gy,goal_size,MASK_SEARCH,next_position,&tmp_next_dir,NEAR_PRIORITY);
        	}
        }


        switch(direction|next_step_flag){
            case front:
 				Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
	  			wall_controll.is_controll = true;
	  			search_straight_update_maze (SECTION,5.0,target.velo,0.3,gx,gy,goal_size,MASK_SEARCH);
            	break;
            case (front|0x80):
 				Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
	  			wall_controll.is_controll = true;
	  			if(next_move_known_next == front && (tmp_next_move == front && is_unknown(next_position.x, next_position.y) == false)) search_straight_update_maze (SECTION,5.0,0.6,0.6,gx,gy,goal_size,MASK_SEARCH);
	  			else							 							search_straight_update_maze (SECTION,5.0,target.velo,0.3,gx,gy,goal_size,MASK_SEARCH);
            	break;
            case right:
            case (right|0x80):
 				wall_controll.is_controll = true;
            	if(90.0 - (sen_fr.distance + sen_fl.distance)/2.0 > slalom_R90_table.Lstart )
            	{
        			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            		//straight(HALF_SECTION,4.0,0.3,0.0);
          			search_straight_update_maze (HALF_SECTION - slalom_R90_table.Lstart ,5.0,0.3,0.0,gx,gy,goal_size,MASK_SEARCH);
          			set_stop_wall(200);
          			Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
            		straight(HALF_SECTION,4.0,0.3,0.3);
            	}
            	else{
            		//search_turn90_table(&param_R90_search);
        			search_turn90_table_update_maze (&param_R90_search, gx, gy, goal_size, MASK_SEARCH);
            		Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            	}
            	break;
            case left:
            case (left|0x80):

            	wall_controll.is_controll = true;
            	if(90.0 - (sen_fr.distance + sen_fl.distance)/2.0 > slalom_L90_table.Lstart )
            	{
        			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            		//straight(HALF_SECTION,4.0,0.3,0.0);
          			search_straight_update_maze (HALF_SECTION - slalom_L90_table.Lstart,5.0,0.3,0.0,gx,gy,goal_size,MASK_SEARCH);
          			set_stop_wall(200);
          			Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
            		straight(HALF_SECTION,4.0,0.3,0.3);
            	}
            	else{
            		//search_turn90_table(&param_L90_search);
        			search_turn90_table_update_maze (&param_L90_search, gx, gy, goal_size, MASK_SEARCH);
            		Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            	}
            	break;
            case rear:
            case (rear|0x80):
            	wall_controll.is_controll = false;
    			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
      			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
                straight(HALF_SECTION,4.0,0.3,0.0);
                if(is_wall_dir(prev_pos.x, prev_pos.y, prev_pos.dir) == true) set_stop_wall(200);
                if(is_wall_dir(prev_pos.x, prev_pos.y, (prev_pos.dir + 1 + 4)%4) == true)
                {
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
                	set_stop_wall(200);
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
                }
                else if(is_wall_dir(prev_pos.x, prev_pos.y, (prev_pos.dir - 1 + 4)%4) == true)
                {
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
                	set_stop_wall(200);
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
                }
                else
                {
                	Spin_turn(DEG2RAD(180.0), 20.0*PI, 2.0*PI, turn_left);
                }
            	wall_controll.is_controll = false;
            	filter_init();
                //straight(HALF_SECTION,4.0,0.3,0.3);
            	search_straight_update_maze (HALF_SECTION,5.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
            	break;
            //case (front|0x80):
            	//segment_accel_cnt++;
            	//break;
        }

    }
	if(is_unknown(mypos.x, mypos.y)){
		set_wall(mypos.x,mypos.y);//壁情報セット
	}
    wall_controll.is_controll = false;
    straight(HALF_SECTION,4.0,0.3,0.0);
    run_mode = NON_CON_MODE;
}


void search_adachi4(int *gx,int *gy,int goal_size,
		            const t_straight_param * base_straight_param,const t_straight_param * accel_straight_param,
					const t_param * turn_l_param				,const t_param * turn_r_param					)
{

    t_direction glob_nextdir;
    int direction = get_tmp_nextdir(gx,gy,goal_size,MASK_SEARCH,mypos,&glob_nextdir,NEAR_PRIORITY);
	Set_Omega_PID_Gain(base_straight_param->om_gain->Kp, base_straight_param->om_gain->Ki, base_straight_param->om_gain->Kd);
	Set_Velo_PID_Gain(base_straight_param->sp_gain->Kp, base_straight_param->sp_gain->Ki, base_straight_param->sp_gain->Kd);
	Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.15, 0.0, 0.0);

	mypos.dir = glob_nextdir;
    switch(mypos.dir){
        case north:
            mypos.y++;
            break;
        case east:
            mypos.x++;
            break;
        case south:
            mypos.y--;
            break;
        case west:
            mypos.x--;
            break;
        case center:
        	break;
    }

    switch(direction){
        case front:
        	wall_controll.is_controll = true;
			Set_Omega_PID_Gain(base_straight_param->om_gain->Kp, base_straight_param->om_gain->Ki, base_straight_param->om_gain->Kd);
  			Set_Velo_PID_Gain(base_straight_param->sp_gain->Kp, base_straight_param->sp_gain->Ki, base_straight_param->sp_gain->Kd);
  			search_straight_update_maze(HALF_SECTION,base_straight_param->param->acc,base_straight_param->param->max_velo,base_straight_param->param->max_velo,gx,gy,goal_size,MASK_SEARCH);
            break;
        case right:
			Set_Omega_PID_Gain(base_straight_param->om_gain->Kp, base_straight_param->om_gain->Ki, base_straight_param->om_gain->Kd);
  			Set_Velo_PID_Gain(base_straight_param->sp_gain->Kp, base_straight_param->sp_gain->Ki, base_straight_param->sp_gain->Kd);
        	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
        	search_straight_update_maze (HALF_SECTION,base_straight_param->param->acc,base_straight_param->param->max_velo,base_straight_param->param->max_velo,gx,gy,goal_size,MASK_SEARCH);
            break;
        case left:
			Set_Omega_PID_Gain(base_straight_param->om_gain->Kp, base_straight_param->om_gain->Ki, base_straight_param->om_gain->Kd);
  			Set_Velo_PID_Gain(base_straight_param->sp_gain->Kp, base_straight_param->sp_gain->Ki, base_straight_param->sp_gain->Kd);
        	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
            //straight(HALF_SECTION,4.0,0.3,0.3);
        	search_straight_update_maze (HALF_SECTION,base_straight_param->param->acc,base_straight_param->param->max_velo,base_straight_param->param->max_velo,gx,gy,goal_size,MASK_SEARCH);
        	break;
        case rear:
			Set_Omega_PID_Gain(base_straight_param->om_gain->Kp, base_straight_param->om_gain->Ki, base_straight_param->om_gain->Kd);
  			Set_Velo_PID_Gain(base_straight_param->sp_gain->Kp, base_straight_param->sp_gain->Ki, base_straight_param->sp_gain->Kd);
        	Spin_turn(DEG2RAD(180.0), 20.0*PI, 2.0*PI, turn_left);
            //straight(HALF_SECTION,4.0,0.3,0.3);
        	search_straight_update_maze (HALF_SECTION,base_straight_param->param->acc,base_straight_param->param->max_velo,base_straight_param->param->max_velo,gx,gy,goal_size,MASK_SEARCH);
        	break;
    }

   // int segment_accel_cnt = 0;
    //int prev_run_direction = rear;
    while(i_am_goal(mypos.x,mypos.y,gx,gy,goal_size) == false){
    	t_position prev_pos = mypos;
    	if(is_unknown(mypos.x, mypos.y)){
    		set_wall(mypos.x,mypos.y);
    	}
        direction = get_tmp_nextdir(gx,gy,goal_size,MASK_SEARCH,mypos,&glob_nextdir,NEAR_PRIORITY);

        mypos.dir = glob_nextdir;

        switch(mypos.dir){
            case north:
                mypos.y++;
                break;
            case east:
                mypos.x++;
                break;
            case south:
                mypos.y--;
                break;
            case west:
                mypos.x--;
                break;
            case center:
            	break;
        }

        uint8_t next_step_flag = 0x00;
        t_direction tmp_next_dir = glob_nextdir;

        //predict next_next_pos
        int next_move_known_next = get_tmp_nextdir(gx,gy,goal_size,MASK_SEARCH,mypos,&tmp_next_dir,NEAR_PRIORITY);
        int tmp_next_move		 = rear;
        t_position next_position = mypos;
        next_position.dir = tmp_next_dir;
        if(is_unknown(mypos.x, mypos.y) == false)
        {
        	next_step_flag = 0x80;
        	if(i_am_goal(next_position.x,next_position.y,gx,gy,goal_size) == false){
                switch(next_position.dir){
                    case north:
                    	next_position.y++;
                        break;
                    case east:
                    	next_position.x++;
                        break;
                    case south:
                    	next_position.y--;
                        break;
                    case west:
                    	next_position.x--;
                        break;
                    case center:
                    	break;
                }
                tmp_next_move = get_tmp_nextdir(gx,gy,goal_size,MASK_SEARCH,next_position,&tmp_next_dir,NEAR_PRIORITY);
        	}
        }


        switch(direction|next_step_flag){
            case front:
 				Set_Omega_PID_Gain(base_straight_param->om_gain->Kp, base_straight_param->om_gain->Ki, base_straight_param->om_gain->Kd);
	  			Set_Velo_PID_Gain(base_straight_param->sp_gain->Kp, base_straight_param->sp_gain->Ki, base_straight_param->sp_gain->Kd);
	  			wall_controll.is_controll = true;
	  			search_straight_update_maze (SECTION,base_straight_param->param->acc,target.velo,base_straight_param->param->max_velo,gx,gy,goal_size,MASK_SEARCH);
            	break;
            case (front|0x80):
 		 		Set_Omega_PID_Gain(base_straight_param->om_gain->Kp, base_straight_param->om_gain->Ki, base_straight_param->om_gain->Kd);
 			  	Set_Velo_PID_Gain(base_straight_param->sp_gain->Kp, base_straight_param->sp_gain->Ki, base_straight_param->sp_gain->Kd);
	  			wall_controll.is_controll = true;
	  			if(next_move_known_next == front && (tmp_next_move == front && is_unknown(next_position.x, next_position.y) == false))
	  			{
	 				Set_Omega_PID_Gain(accel_straight_param->om_gain->Kp, accel_straight_param->om_gain->Ki, accel_straight_param->om_gain->Kd);
		  			Set_Velo_PID_Gain(accel_straight_param->sp_gain->Kp, accel_straight_param->sp_gain->Ki, accel_straight_param->sp_gain->Kd);
	  				search_straight_update_maze (SECTION,accel_straight_param->param->acc,accel_straight_param->param->max_velo,accel_straight_param->param->max_velo,gx,gy,goal_size,MASK_SEARCH);
	  			}
	  			else
	  			{
	  				search_straight_update_maze (SECTION,accel_straight_param->param->acc,target.velo,base_straight_param->param->max_velo,gx,gy,goal_size,MASK_SEARCH);
	  			}
            	break;
            case right:
            case (right|0x80):
 				wall_controll.is_controll = true;
            	if(90.0 - (sen_fr.distance + sen_fl.distance)/2.0 > slalom_R90_table.Lstart || ((sen_fr.distance + sen_fl.distance)/2.0 < 110.0 && ABS(sen_fr.distance - sen_fl.distance) > 5.0))
            	{
        			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            		//straight(HALF_SECTION,4.0,0.3,0.0);
          			search_straight_update_maze(HALF_SECTION - (90.0 - (sen_fr.distance + sen_fl.distance)/2.0) ,5.0,0.3,0.0,gx,gy,goal_size,MASK_SEARCH);
          			set_stop_wall(100);
          			Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_right);
            		straight(HALF_SECTION,4.0,0.3,0.3);
            	}
            	else if(sen_l.is_wall == true && ABS(sen_l.distance - 45.0) > 5.0)
            	{
     				Set_Omega_PID_Gain(base_straight_param->om_gain->Kp, base_straight_param->om_gain->Ki, base_straight_param->om_gain->Kd);
    	  			Set_Velo_PID_Gain(base_straight_param->sp_gain->Kp, base_straight_param->sp_gain->Ki, base_straight_param->sp_gain->Kd);
                	search_straight_update_maze (HALF_SECTION,base_straight_param->param->acc,base_straight_param->param->max_velo,0.0,gx,gy,goal_size,MASK_SEARCH);
                	Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_left);
                	set_stop_wall(100);
                	Spin_turn(DEG2RAD(180.0), 50.0*PI, 4.0*PI, turn_right);
                	straight(HALF_SECTION,4.0,0.3,0.3);
            	}
            	else{
            		//search_turn90_table(&param_R90_search);
        			search_turn90_table_update_maze (turn_r_param, gx, gy, goal_size, MASK_SEARCH);
     				Set_Omega_PID_Gain(base_straight_param->om_gain->Kp, base_straight_param->om_gain->Ki, base_straight_param->om_gain->Kd);
    	  			Set_Velo_PID_Gain(base_straight_param->sp_gain->Kp, base_straight_param->sp_gain->Ki, base_straight_param->sp_gain->Kd);
            	}
            	break;
            case left:
            case (left|0x80):

            	wall_controll.is_controll = true;
            	if(90.0 - (sen_fr.distance + sen_fl.distance)/2.0 > slalom_R90_table.Lstart || ((sen_fr.distance + sen_fl.distance)/2.0 < 110.0 && ABS(sen_fr.distance - sen_fl.distance) > 5.0))
            	{
     				Set_Omega_PID_Gain(base_straight_param->om_gain->Kp, base_straight_param->om_gain->Ki, base_straight_param->om_gain->Kd);
    	  			Set_Velo_PID_Gain(base_straight_param->sp_gain->Kp, base_straight_param->sp_gain->Ki, base_straight_param->sp_gain->Kd);
            		//straight(HALF_SECTION,4.0,0.3,0.0);
          			search_straight_update_maze (HALF_SECTION - (90.0 - (sen_fr.distance + sen_fl.distance)/2.0),base_straight_param->param->acc,base_straight_param->param->max_velo,0.0,gx,gy,goal_size,MASK_SEARCH);
          			set_stop_wall(100);
          			Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_left);
          			straight(HALF_SECTION,base_straight_param->param->acc,base_straight_param->param->max_velo,base_straight_param->param->max_velo);
            	}
            	else if(sen_r.is_wall == true && ABS(sen_r.distance - 45.0) > 10.0)
            	{
     				Set_Omega_PID_Gain(base_straight_param->om_gain->Kp, base_straight_param->om_gain->Ki, base_straight_param->om_gain->Kd);
    	  			Set_Velo_PID_Gain(base_straight_param->sp_gain->Kp, base_straight_param->sp_gain->Ki, base_straight_param->sp_gain->Kd);
                	search_straight_update_maze (HALF_SECTION,base_straight_param->param->acc,base_straight_param->param->max_velo,0.0,gx,gy,goal_size,MASK_SEARCH);
                	Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_right);
                	set_stop_wall(100);
                	Spin_turn(DEG2RAD(180.0), 50.0*PI, 4.0*PI, turn_left);
                	straight(HALF_SECTION,4.0,0.3,0.3);
            	}
            	else{
            		//search_turn90_table(&param_L90_search);
        			search_turn90_table_update_maze (turn_l_param, gx, gy, goal_size, MASK_SEARCH);
     				Set_Omega_PID_Gain(base_straight_param->om_gain->Kp, base_straight_param->om_gain->Ki, base_straight_param->om_gain->Kd);
    	  			Set_Velo_PID_Gain(base_straight_param->sp_gain->Kp, base_straight_param->sp_gain->Ki, base_straight_param->sp_gain->Kd);
            	}
            	break;
            case rear:
            case (rear|0x80):
            	wall_controll.is_controll = false;
				Set_Omega_PID_Gain(base_straight_param->om_gain->Kp, base_straight_param->om_gain->Ki, base_straight_param->om_gain->Kd);
				Set_Velo_PID_Gain(base_straight_param->sp_gain->Kp, base_straight_param->sp_gain->Ki, base_straight_param->sp_gain->Kd);
                straight(HALF_SECTION,base_straight_param->param->acc,base_straight_param->param->max_velo,0.0);
                if(is_wall_dir(prev_pos.x, prev_pos.y, prev_pos.dir) == true) set_stop_wall(200);
                if(is_wall_dir(prev_pos.x, prev_pos.y, (prev_pos.dir + 1 + 4)%4) == true)
                {
                	Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_right);
                	set_stop_wall(100);
                	Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_right);
                }
                else if(is_wall_dir(prev_pos.x, prev_pos.y, (prev_pos.dir - 1 + 4)%4) == true)
                {
                	Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_left);
                	set_stop_wall(100);
                	Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_left);
                }
                else
                {
                	Spin_turn(DEG2RAD(180.0), 50.0*PI, 4.0*PI, turn_left);
                }
            	wall_controll.is_controll = false;
            	filter_init();
                //straight(HALF_SECTION,4.0,0.3,0.3);
            	search_straight_update_maze (HALF_SECTION,base_straight_param->param->acc,base_straight_param->param->max_velo,base_straight_param->param->max_velo,gx,gy,goal_size,MASK_SEARCH);
            	break;
            //case (front|0x80):
            	//segment_accel_cnt++;
            	//break;
        }

    }
	if(is_unknown(mypos.x, mypos.y)){
		set_wall(mypos.x,mypos.y);//壁情報セット
	}
    wall_controll.is_controll = false;
    straight(HALF_SECTION,base_straight_param->param->acc,base_straight_param->param->max_velo,0.0);
    run_mode = NON_CON_MODE;


}

void search_adachi_zenmen(int *gx,int *gy,int goal_size){
    t_direction glob_nextdir;
    int direction = get_nextdir_zenmen(gx,gy,goal_size,MASK_SEARCH,&glob_nextdir);
	Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
	Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.15, 0.0, 0.0);

	mypos.dir = glob_nextdir;
    switch(mypos.dir){
        case north:
            mypos.y++;
            break;
        case east:
            mypos.x++;
            break;
        case south:
            mypos.y--;
            break;
        case west:
            mypos.x--;
            break;
        case center:
        	break;
    }

    switch(direction){
        case front:
        	wall_controll.is_controll = true;
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
  			search_straight(HALF_SECTION,4.0,0.3,0.3);
            break;
        case right:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
            straight(HALF_SECTION,4.0,0.3,0.3);
            break;
        case left:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
            straight(HALF_SECTION,4.0,0.3,0.3);
            break;
        case rear:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(180.0), 20.0*PI, 2.0*PI, turn_left);
            straight(HALF_SECTION,4.0,0.3,0.3);
		    break;
    }


    while(i_am_goal(mypos.x,mypos.y,gx,gy,goal_size) == false){
    	t_position prev_pos = mypos;
    	set_wall(mypos.x,mypos.y);
        direction = get_nextdir_zenmen(gx,gy,goal_size,MASK_SEARCH,&glob_nextdir);

        mypos.dir = glob_nextdir;

        switch(mypos.dir){
            case north:
                mypos.y++;
                break;
            case east:
                mypos.x++;
                break;
            case south:
                mypos.y--;
                break;
            case west:
                mypos.x--;
                break;
            case center:
            	break;
        }

        switch(direction){
            case front:
 				Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
	  			wall_controll.is_controll = true;
	  			search_straight(SECTION,4.0,0.3,0.3);
            	break;
            case right:
 				wall_controll.is_controll = true;
            	if(90.0 - (sen_fr.distance + sen_fl.distance)/2.0 > slalom_R90_table.Lstart )
            	{
        			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            		straight(HALF_SECTION,4.0,0.3,0.0);
            		Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
            		straight(HALF_SECTION,4.0,0.3,0.3);
            	}
            	else{
            		search_turn90_table(&param_R90_search);
        			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            	}
            	break;
            case left:
            	wall_controll.is_controll = true;
            	if(90.0 - (sen_fr.distance + sen_fl.distance)/2.0 > slalom_L90_table.Lstart )
            	{
        			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            		straight(HALF_SECTION,4.0,0.3,0.0);
            		Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
            		straight(HALF_SECTION,4.0,0.3,0.3);
            	}
            	else{
            		search_turn90_table(&param_L90_search);
        			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            	}
            	break;
            case rear:
            	wall_controll.is_controll = false;
    			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
      			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
                straight(HALF_SECTION,4.0,0.3,0.0);
                if(is_wall_dir(prev_pos.x, prev_pos.y, prev_pos.dir) == true) set_stop_wall(200);
                if(is_wall_dir(prev_pos.x, prev_pos.y, (prev_pos.dir + 1 + 4)%4) == true)
                {
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
                	set_stop_wall(100);
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
                }
                else if(is_wall_dir(prev_pos.x, prev_pos.y, (prev_pos.dir - 1 + 4)%4) == true)
                {
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
                	set_stop_wall(100);
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
                }
                else
                {
                	Spin_turn(DEG2RAD(180.0), 20.0*PI, 2.0*PI, turn_left);
                }
            	wall_controll.is_controll = false;
            	filter_init();
                straight(HALF_SECTION,4.0,0.3,0.3);
                break;
        }

    }
    set_wall(mypos.x,mypos.y);		//壁をセット
    wall_controll.is_controll = false;
    straight(HALF_SECTION,4.0,0.3,0.0);
    run_mode = NON_CON_MODE;
}

void search_adachi_zenmen2(int *gx,int *gy,int goal_size){
    t_direction glob_nextdir;
    int direction = get_nextdir_zenmen(gx,gy,goal_size,MASK_SEARCH,&glob_nextdir);
	Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
	Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.15, 0.0, 0.0);

	mypos.dir = glob_nextdir;
    switch(mypos.dir){
        case north:
            mypos.y++;
            break;
        case east:
            mypos.x++;
            break;
        case south:
            mypos.y--;
            break;
        case west:
            mypos.x--;
            break;
        case center:
        	break;
    }

    switch(direction){
        case front:
        	wall_controll.is_controll = true;
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
  			search_straight_update_maze_zenmen(HALF_SECTION,4.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
            break;
        case right:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
        	search_straight_update_maze_zenmen(HALF_SECTION,4.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
            break;
        case left:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
            //straight(HALF_SECTION,4.0,0.3,0.3);
        	search_straight_update_maze_zenmen(HALF_SECTION,4.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
        	break;
        case rear:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(180.0), 20.0*PI, 2.0*PI, turn_left);
            //straight(HALF_SECTION,4.0,0.3,0.3);
        	search_straight_update_maze_zenmen(HALF_SECTION,4.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
        	break;
    }

    int segment_accel_cnt = 0;
    while(i_am_goal(mypos.x,mypos.y,gx,gy,goal_size) == false){
    	t_position prev_pos = mypos;
    	if(is_unknown(mypos.x, mypos.y)){
    		set_wall(mypos.x,mypos.y);
    	}
        direction = get_nextdir_zenmen(gx,gy,goal_size,MASK_SEARCH,&glob_nextdir);

        mypos.dir = glob_nextdir;

        switch(mypos.dir){
            case north:
                mypos.y++;
                break;
            case east:
                mypos.x++;
                break;
            case south:
                mypos.y--;
                break;
            case west:
                mypos.x--;
                break;
            case center:
            	break;
        }

        uint8_t next_step_flag = 0x00;
        if(is_unknown(mypos.x, mypos.y) == false)
        {
        	next_step_flag = 0x80;
        }


        switch(direction|next_step_flag){
            case front:
            case (front|0x80):
 				Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
	  			wall_controll.is_controll = true;
	  			if(segment_accel_cnt > 1)
	  			{
	  				//FAN_Motor_SetDuty(300);
	  				Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	  				Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
	  				wall_controll.is_controll = true;
	  				for(int i = 0; i < segment_accel_cnt - 1;i++)
	  				{
	  					search_straight_update_maze_zenmen(SECTION,5.0,0.6,0.6,gx,gy,goal_size,MASK_SEARCH);
	  				}
	  				FAN_Motor_SetDuty(0);
	  				//search_straight(SECTION,5.0,0.6,0.3);
	  				search_straight_update_maze_zenmen(SECTION,5.0,0.6,0.3,gx,gy,goal_size,MASK_SEARCH);
	  				segment_accel_cnt = 0;
	  			}
	  			else if(segment_accel_cnt == 1)
	  			{
	  				Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	  				Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
	  				wall_controll.is_controll = true;
	  				//search_straight(SECTION,4.0,0.3,0.3);
	  				search_straight_update_maze_zenmen(SECTION,5.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
	  				segment_accel_cnt = 0;
	  			}
	  			//search_straight(SECTION,4.0,0.3,0.3);
	  			search_straight_update_maze_zenmen(SECTION,5.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
            	break;
            case right:
            case (right|0x80):
				if(segment_accel_cnt > 1)
				{
					//FAN_Motor_SetDuty(300);
					Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
					Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
					wall_controll.is_controll = true;
					for(int i = 0; i < segment_accel_cnt - 1;i++)
					{
						//search_straight(SECTION,5.0,0.6,0.6);
						search_straight_update_maze_zenmen(SECTION,5.0,0.6,0.6,gx,gy,goal_size,MASK_SEARCH);
					}
					//search_straight(SECTION,5.0,0.6,0.3);
					search_straight_update_maze_zenmen(SECTION,5.0,0.6,0.3,gx,gy,goal_size,MASK_SEARCH);
					segment_accel_cnt = 0;
					FAN_Motor_SetDuty(0);
				}
				else if(segment_accel_cnt == 1)
				{
					Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
					Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
					wall_controll.is_controll = true;
					//search_straight(SECTION,4.0,0.3,0.3);
					search_straight_update_maze_zenmen(SECTION,5.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
					segment_accel_cnt = 0;
				}
 				wall_controll.is_controll = true;
            	if(90.0 - (sen_fr.distance + sen_fl.distance)/2.0 > slalom_R90_table.Lstart )
            	{
        			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            		//straight(HALF_SECTION,4.0,0.3,0.0);
          			search_straight_update_maze_zenmen(HALF_SECTION,5.0,0.3,0.0,gx,gy,goal_size,MASK_SEARCH);
          			Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
            		straight(HALF_SECTION,4.0,0.3,0.3);
            	}
            	else{
            		//search_turn90_table(&param_R90_search);
        			search_turn90_table_update_maze_zenmen(&param_R90_search, gx, gy, goal_size, MASK_SEARCH);
            		Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            	}
            	break;
            case left:
            case (left|0x80):
				if(segment_accel_cnt > 1)
				{
					//FAN_Motor_SetDuty(300);
					Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
					Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
					wall_controll.is_controll = true;
					for(int i = 0; i < segment_accel_cnt - 1;i++)
					{
						//search_straight(SECTION,5.0,0.6,0.6);
						search_straight_update_maze_zenmen(SECTION,5.0,0.6,0.6,gx,gy,goal_size,MASK_SEARCH);
					}
					//search_straight(SECTION,5.0,0.6,0.3);
					search_straight_update_maze_zenmen(SECTION,5.0,0.6,0.3,gx,gy,goal_size,MASK_SEARCH);
					segment_accel_cnt = 0;
					FAN_Motor_SetDuty(0);
				}
				else if(segment_accel_cnt == 1)
				{
					Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
					Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
					wall_controll.is_controll = true;
					//search_straight(SECTION,4.0,0.3,0.3);
					search_straight_update_maze_zenmen(SECTION,5.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
					segment_accel_cnt = 0;
				}
            	wall_controll.is_controll = true;
            	if(90.0 - (sen_fr.distance + sen_fl.distance)/2.0 > slalom_L90_table.Lstart )
            	{
        			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            		//straight(HALF_SECTION,4.0,0.3,0.0);
          			search_straight_update_maze_zenmen(HALF_SECTION,5.0,0.3,0.0,gx,gy,goal_size,MASK_SEARCH);
            		Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
            		straight(HALF_SECTION,4.0,0.3,0.3);
            	}
            	else{
            		//search_turn90_table(&param_L90_search);
        			search_turn90_table_update_maze_zenmen(&param_L90_search, gx, gy, goal_size, MASK_SEARCH);
            		Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            	}
            	break;
            case rear:
            case (rear|0x80):
				if(segment_accel_cnt > 1)
				{
					//FAN_Motor_SetDuty(300);
					Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
					Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
					wall_controll.is_controll = true;
					for(int i = 0; i < segment_accel_cnt - 1;i++)
					{
						search_straight(SECTION,5.0,0.6,0.6);
					}
					search_straight(SECTION,5.0,0.6,0.3);
					segment_accel_cnt = 0;
					FAN_Motor_SetDuty(0);
				}
				else if(segment_accel_cnt == 1)
				{
					Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
					Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
					wall_controll.is_controll = true;
					search_straight(SECTION,4.0,0.3,0.3);
					segment_accel_cnt = 0;
				}
            	wall_controll.is_controll = false;
    			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
      			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
                straight(HALF_SECTION,4.0,0.3,0.0);
                if(is_wall_dir(prev_pos.x, prev_pos.y, prev_pos.dir) == true) set_stop_wall(200);
                if(is_wall_dir(prev_pos.x, prev_pos.y, (prev_pos.dir + 1 + 4)%4) == true)
                {
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
                	set_stop_wall(200);
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
                }
                else if(is_wall_dir(prev_pos.x, prev_pos.y, (prev_pos.dir - 1 + 4)%4) == true)
                {
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
                	set_stop_wall(200);
                	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
                }
                else
                {
                	Spin_turn(DEG2RAD(180.0), 20.0*PI, 2.0*PI, turn_left);
                }
            	wall_controll.is_controll = false;
            	filter_init();
                //straight(HALF_SECTION,4.0,0.3,0.3);
            	search_straight_update_maze_zenmen(HALF_SECTION,5.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
            	break;
            //case (front|0x80):
            	//segment_accel_cnt++;
            	//break;
        }

    }
	if(segment_accel_cnt > 1)
	{
		//FAN_Motor_SetDuty(300);
		Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
		Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
		wall_controll.is_controll = true;
		for(int i = 0; i < segment_accel_cnt - 1;i++)
		{
			search_straight(SECTION,5.0,0.6,0.6);
		}
		search_straight(SECTION,5.0,0.6,0.3);
		segment_accel_cnt = 0;
		FAN_Motor_SetDuty(0);
	}
	else if(segment_accel_cnt == 1)
	{
		Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
		Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
		wall_controll.is_controll = true;
		search_straight(SECTION,4.0,0.3,0.3);
		segment_accel_cnt = 0;
	}
	if(is_unknown(mypos.x, mypos.y)){
		set_wall(mypos.x,mypos.y);//壁情報セット
	}
    wall_controll.is_controll = false;
    straight(HALF_SECTION,4.0,0.3,0.0);
    run_mode = NON_CON_MODE;
}

void search_adachi_zenmen3(int *gx,int *gy,int goal_size){
    t_direction glob_nextdir;
    int direction = get_tmp_nextdir(gx,gy,goal_size,MASK_SEARCH,mypos,&glob_nextdir,FAR_PRIORITY);
	Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
	Set_PID_Gain(&(wall_controll.side_om_wall_gain), 0.15, 0.0, 0.0);

	mypos.dir = glob_nextdir;
    switch(mypos.dir){
        case north:
            mypos.y++;
            break;
        case east:
            mypos.x++;
            break;
        case south:
            mypos.y--;
            break;
        case west:
            mypos.x--;
            break;
        case center:
        	break;
    }

    switch(direction){
        case front:
        	wall_controll.is_controll = true;
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
  			search_straight_update_maze_zenmen(HALF_SECTION,4.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
            break;
        case right:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_right);
        	search_straight_update_maze_zenmen(HALF_SECTION,4.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
            break;
        case left:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(90.0), 20.0*PI, 2.0*PI, turn_left);
            //straight(HALF_SECTION,4.0,0.3,0.3);
        	search_straight_update_maze_zenmen(HALF_SECTION,4.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
        	break;
        case rear:
			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
        	Spin_turn(DEG2RAD(180.0), 20.0*PI, 2.0*PI, turn_left);
            //straight(HALF_SECTION,4.0,0.3,0.3);
        	search_straight_update_maze_zenmen(HALF_SECTION,4.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
        	break;
    }

    //int segment_accel_cnt = 0;
    //int prev_run_direction = rear;
    while(i_am_goal(mypos.x,mypos.y,gx,gy,goal_size) == false){
    	t_position prev_pos = mypos;
    	if(is_unknown(mypos.x, mypos.y)){
    		set_wall(mypos.x,mypos.y);
    	}
        direction = get_tmp_nextdir(gx,gy,goal_size,MASK_SEARCH,mypos,&glob_nextdir,FAR_PRIORITY);

        mypos.dir = glob_nextdir;

        switch(mypos.dir){
            case north:
                mypos.y++;
                break;
            case east:
                mypos.x++;
                break;
            case south:
                mypos.y--;
                break;
            case west:
                mypos.x--;
                break;
            case center:
            	break;
        }

        uint8_t next_step_flag = 0x00;
        t_direction tmp_next_dir = glob_nextdir;

        //predict next_next_pos
        //int next_move_known_next = get_nextdir_zenmen(gx,gy,goal_size,MASK_SEARCH,&tmp_next_dir);
        int next_move_known_next = get_tmp_nextdir(gx,gy,goal_size,MASK_SEARCH,mypos,&tmp_next_dir,FAR_PRIORITY);
        int tmp_next_move		 = rear;
        t_position next_position = mypos;
        next_position.dir = tmp_next_dir;
        if(is_unknown(mypos.x, mypos.y) == false)
        {
        	next_step_flag = 0x80;
        	if(i_am_goal(next_position.x,next_position.y,gx,gy,goal_size) == false){
                switch(next_position.dir){
                    case north:
                    	next_position.y++;
                        break;
                    case east:
                    	next_position.x++;
                        break;
                    case south:
                    	next_position.y--;
                        break;
                    case west:
                    	next_position.x--;
                        break;
                    case center:
                    	break;
                }
                tmp_next_move = get_tmp_nextdir(gx,gy,goal_size,MASK_SEARCH,next_position,&tmp_next_dir,FAR_PRIORITY);
        	}
        }


        switch(direction|next_step_flag){
            case front:
 				Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
	  			wall_controll.is_controll = true;
            	if(sen_r.is_wall == true && ABS(sen_r.distance - 45.0) > 10.0)
            	{
        			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
          			search_straight_update_maze_zenmen(HALF_SECTION,5.0,0.3,0.0,gx,gy,goal_size,MASK_SEARCH);
          			Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_right);
                	set_stop_wall(100);
                	Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_left);
                	straight(HALF_SECTION,4.0,0.3,0.3);
            	}
            	else if(sen_r.is_wall == true && ABS(sen_r.distance - 45.0) > 10.0)
            	{
        			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
          			search_straight_update_maze_zenmen(HALF_SECTION,5.0,0.3,0.0,gx,gy,goal_size,MASK_SEARCH);
          			Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_right);
                	set_stop_wall(100);
                	Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_left);
                	straight(HALF_SECTION,4.0,0.3,0.3);
            	}
            	else
            	{
            		search_straight_update_maze_zenmen(SECTION,5.0,target.velo,0.3,gx,gy,goal_size,MASK_SEARCH);
            	}
	  			break;
            case (front|0x80):
 				Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
	  			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
	  			wall_controll.is_controll = true;
	  			if(next_move_known_next == front && (tmp_next_move == front && is_unknown(next_position.x, next_position.y) == false)) search_straight_update_maze_zenmen(SECTION,5.0,0.6,0.6,gx,gy,goal_size,MASK_SEARCH);
	  			else							 							search_straight_update_maze_zenmen(SECTION,5.0,target.velo,0.3,gx,gy,goal_size,MASK_SEARCH);
            	break;
            case right:
            case (right|0x80):
 				wall_controll.is_controll = true;
            	if(90.0 - (sen_fr.distance + sen_fl.distance)/2.0 > slalom_R90_table.Lstart || ((sen_fr.distance + sen_fl.distance)/2.0 < 110.0 && ABS(sen_fr.distance - sen_fl.distance) > 5.0))
            	{
        			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            		//straight(HALF_SECTION,4.0,0.3,0.0);
          			search_straight_update_maze_zenmen(HALF_SECTION - (90.0 - (sen_fr.distance + sen_fl.distance)/2.0) ,5.0,0.3,0.0,gx,gy,goal_size,MASK_SEARCH);
          			set_stop_wall(100);
          			Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_right);
            		straight(HALF_SECTION,4.0,0.3,0.3);
            	}
            	else if(sen_l.is_wall == true && ABS(sen_l.distance - 45.0) > 10.0)
            	{
        			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
          			search_straight_update_maze_zenmen(HALF_SECTION,5.0,0.3,0.0,gx,gy,goal_size,MASK_SEARCH);
                	Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_left);
                	set_stop_wall(100);
                	Spin_turn(DEG2RAD(180.0), 50.0*PI, 4.0*PI, turn_right);
                	straight(HALF_SECTION,4.0,0.3,0.3);
            	}
            	else{
            		//search_turn90_table(&param_R90_search);
        			search_turn90_table_update_maze_zenmen(&param_R90_search, gx, gy, goal_size, MASK_SEARCH);
            		Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            	}
            	break;
            case left:
            case (left|0x80):

            	wall_controll.is_controll = true;
            	if(90.0 - (sen_fr.distance + sen_fl.distance)/2.0 > slalom_L90_table.Lstart  || ((sen_fr.distance + sen_fl.distance)/2.0 < 110.0 && ABS(sen_fr.distance - sen_fl.distance) > 5.0))
            	{
        			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            		//straight(HALF_SECTION,4.0,0.3,0.0);
          			search_straight_update_maze_zenmen(HALF_SECTION - (90.0 - (sen_fr.distance + sen_fl.distance)/2.0),5.0,0.3,0.0,gx,gy,goal_size,MASK_SEARCH);
          			set_stop_wall(100);
          			Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_left);
            		straight(HALF_SECTION,4.0,0.3,0.3);
            	}
            	else if(sen_r.is_wall == true && ABS(sen_r.distance - 45.0) > 10.0)
            	{
        			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
          			search_straight_update_maze_zenmen(HALF_SECTION,5.0,0.3,0.0,gx,gy,goal_size,MASK_SEARCH);
          			Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_right);
                	set_stop_wall(100);
                	Spin_turn(DEG2RAD(180.0), 50.0*PI, 4.0*PI, turn_left);
                	straight(HALF_SECTION,4.0,0.3,0.3);
            	}
            	else{
            		//search_turn90_table(&param_L90_search);
        			search_turn90_table_update_maze_zenmen(&param_L90_search, gx, gy, goal_size, MASK_SEARCH);
            		Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
          			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
            	}
            	break;
            case rear:
            case (rear|0x80):
            	wall_controll.is_controll = false;
    			Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
      			Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
                straight(HALF_SECTION,4.0,0.3,0.0);
                if(is_wall_dir(prev_pos.x, prev_pos.y, prev_pos.dir) == true) set_stop_wall(200);
                if(is_wall_dir(prev_pos.x, prev_pos.y, (prev_pos.dir + 1 + 4)%4) == true)
                {
                	Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_right);
                	set_stop_wall(100);
                	Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_right);
                }
                else if(is_wall_dir(prev_pos.x, prev_pos.y, (prev_pos.dir - 1 + 4)%4) == true)
                {
                	Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_left);
                	set_stop_wall(100);
                	Spin_turn(DEG2RAD(90.0), 50.0*PI, 4.0*PI, turn_left);
                }
                else
                {
                	Spin_turn(DEG2RAD(180.0), 50.0*PI, 4.0*PI, turn_left);
                }
            	wall_controll.is_controll = false;
            	filter_init();
                //straight(HALF_SECTION,4.0,0.3,0.3);
            	search_straight_update_maze_zenmen(HALF_SECTION,5.0,0.3,0.3,gx,gy,goal_size,MASK_SEARCH);
            	break;
            //case (front|0x80):
            	//segment_accel_cnt++;
            	//break;
        }

    }
    /*
	if(segment_accel_cnt > 1)
	{
		//FAN_Motor_SetDuty(300);
		Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
		Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
		wall_controll.is_controll = true;
		for(int i = 0; i < segment_accel_cnt - 1;i++)
		{
			search_straight(SECTION,5.0,0.6,0.6);
		}
		search_straight(SECTION,5.0,0.6,0.3);
		segment_accel_cnt = 0;
		FAN_Motor_SetDuty(0);
	}
	else if(segment_accel_cnt == 1)
	{
		Set_Omega_PID_Gain(om_gain_search_straight.Kp, om_gain_search_straight.Ki, om_gain_search_straight.Kd);
		Set_Velo_PID_Gain(sp_gain_search_straight.Kp, sp_gain_search_straight.Ki, sp_gain_search_straight.Kd);
		wall_controll.is_controll = true;
		search_straight(SECTION,4.0,0.3,0.3);
		segment_accel_cnt = 0;
	}*/
	if(is_unknown(mypos.x, mypos.y)){
		set_wall(mypos.x,mypos.y);//壁情報セット
	}
    wall_controll.is_controll = false;
    straight(HALF_SECTION,4.0,0.3,0.0);
    run_mode = NON_CON_MODE;
}

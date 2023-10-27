/*
 * queue.h
 *
 *  Created on: 2022/08/28
 *      Author: sato1
 */


#ifndef MODULE_INC_QUEUE_H_
#define MODULE_INC_QUEUE_H_

#include "typedef.h"
#include "index.h"


void list_init(t_queue *queue);
t_MapNode pop(t_queue *queue);
void push(t_queue *queue,t_MapNode input);
void swap(t_MapNode* a,t_MapNode* b);
void min_heapify(t_queue *queue,int i);
void build_heap(t_queue *queue);
t_MapNode heap_pop(t_queue *queue);
void heap_push(t_queue *queue,t_MapNode element);
t_MapNode node_set(int16_t st_x,int16_t st_y,int16_t cost,int16_t cost_h);



#endif /* MODULE_INC_QUEUE_H_ */

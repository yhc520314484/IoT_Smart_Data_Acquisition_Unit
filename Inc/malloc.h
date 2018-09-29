#ifndef __MALLOC_H
#define __MALLOC_H

#include "main.h"
#include "stm32l4xx_hal.h"


 
#ifndef NULL
#define NULL 0
#endif

//定义1个内存池
#define SRAMIN	 0		//内部内存池

#define SRAMBANK 	1	//定义支持的SRAM块数.	


//mem1内存参数设定.mem1完全处于内部SRAM里面.
#define MEM1_BLOCK_SIZE			64  	  						//内存块大小为64字节
#define MEM1_MAX_SIZE			32*1024  					   	//最大管理内存 32K  大概能装100组传感器参数
#define MEM1_ALLOC_TABLE_SIZE	MEM1_MAX_SIZE/MEM1_BLOCK_SIZE 	//内存表大小
		 

//内存管理控制器
struct _m_mallco_dev
{
	void (*init)(uint8_t);					//初始化
	uint16_t (*perused)(uint8_t);		  	    	//内存使用率
	uint8_t 	*membase[SRAMBANK];				//内存池 管理SRAMBANK个区域的内存
	uint32_t *memmap[SRAMBANK]; 				//内存管理状态表
	uint8_t  memrdy[SRAMBANK]; 				//内存管理是否就绪
};
extern struct _m_mallco_dev mallco_dev;	 //在mallco.c里面定义

void mymemset(void *s,uint8_t c,uint32_t count);	//设置内存
void mymemcpy(void *des,void *src,uint32_t n);//复制内存     
void my_mem_init(uint8_t memx);				//内存管理初始化函数(外/内部调用)
uint32_t my_mem_malloc(uint8_t memx,uint32_t size);	//内存分配(内部调用)
uint8_t my_mem_free(uint8_t memx,uint32_t offset);		//内存释放(内部调用)
uint16_t my_mem_perused(uint8_t memx) ;			//获得内存使用率(外/内部调用) 
////////////////////////////////////////////////////////////////////////////////
//用户调用函数
void myfree(void *ptr);  			//内存释放(外部调用)
void *mymalloc(uint32_t size);			//内存分配(外部调用)
void *myrealloc(void *ptr,uint32_t size);//重新分配内存(外部调用)
#endif














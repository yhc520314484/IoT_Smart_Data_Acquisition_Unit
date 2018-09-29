#ifndef __LINKED_LIST_H__
#define __LINKED_LIST_H__

#include "main.h"
#include "stm32l4xx_hal.h"
#include "local_protocol.h"
#include "EEPROM_24C256.h"

#ifndef TAIL 
#define TAIL 0xffff
#endif


typedef struct node{
	reg_data_sensor_parm_and_ID   * sensor_parm_and_ID;
	sensor_data_entity_for_update * sensor_data_package_point; 

	struct node *pNext; //指向下一个节点的指针
}Linked_List;


Linked_List *CreateNode(void);
Linked_List *GetNode(Linked_List *pHeader, uint16_t num);
uint8_t InsertNode(Linked_List * pHeader, uint16_t num);
uint8_t DeleNode(Linked_List * pHeader, uint16_t num);
uint16_t GetNodeNum(Linked_List * pHeader);

//struct node
//{
//	int data; //有效数据
//	struct node *pNext; //指向下一个节点的指针
//};

//struct node *CreateNode(void);
//struct node *GetNode(struct node *pHeader, uint16_t num);
//uint8_t InsertNode(struct node * pHeader, uint16_t num);
//uint8_t DeleNode(struct node * pHeader, uint16_t num);
//uint16_t GetNodeNum(struct node * pHeader);


#endif

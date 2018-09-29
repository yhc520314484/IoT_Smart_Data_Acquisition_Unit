#include "linked_list.h"
#include "malloc.h"

#ifndef TURE
#define TURE 1
#endif


#ifndef FAULSE 
#define FAULSE 0
#endif




//创建链表节点
Linked_List *CreateNode(void)
{ 
    Linked_List *p = (Linked_List *)mymalloc(sizeof(Linked_List));
    if (NULL == p)
    {
        return NULL;
    }
    memset(p, 0, sizeof(Linked_List));
    p->pNext = NULL;
    return p;
}

//获取链表节点地址
Linked_List *GetNode(Linked_List *pHeader, uint16_t num)
{
    uint8_t i;
    Linked_List *p = pHeader;
    if (NULL == p->pNext)
    {
        return FAULSE;
    }
    for (i = 0; i < num; i++)
    {
        if (p->pNext != NULL) //节点的位置不得大于链表长度
				{
					p=p->pNext;
				}
				else{
					return FAULSE;
				}
		}
		return p;
}

//插入节点
//形参 pHeader:链表头 ?num = 0:头 TAIL:尾部 !0&&!TAIL : 链表中间
//返回值 TURE:成功 FAULSE:失败
uint8_t InsertNode(Linked_List * pHeader, uint16_t num)
{
		uint8_t i;
		Linked_List *p = pHeader;
		Linked_List *p1 = (Linked_List *)mymalloc(sizeof(Linked_List));
		if (NULL == p1)
		{
				return FAULSE;
		}
		if (0 == num) //在头部增加节点
		{
			if(NULL == pHeader)
			{
				pHeader = p1;
			}
			else{
				pHeader->pNext = p1;
			}
		}
		else if (TAIL == num)//末尾增加节点
		{
			while(p->pNext != NULL)
			{
				p=p->pNext;
			}
			p->pNext = p1;
		}
		else if(num > 0 && num != TAIL)//在链表中间增加节点
		{
			for(i=0;i<num-1;i++)
			{
				if(NULL != p->pNext)//增加节点的位置不得大于链表长度
				{
					p=p->pNext;
				}
				else
				{
					return FAULSE;
				}
			}
			p1->pNext = p->pNext;
			p->pNext = p1;
		}
		return true;
}

//删除链表节点
//形参 pHeader:链表头 ?num = 0:头 TAIL:尾部 !0&&!TAIL : 链表中间，注意num不能为0
//返回值 TURE:成功 FAULSE:失败
uint8_t DeleNode(Linked_List * pHeader, uint16_t num)
{
		uint8_t i;
		Linked_List *p = pHeader, *p1, *p2;
		if (NULL == p) //空链表，无意义
			{
				return FAULSE;
			}
		 if(0 == num)//在头部删除节点
		 {
				p1 = pHeader;
				pHeader = pHeader->pNext;
				myfree(p1);
		 }
		 if (TAIL == num)//末尾删除节点
		 {
				while(p->pNext != NULL)
				{
					p1=p;//获取新的尾巴
					p=p->pNext;//指向下一个节点
				}
			p1->pNext = NULL;
			myfree(p);
		}
		else if(num > 0 && num != TAIL)//在链表中间删除节点
		{
			for(i=0;i<num-1;i++)
			{
				if(NULL != p->pNext)//增加节点的位置不得大于链表长度
				{
					p=p->pNext;
				}
				else{
					return FAULSE;
				}
			}
			p1 = p->pNext;
			p2 = p1->pNext;
			p->pNext = p2;
			myfree(p1);
		}
		return TURE;
}

//获取链表长度
uint16_t GetNodeNum(Linked_List * pHeader)
{
		uint16_t ret, i;
		Linked_List *p = pHeader;
		if (NULL == p) //空链表
		{
			return FAULSE;
		}
		for(i=1;p->pNext != NULL;i++)
		{
			p = p->pNext;
		}
		ret = i;
		return ret;
}

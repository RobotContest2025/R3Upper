#ifndef __LIST_H__
#define __LIST_H__

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

typedef struct Load_t Load_t;

struct Load_t
{
    Load_t* next;
    uint8_t data[];
};

typedef struct List_t
{
    int element_size;
    int length;
    Load_t* data;
}MyList_t;

typedef struct
{
    MyList_t* list;
    Load_t* current;
}ListIterator_t;


typedef uint32_t(*ListIsMatch_Cb_t)(void* user,void* dst);

//链表
MyList_t* ListCreate(int element_size);
int ListRemove(MyList_t *list);
int ListAddElement(MyList_t *list, void* data);
int ListDeleteElement(MyList_t *list, int index);
void* ListFind(MyList_t *list,void* user,ListIsMatch_Cb_t match_cb);
int ListGetIndex(MyList_t *list,void* src,ListIsMatch_Cb_t match_cb);
void* ListGetDataByIndex(MyList_t *list,int index);

//迭代器
void InitListIterator(ListIterator_t *iterater,MyList_t *list);
void ResetListIterator(ListIterator_t *iterater);
void* IteraterGet(ListIterator_t *iterater);
void IteraterNext(ListIterator_t *iterater);

#endif

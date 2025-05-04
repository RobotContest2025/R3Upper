#include "list.h"

MyList_t* ListCreate(int element_size)
{
    MyList_t* list=(MyList_t*)malloc(sizeof(MyList_t));
    list->data=NULL;
    list->length=0;
    list->element_size=element_size;
    return list;
}

int ListRemove(MyList_t *list)
{
    return 0;
}

int ListAddElement(MyList_t *list,void* data)
{
    list->length++;
    Load_t* load=(Load_t*)malloc(list->element_size+sizeof(Load_t));
    load->next=NULL;
    memcpy(load->data,data,list->element_size);
    if(list->data==NULL)    //如果是第一个元素，直接使用即可，否则遍历到最后一个元素，在其后添加
        list->data=load;
    else
    {
        Load_t *buffer=list->data;
        while(buffer->next)
            buffer=buffer->next;
        buffer->next=load;
    }
    return 0;
}

int ListDeleteElement(MyList_t *list,int index)
{
    Load_t *buffer=list->data;
    if (index==0)
    {
        Load_t *temp=buffer->next;
        free(buffer);
        list->data=temp;
    }
    else
    {
        for(int i=0;i<index-1;i++)
        {
            buffer=buffer->next;
            if(buffer==NULL)
                return -1;
        }
        if(buffer->next)
        {
            Load_t *next_load=buffer->next->next;
            list->length--;
            free(buffer->next);
            buffer->next=next_load;
        }
        else
            return -1;
    }
    return 0;
}

void* ListFind(MyList_t *list,void* user,ListIsMatch_Cb_t match_cb)
{
    Load_t *buffer=list->data;
    for(uint32_t i=0;i<list->length;i++)
    {
        if(match_cb(user,buffer->data))
            return buffer->data;
        buffer=buffer->next;
    }
    return NULL;
}

int ListGetIndex(MyList_t *list,void* src,ListIsMatch_Cb_t match_cb)
{
    Load_t *buffer=list->data;
    for(uint32_t i=0;i<list->length;i++)
    {
        if(match_cb(src,buffer->data))
            return i;
        buffer=buffer->next;
    }
    return -1;
}

void* ListGetDataByIndex(MyList_t *list,int index)
{
    Load_t* temp=list->data;
    for(int i=0;i<index;i++)
        temp=temp->next;
    return temp->data;
}


void InitListIterator(ListIterator_t *iterater,MyList_t *list)
{
    iterater->list=list;
    iterater->current=list->data;
}

void ResetListIterator(ListIterator_t *iterater)
{
    iterater->current=iterater->list->data;
}

void* IteraterGet(ListIterator_t *iterater)
{
    if(iterater->current)
        return iterater->current->data;
    return NULL;
}

void IteraterNext(ListIterator_t *iterater)
{
    if(iterater->current)
        iterater->current=iterater->current->next;
}
#include "WatchDog2.h"

static MyList_t *watchDogList = NULL;
static TaskHandle_t watchDogTask = NULL;
static uint32_t watchdog_id = 0;


static uint32_t match(void *user, void *dst)
{
    if ((uint32_t)user == ((WatchDog2_t *)dst)->id)
        return 1;
    return 0;
}

static void WatchDogTask(void *param)
{
    ListIterator_t it;
    WatchDog2_t *temp;
    uint32_t current_time;
    InitListIterator(&it, watchDogList);
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        current_time = HAL_GetTick();
        ResetListIterator(&it);
        while ((temp = (WatchDog2_t *)IteraterGet(&it))!=NULL)
        {
            if ((current_time - temp->last_wake_up_time_ms > temp->timeout_ms) && ((temp->state & 0x06) == 0x06)) // 第3位置1，启用看门狗，第2位置于1，解除挂起
            {
                temp->watchdog_cb(temp->user_data);
                if (temp->state & 0x01)
                    temp->state = temp->state & (~0x02); // 第二位置0挂起看门狗，下次检查时不再执行该回调函数
            }
            IteraterNext(&it);
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000 / WATCHDOG_FREQUENCY));
    }
}

uint32_t AddWatchDog(WatchDogCb_t watchdog_cb, uint32_t timeout_ms, void *user_data, uint32_t mode)
{
    WatchDog2_t watch_dog;
    if (!watchDogTask)
    {
        watchDogList = ListCreate(sizeof(WatchDog2_t));
        xTaskCreate(WatchDogTask, "WatchDogTask", 128, NULL, 6, &watchDogTask);
    }
    watch_dog.id = watchdog_id;
    watch_dog.timeout_ms = timeout_ms;
    watch_dog.user_data = user_data;
    watch_dog.watchdog_cb = watchdog_cb;
    watch_dog.last_wake_up_time_ms = HAL_GetTick();
    watch_dog.state = mode;
    ListAddElement(watchDogList, &watch_dog);
    watchdog_id = watchdog_id + 1;
    return watchdog_id - 1;
}

uint32_t FeedDog(uint32_t id)
{
    WatchDog2_t *temp = ListFind(watchDogList, (void *)id, match);
    if (temp)
    {
        temp->last_wake_up_time_ms = HAL_GetTick();
        if (temp->state & 0x01) // 已经喂狗，解除挂起
            temp->state = temp->state | 0x02;
        return 1;
    }
    return 0;
}

uint32_t DisableDog(uint32_t id)
{
    WatchDog2_t *temp = ListFind(watchDogList, (void *)id, match);
    if (temp)
    {
        temp->state = temp->state & (~0x04);
        return 1;
    }
    return 0;
}

uint32_t EnableDog(uint32_t id)
{
    WatchDog2_t *temp = ListFind(watchDogList, (void *)id, match);
    if (temp)
    {
        temp->state = temp->state | 0x04;
        return 1;
    }
    return 0;
}

uint32_t DeleteDog(uint32_t id)
{
    UNUSED(id);
    //TOOD:删除链表元素(未实现，因为几乎没有使用的需求)
    return 0;
}
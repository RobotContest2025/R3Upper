#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

#include "stm32f4xx_hal.h"

#include "CANDrive.h"
#include "motorEx.h"
#include "PID.h"
#include "slope.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "cmsis_os.h"

#include "Action.h"

void ActionDealTask(void* param);
void CoplitActionTask(void* queue);
void MotorControl(void* param);


#endif

#ifndef _STP_23L_H_
#define _STP_23L_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/*距离 噪音 反射强度 可信度 积分次数 温度*/
typedef struct{
  uint16_t distance;			//距离
  uint16_t noise;					//噪音
	uint32_t strength;			//反射强度
	uint8_t possibility;		//可信度
	uint32_t intergrial;		//积分次数
	uint16_t temp;					//温度
}STP_23L_Data;
uint8_t CheckSum(uint8_t *Buf, uint8_t Len);
uint8_t STP_23L_DataProcess(uint8_t *buffer,STP_23L_Data *para);

#ifdef __cplusplus
}
#endif

#endif

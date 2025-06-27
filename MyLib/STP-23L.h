#ifndef _STP_23L_H_
#define _STP_23L_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/*���� ���� ����ǿ�� ���Ŷ� ���ִ��� �¶�*/
typedef struct{
  uint16_t distance;			//����
  uint16_t noise;					//����
	uint32_t strength;			//����ǿ��
	uint8_t possibility;		//���Ŷ�
	uint32_t intergrial;		//���ִ���
	uint16_t temp;					//�¶�
}STP_23L_Data;
uint8_t CheckSum(uint8_t *Buf, uint8_t Len);
uint8_t STP_23L_DataProcess(uint8_t *buffer,STP_23L_Data *para);

#ifdef __cplusplus
}
#endif

#endif

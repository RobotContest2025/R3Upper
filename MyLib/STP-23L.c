#include "STP-23L.h"

uint8_t STP_23L_DataProcess(uint8_t *buffer,STP_23L_Data *para)
{
	for(int i = 0;i < 3;i++)
	{
		if(buffer[i] != 0xAA)
			return 0;
	}
	
	for(uint8_t j = 0;j < 12 * 15;j += 15){
		//距离 
	  para->distance = ((uint16_t)buffer[11+j] << 8)|(uint16_t)buffer[10+j];
//	  噪音 
		para->noise =((uint16_t)buffer[13+j] << 8)|(uint16_t)buffer[12+j];
//		反射强度 
//	  para->strength = ((uint16_t)buffer[15+j] << 8)|(uint16_t)buffer[14+j];
		para->strength = ((((uint16_t)buffer[17+j] << 8)|(uint16_t)buffer[16+j])<<16)|(((uint16_t)buffer[15+j] << 8)|(uint16_t)buffer[14+j]);
//		可信度 
//	  para->possibility = ((uint16_t)buffer[17+j] << 8)|(uint16_t)buffer[16+j];
		para->possibility =(uint16_t)buffer[18+j];
//		积分次数 
//	  para->intergrial = ((uint16_t)buffer[19+j] << 8)|(uint16_t)buffer[18+j];
		para->intergrial = ((((uint16_t)buffer[22+j] << 8)|(uint16_t)buffer[21+j])<<16)|(((uint16_t)buffer[20+j] << 8)|(uint16_t)buffer[19+j]);
//	  温度
//		para->temp = ((uint16_t)buffer[21+j] << 8)|(uint16_t)buffer[20+j];
		para->temp = ((uint16_t)buffer[24+j] << 8)|(uint16_t)buffer[23+j];
	}
	
	if(CheckSum(buffer,194) == buffer[194])
		return 0;
	
	return 1;
}

uint8_t CheckSum(uint8_t *Buf, uint8_t Len)
{
  uint8_t i = 0;
  uint8_t sum = 0;
  uint8_t checksum = 0;
 
  for(i=0; i<Len; i++)
  {
    sum += *Buf++;
  }
 
  checksum = sum % 0xff;
 
  return checksum;
}

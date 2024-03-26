#ifndef _DRV_CAN_
#define _DRV_CAN_

#include "function.h"
#include "linux_list.h"


#include "drv_can.h"


void CAN_Filter_Init(CAN_HandleTypeDef *hcan);
void CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t len);
void CAN_ReceiveData(CAN_HandleTypeDef *hcan, uint8_t fifo, uint8_t *data);


extern uint16_t Communication_Onlinecnt;


#endif /*_DRV_CAN_*/

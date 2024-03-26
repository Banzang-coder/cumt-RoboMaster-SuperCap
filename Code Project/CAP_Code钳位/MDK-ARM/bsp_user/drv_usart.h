#ifndef _DRV_USART_
#define _DRV_USART_
/*原始库*/
#include "usart.h"
#include "stdio.h"


#define USART_MAX_LEN    10               //定义最大接收字节数

extern uint8_t data_to_send_V6[20];
extern uint8_t RxBuffer[USART_MAX_LEN];

/*-------------------------------串口2---------------------------------*/
void UART2_IDLE_CALLBACK(void);
void UART2_Open(uint8_t* Data,uint16_t Size);
void UploadData_vofa(void);
void ANO_V6_Send_Up_Computer(UART_HandleTypeDef* UART_X,int32_t user1,int16_t user2,int16_t user3,int16_t user4,int16_t user5,int16_t user6);


#endif /*_DRV_USART_*/

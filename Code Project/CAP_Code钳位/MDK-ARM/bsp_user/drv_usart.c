#include "drv_usart.h"

/*不是所有的都能用，可以使用正常收发与VOFA*/


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	

/*-------------------------------地址--------------------------------*/
uint8_t data_to_send_V6[20];
uint8_t RxBuffer[USART_MAX_LEN];    







/*------------------------重定向printf，scanf------------------------*/
/*VOFA-FireWater*/
int fputc(int ch,FILE *f)
{
    uint32_t temp = ch;
	  while((USART2->ISR & 0X40) == 0);
    HAL_UART_Transmit(&huart2,(uint8_t *)&temp,1,1000);
	
//	HAL_UART_Transmit_DMA(&huart4,(uint8_t *)&temp,34);
    return ch;
}

int fgetc(FILE * f)
{
  uint8_t ch;
  HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/*--------------------------接收完成回调函数--------------------------*/

/*-weak-*/void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)	//串口中断回调函数
{
	if(huart->Instance == USART2)
	{
			
	}
}

/*-weak-*/void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)	//串口中断回调函数
{
	if(huart->Instance == USART2)
	{
		
	}
}

void UART2_Open(uint8_t* Data,uint16_t Size)
{
	HAL_UART_Receive_DMA(&huart2,Data,Size);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);   		//<开启中断前分配回调比较好
} 

void UART2_IDLE_CALLBACK(void)
{
	if((__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET))
	{  
		HAL_UART_DMAStop(&huart2);
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);

		
	 HAL_UART_Receive_DMA(&huart2, RxBuffer, sizeof(RxBuffer));
	}
}


/*-------------------------------匿名上位机---------------------------------*/

void ANO_V6_Send_Up_Computer(UART_HandleTypeDef* UART_X,int32_t user1,int16_t user2,int16_t user3,int16_t user4,int16_t user5,int16_t user6)
{
	uint8_t _cnt=0;	
//	data_to_send_V6[_cnt++]=0xAA;
//	data_to_send_V6[_cnt++]=0xFF;
//	data_to_send_V6[_cnt++]=0xF1;
//	data_to_send_V6[_cnt++]=12;
	
	///*   协议为V6版本 注意观察区别   */
	data_to_send_V6[_cnt++]=0xAA;
	data_to_send_V6[_cnt++]=0x05; //或许00也可以 05是匿名拓空者飞控
	data_to_send_V6[_cnt++]=0xAF; //AF是上位机识别码
	data_to_send_V6[_cnt++]=0xF1; 
	data_to_send_V6[_cnt++]=0;
	
	data_to_send_V6[_cnt++]=BYTE2(user1);
	data_to_send_V6[_cnt++]=BYTE3(user1);

	data_to_send_V6[_cnt++]=BYTE0(user3);
	data_to_send_V6[_cnt++]=BYTE1(user3);
	
	data_to_send_V6[_cnt++]=BYTE0(user4);
	data_to_send_V6[_cnt++]=BYTE1(user4);

	data_to_send_V6[_cnt++]=BYTE0(user5);
	data_to_send_V6[_cnt++]=BYTE1(user5);
	
	data_to_send_V6[_cnt++]=BYTE0(user6);
	data_to_send_V6[_cnt++]=BYTE1(user6);
	
	uint8_t sc=0;
	uint8_t ac=0;
	
	for(uint8_t i=0;i<(data_to_send_V6[3]+4);i++)
	{
		sc += data_to_send_V6[i];
		ac += sc;	
	}
		data_to_send_V6[_cnt++]=sc;
	  data_to_send_V6[_cnt++]=ac;
	
	HAL_UART_Transmit_DMA(UART_X,data_to_send_V6,_cnt);
}

/*-----------------------------VOFA+Justfloat-------------------------------*/
float temp[3];
static uint8_t tempData[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0x80,0x7F};
void UploadData_vofa(void)
{
//	static float temp[3];//float temp[15];
//	static uint16_t time_count;
	
//	temp[0]=Parameter.Current.INCharge;
//	temp[1]=Parameter.Voltage.Battery;
//	temp[2]=Parameter.Voltage.Capacitor;
//	memcpy(tempData,(uint8_t *)&temp,sizeof(temp));
	
	HAL_UART_Transmit_DMA(&huart2,(uint8_t *)tempData,16);
}


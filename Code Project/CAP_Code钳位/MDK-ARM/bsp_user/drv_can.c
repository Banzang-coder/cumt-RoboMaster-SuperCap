#include "drv_can.h"
/*
 ** ===================================================================
 **     Funtion Name : void CAN_Filter_Init(CAN_HandleTypeDef *hcan)
 **     Description :  can过滤器配置
 **     Parameters  :无
 **                 none
 **     Returns     :无
 **                 none
 ** ===================================================================
 */	
void CAN_Filter_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // 使能fifo0接收到新信息中断
	
  HAL_CAN_Start(hcan);                   //< 使能can
}
/*
 ** ===================================================================
 **     Funtion Name : void CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t len)
 **     Description :  can发送函数
 **     Parameters  :无
 **                 none
 **     Returns     :无
 **                 none
 ** ===================================================================
 */	
void CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t len)
{
  CAN_TxHeaderTypeDef txHeader;
  uint32_t txMailbox;
	uint8_t txData[8]={1,0,1,0,1,0,2,0};

  txHeader.StdId = id;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.IDE = CAN_ID_STD;
  txHeader.DLC = len;
	txHeader.TransmitGlobalTime = DISABLE;
	
	uint16_t temp1 = 0;
	uint16_t temp2 = 0;
	uint16_t temp3 = 0;
	
	temp1 = cap.V * 100;
	txData[0] = temp1>>8;
	txData[1] = 0xff&temp1;
	txData[2] = temp2>>8;
	txData[3] = 0xff&temp2;
	txData[4] = temp3>>8;
	txData[5] = 0xff&temp3;
	txData[6] = FLAGS.ChargePNFlag;
	txData[7] = 0xff;
	
  if (HAL_CAN_AddTxMessage(hcan, &txHeader, txData, &txMailbox) != HAL_OK)
  {
    /* Transmission request Error */
//    Error_Handler();
  }
	
		/* 检查分配给发送消息的邮箱编号 */
	if (txMailbox == CAN_TX_MAILBOX0)
	{
		/* 发送消息被存储在邮箱0中 */
	}
	else if (txMailbox == CAN_TX_MAILBOX1)
	{
		/* 发送消息被存储在邮箱1中 */
	}
	else if (txMailbox == CAN_TX_MAILBOX2)
	{
		/* 发送消息被存储在邮箱2中 */
	}

}

/*
 ** ===================================================================
 **     Funtion Name :  HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
 **     Description :  重写HAL Fifo0接收中断回调函数
                      -weak-
 **     Parameters  :无
 **                 none
 **     Returns     :无
 **                 none
 ** ===================================================================
 */	
uint8_t rxdata[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_ReceiveData(hcan, CAN_RX_FIFO0,rxdata);
}

/*
 ** ===================================================================
 **     Funtion Name : void CAN_ReceiveData(CAN_HandleTypeDef *hcan, uint8_t fifo, uint8_t *data)
 **     Description :  can接收数据处理函数
 **     Parameters  :无
 **                 none
 **     Returns     :无
 **                 none
 ** ===================================================================
 */	
uint16_t Communication_Onlinecnt = 0;
uint8_t Rxrealbuffer[8] = {0};
void CAN_ReceiveData(CAN_HandleTypeDef *hcan, uint8_t fifo, uint8_t *data)
{
  CAN_RxHeaderTypeDef rxHeader;
	if (HAL_CAN_GetRxMessage(hcan, fifo, &rxHeader, data) != HAL_ERROR)
  {
			if(rxHeader.StdId == 0x6ff)
			{
				for(uint8_t i=0 ; i < 8 ; i++ )
				{
					Rxrealbuffer[i] = rxdata[i];
				}
				Communication_Onlinecnt = 0;
				
				FLAGS.ChargePNFlag = Rxrealbuffer[0];
				CtrValue.Poref = Rxrealbuffer[1]-8;
				CtrValue.BufferPower = Rxrealbuffer[2];
				CtrValue.JudgeChassisPower = (float)Rxrealbuffer[3];
				
			}
  }
}



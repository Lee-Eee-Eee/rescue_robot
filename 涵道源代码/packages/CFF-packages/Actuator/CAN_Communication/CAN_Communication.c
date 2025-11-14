/* 包含头文件 ----------------------------------------------------------------*/
#include "CAN_Communication.h"

/* 私有类型定义 --------------------------------------------------------------*/
typedef union
{
    uint32_t WORD;
    struct
    {
        uint32_t SourceList     :4;
        uint32_t SourceAddr     :8;
        uint32_t ComanndNumber  :4;
        uint32_t TargetList     :4;
        uint32_t TargetAddr     :8;
        uint32_t Page           :1;
        uint32_t retain         :3;
    }bits;
}CanIDType;
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
static rt_device_t can_dev = NULL;
static uint16_t Tim_GetCurrentTime = 0;

/* 扩展变量 ------------------------------------------------------------------*/
uint8_t canrxtxerr = 0;
/* 私有函数原形 --------------------------------------------------------------*/
/*****************************************************************************************
*  <summary>
*  get current time;
*  <returns> current time tick
******************************************************************************************/
static uint8_t Tim_CheckTimeOut(uint16_t TimeStart, uint16_t TimeNow, uint16_t TimeOut)
{
  #define ActivateCounter ((uint16_t)(TimeOut+TimeStart))
  if (ActivateCounter >= TimeStart)
  {
    if ((TimeNow >= ActivateCounter) ||
        (TimeNow < TimeStart))
    {
      return TRUE;
    }
  }
  else if ((TimeNow >= ActivateCounter) &&
           (TimeNow < TimeStart))
  {
    return TRUE;
  }
  return FALSE;
}
/** @brief  发送一帧数据
  * @param  
  * @retval 
  */
static void SendUsedPG(PgnCanTxDataType *pQ)
{
    int8_t err;
    struct rt_can_msg msg = {0};
    
    msg.id = pQ->Source;
    msg.ide = RT_CAN_EXTID;
    msg.len = pQ->TxDataLength;
    if(pQ->Datum == 0)
	{
		msg.rtr = RT_CAN_RTR; 
	}
	else
	{
		msg.rtr = RT_CAN_DTR; 
	}
    memcpy(msg.data, pQ->Datum, pQ->TxDataLength);
    
    err = rt_device_write(can_dev, 0, &msg, sizeof(msg));
    
	if(0 == err)  //发送成功
	{
        if(MODE_REQ == pQ->mode)
        {
            pQ->PGState = PG_TX_TxING;
        }
        else if((MODE_BRO == pQ->mode)||(MODE_ACK == pQ->mode))
        {
            pQ->PGState = PG_TX_SUC;
        }          
	}
}
static void CANCommunication_InformationParsing(struct rt_can_msg *pQ)
{
	uint8_t counts;
    uint8_t i;
	uint8_t dlcbuff;
	
	for (counts=TX_PGN_USER; counts<NR_OF_TX_PG; counts++)
	{
		if (TxPGArray[counts].Target == pQ->id)
		{
			dlcbuff = 0; 
			if(TxPGArray[counts].RxDatum != 0)
			{
				dlcbuff = (TxPGArray[counts].RxDataLength == pQ->len);
			}
			else if(TxPGArray[counts].RxDataLength != 0)
			{
				if(RT_CAN_DTR != pQ->rtr)//为遥控帧
				{
					dlcbuff = 1;
				}
			}
			if(1 == dlcbuff)
			{
				if(TxPGArray[counts].RxDatum != 0)
				{
					for(i=0; i<pQ->len; i++)
					{
						((uint8_t*)TxPGArray[counts].RxDatum)[i] = pQ->data[i];
					}
				}
				GetReferenceToDevice(pQ->id);
				if((TxPGArray[counts].Datum != 0)||(TxPGArray[counts].TxDataLength != 0))
				{
					switch(TxPGArray[counts].mode)
					{
						case MODE_BRO: //广播模式
							TxPGArray[counts].Timer = Tim_GetCurrentTime-TxPGArray[counts].TransRate;
							break;
						case MODE_REQ: //请求模式
							if(PG_TX_TxING == TxPGArray[counts].PGState)
							{
								TxPGArray[counts].PGState = PG_TX_SUC;
							}
							break;
						case MODE_ACK: //答应模式
							TxPGArray[counts].Enable = TRUE;
							TxPGArray[counts].PGState = PG_TX_REQ;
							TxPGArray[counts].Timer = Tim_GetCurrentTime-TxPGArray[counts].TransRate;
							break;
					}
				}
			}
		}
	}
}
/* 函数体 --------------------------------------------------------------------*/
void CAN_TT_ASYNC_Enable(uint8_t list) //异步模式使能
{
	if(TxPGArray[list].TransType == TT_ASYNC)
	{
		TxPGArray[list].PGState = PG_TX_REQ;
	}
}
void CAN_TT_EnableSet(uint8_t list,uint8_t enable) //使能失能
{
	if(enable == FALSE)
	{
		TxPGArray[list].Enable = FALSE;
	}
	else
	{
		TxPGArray[list].Enable = TRUE;
	}
}
uint8_t CAN_TT_PGStateRead(uint8_t list) //读取CAN总线中列表序号对应帧的通讯状态
{
    return TxPGArray[list].PGState;
}
uint8_t CAN_TT_ErrRead(uint8_t list) //读取CAN总线中列表序号对应帧的错误信息
{
    if(TxPGArray[list].LogBootFail_cnt >= 10)
    {
        return 0x81;
    }
    else if(TxPGArray[list].can_err >= 5)
    {
        return 0x82;
    }
	if(TxPGArray[list].LogBootFail_cnt > 0)
    {
        return 0x01;
    }
    else if(TxPGArray[list].can_err > 0)
    {
        return 0x02;
    }
    else 
    {
        return 0;
    }
}

void CANCommunication_Scan(void)
{
    uint8_t counts;
	static uint16_t time = 0;
	static uint8_t timeflg = 0;
	
    struct rt_can_msg rxmsg = {0};
    /**************************************************************************************
	***************************************************************************************
	**************************************接收部分*****************************************
	***************************************************************************************
	**************************************************************************************/
    rxmsg.hdr_index = -1;
    if(rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg)))
	{
        CANCommunication_InformationParsing(&rxmsg);
    }
    /**************************************************************************************
	***************************************************************************************
	**************************************发送部分*****************************************
	***************************************************************************************
	**************************************************************************************/
    if(timeflg == 0)
	{
		GetReferenceFromDevice();
		for (counts=TX_PGN_USER; counts<NR_OF_TX_PG; counts++)
		{
			if (TxPGArray[counts].Enable) 
			{
				if (TxPGArray[counts].TransType  == TT_ASYNC) //异步模式
				{
					if (TxPGArray[counts].PGState == PG_TX_REQ)
					{
						(void)SendUsedPG(&TxPGArray[counts]); 
						if((TxPGArray[counts].can_err < 5)&&(TxPGArray[counts].PGState == PG_TX_REQ))
						{
							TxPGArray[counts].can_err++;
						}
						TxPGArray[counts].Timer = Tim_GetCurrentTime; 
						break;
					}
					else if(TxPGArray[counts].PGState == PG_TX_TxING)
					{
						TxPGArray[counts].can_err = 0;
						if(Tim_CheckTimeOut(TxPGArray[counts].Timer,Tim_GetCurrentTime,TxPGArray[counts].TransRate))
						{
							TxPGArray[counts].PGState = PG_TX_REQ; 
							if(TxPGArray[counts].LogBootFail_cnt < 10)
							{
								TxPGArray[counts].LogBootFail_cnt++;
							}
						}
					}
					else if(TxPGArray[counts].PGState == PG_TX_SUC)
					{
						TxPGArray[counts].can_err = 0;
						TxPGArray[counts].LogBootFail_cnt = 0;
						TxPGArray[counts].PGState = PG_TX_FREE; 
					}
				}
				else if (TxPGArray[counts].TransType == TT_CYCLIC) //循环模式
				{
					if ((Tim_CheckTimeOut(TxPGArray[counts].Timer,Tim_GetCurrentTime,TxPGArray[counts].TransRate))
					&& (TxPGArray[counts].PGState == PG_TX_REQ))
					{
						(void)SendUsedPG(&TxPGArray[counts]);
						if((TxPGArray[counts].can_err < 5)&&(TxPGArray[counts].PGState == PG_TX_REQ))
						{
							TxPGArray[counts].can_err++;
						}
						TxPGArray[counts].Timer = Tim_GetCurrentTime; 
						break;
					}
					else if(TxPGArray[counts].PGState == PG_TX_TxING)
					{
						TxPGArray[counts].can_err = 0;
						if(Tim_CheckTimeOut(TxPGArray[counts].Timer,Tim_GetCurrentTime,TxPGArray[counts].TransRate))
						{
							TxPGArray[counts].PGState = PG_TX_REQ; 
							if(TxPGArray[counts].LogBootFail_cnt < 10)
							{
								TxPGArray[counts].LogBootFail_cnt++;
							}
						}
					}
					else if (TxPGArray[counts].PGState == PG_TX_SUC)
					{
						TxPGArray[counts].can_err = 0;
						TxPGArray[counts].LogBootFail_cnt = 0;
						TxPGArray[counts].Timer = Tim_GetCurrentTime; 
						TxPGArray[counts].PGState = PG_TX_REQ;  
					} 
				}
			}
		}
		/**************************************************************************************
		***************************************************************************************
		**************************************检错部分*****************************************
		***************************************************************************************
		**************************************************************************************/ 
		//...
		uint8_t err = 0;
		for (counts=TX_PGN_USER; counts<NR_OF_TX_PG; counts++)
		{
			if (TxPGArray[counts].can_err >= 5) 
			{
				err |= 0x02;
				TxPGArray[counts].can_err = 0;
			}
//			if(Bsp_Cancom_ReadERR() != 0)
//			{
//				err |= 0x04;
//			}
			if(err != 0)
			{
				break;
			}
		}
		if(err)
		{
//			Bsp_Cancom_ConfigurationInit();
//			Bsp_Cancom_FilterInit();
			time = Tim_GetCurrentTime;
			timeflg = 1;
		}
		canrxtxerr |= err;	
	}
	else if(Tim_CheckTimeOut(time,
		                     Tim_GetCurrentTime,
	                         50))
	{
		timeflg = 0;
	}
}
/**
  * 函 数 名: xCreate
  * 函数功能: CAN通讯数据结构创建原函数.
  * 输入参数: SourceAddr    数据来源地址
  *           SourceID      数据来源ID号
  *           TargetAddr    数据目标地址
  *           TargetID      数据目标ID号
  *           Enable        发送使能(TRUE/FALSE)
  *           PGState       发送使能(PG_TX_FREE/被动使能发送
  *                                  PG_TX_REQ/开机使能发送
  *                                  PG_TX_TxING
  *                                  PG_TX_SUC)
  *           mode          协议模式(MODE_BRO/广播模式
  *                                  MODE_REQ/请求模式
  *                                  MODE_ACK/答应模式)
  *           TransType     发送模式(TT_ASYNC/异步模式
  *                                  TT_CYCLIC/循环模式)
  *           TransRate     发送速率
  *           TxDatum       发送数据
  *           RxDatum       接收数据
  */
PgnCanTxDataType xCreate(   uint8_t SourceAddr,
                            uint8_t SourceID,
                            uint8_t TargetAddr,
                            uint8_t TargetID,
                            uint8_t TXComanndNumber,
                            uint8_t RXComanndNumber,
                            uint8_t Enable,
                            uint8_t PGState,
                            uint8_t mode,
                            uint8_t TransType,
                            uint16_t TransRate,
							uint16_t Timer,
                            void *TxDatum,
							uint8_t TxDataLength, 
                            void *RxDatum,
							uint8_t RxDataLength 							
                            )
{
    PgnCanTxDataType CanDataType;
    CanIDType PG_TX_ID;
    CanIDType PG_RX_ID;
    
    PG_TX_ID.WORD = 0;
    PG_RX_ID.WORD = 0;
    
    if((TxDatum != 0)||(TxDataLength != 0))
    {
        PG_TX_ID.bits.TargetList = TargetID;
        PG_TX_ID.bits.TargetAddr = TargetAddr;
        PG_TX_ID.bits.ComanndNumber = TXComanndNumber;
        PG_TX_ID.bits.SourceList = SourceID;
        PG_TX_ID.bits.SourceAddr = SourceAddr;
        PG_TX_ID.bits.Page = 0;
    }
    if((RxDatum != 0)||(RxDataLength != 0))
    {
        PG_RX_ID.bits.TargetList = SourceID;
        PG_RX_ID.bits.TargetAddr = SourceAddr;
        PG_RX_ID.bits.ComanndNumber = RXComanndNumber;
        PG_RX_ID.bits.SourceList = TargetID;
        PG_RX_ID.bits.SourceAddr = TargetAddr;
        PG_RX_ID.bits.Page = 0;
    }
    if(TxDataLength > 8)
	{
		TxDataLength = 8;
	}
	if(RxDataLength > 8)
	{
		RxDataLength = 8;
	}
    CanDataType.Enable = Enable;
	CanDataType.PGState = PGState;
    CanDataType.mode = mode;
	CanDataType.Timer = Timer;
	CanDataType.Source = PG_TX_ID.WORD;
    CanDataType.Target = PG_RX_ID.WORD;
	CanDataType.TransType = TransType;
	CanDataType.TransRate = TransRate;
	CanDataType.Datum = (uint8_t*)TxDatum;
	CanDataType.TxDataLength = TxDataLength;
    CanDataType.RxDatum = (uint8_t*)RxDatum;
	CanDataType.RxDataLength = RxDataLength;
	CanDataType.BagState = BAG_NOT;
	CanDataType.LogBootFail_cnt = 0;
	CanDataType.can_err = 0;
    
    return CanDataType;
}
void Tim_GetCurrentTimeAdd_Scan1MS(void)
{
	Tim_GetCurrentTime++;
}
void CANCommunication_Init(const char *can_name)
{
    can_dev = rt_device_find(can_name);
    if (can_dev == RT_NULL)
    {
        rt_kprintf("CANComm: can not find device: %s\n", can_name);
        return;
    }
    rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);   // 以中断接收及发送方式打开 CAN 设备
    rt_device_control(can_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN500kBaud);     // 设置 CAN 通信的波特率为 500kbit/s
    rt_device_control(can_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);    // 设置 CAN 的工作模式为正常工作模式
    
    Tim_GetCurrentTime=2000;
    InitialUsedPGs();

}

void QueueOverflow_Dispose(struct rt_can_msg *pQ)
{
	CANCommunication_InformationParsing(pQ);
}



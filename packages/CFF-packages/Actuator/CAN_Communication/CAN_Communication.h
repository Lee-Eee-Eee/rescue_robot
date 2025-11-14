#ifndef __CAN_Communication_H__
#define __CAN_Communication_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* 包含头文件 ----------------------------------------------------------------*/
#include <rtthread.h>
#include <rtdevice.h>
#include "CAN_Communication_canconfig.h"
    
/* 宏定义 --------------------------------------------------------------------*/
#ifndef  FALSE
    #define  FALSE    0
#endif
#ifndef  TRUE
    #define  TRUE     1
#endif


/* 类型定义 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
extern uint8_t canrxtxerr;
/* 函数声明 ------------------------------------------------------------------*/
void CANCommunication_Init(const char *can_name);
void Tim_GetCurrentTimeAdd_Scan1MS(void);
void CANCommunication_Scan(void);
void CAN_TT_ASYNC_Enable(uint8_t list); //异步模式使能
void CAN_TT_EnableSet(uint8_t list,uint8_t enable); //使能失能
uint8_t CAN_TT_PGStateRead(uint8_t list); //读取CAN总线中列表序号对应帧的通讯状态
uint8_t CAN_TT_ErrRead(uint8_t list); //读取CAN总线中列表序号对应帧的错误信息
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
                            );

#ifdef __cplusplus
}
#endif

#endif  // __CAN_Communication_H__

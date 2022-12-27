#ifndef __CAN_SPI_H
#define	__CAN_SPI_H

#include "stm32f4xx_hal.h"
typedef union {
  struct {
    uint8_t IDType;
    uint32_t ID;
    uint8_t DLC;
    uint8_t Data0;
    uint8_t Data1;
    uint8_t Data2;
    uint8_t Data3;
    uint8_t Data4;
    uint8_t Data5;
    uint8_t Data6;
    uint8_t Data7;
  } frame;
  uint8_t array[14];
} CAN_HeaderTypeDef;

#define dSTANDARD_CAN_MSG_ID_2_0B 1
#define dEXTENDED_CAN_MSG_ID_2_0B 2

#define _500KBPS	1
#define _250KBPS	2

#define _Enable		1
#define _Disable	0

int CANSPI_Initialize(uint8_t BitRate);
void CANSPI_RxInterruptPins(uint8_t status);
void CANSPI_RXB0_CLR(void);
void CANSPI_Sleep(void);
uint8_t CAN_GetRxMessage(CAN_HeaderTypeDef *tempCanMsg);
uint8_t CAN_AddTxMessage(CAN_HeaderTypeDef *tempCanMsg);
uint8_t CANSPI_messagesInBuffer(void);
uint8_t CANSPI_isBussOff(void);
uint8_t CANSPI_isRxErrorPassive(void);
uint8_t CANSPI_isTxErrorPassive(void);

#endif	/* __CAN_SPI_H */

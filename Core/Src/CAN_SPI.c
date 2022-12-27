#include "CAN_SPI.h"
#include "MCP2515.h"

/* -------------------------------
 * | Before using SPI CAN Driver |
 * -------------------------------
 * ------------------------------------------------------------------------------
 * | 1. Modify the Hardware Configuration and Pins using CubeMX software  		|
 * | This driver use SPI3 with the following pins:								|
 * | --------------------------------------------------------------------------	|
 * | PC7 -> SPI3_CS | PC10 -> SPI3_SCK | PC11 -> SPI3_MISO |PC12 -> SPI3_MOSI 	|
 * | -------------------------------------------------------------------------- |
 * |																			|
 * | This driver use PC9 pin as RX0B interrupt pin (Named: RX0B_Interrupt)		|
 * | -------------------------------------------------------------------------- |
 * |																			|
 * | This Board has an optional LED and Buzzer with the following pins:			|
 * | 				------------------------------								|
 * | 				| PC5 -> LED | PA0 -> Buzzer | 								|
 * | 			 	------------------------------								|
 * ------------------------------------------------------------------------------
 *
 * ----------------------------------
 * | Use CAN SPI Driver instruction |
 * ----------------------------------
 * ------------------------------------------------------------------------------
 * | 1. Initialize the CAN SPI driver using: CANSPI_Initialize(uint8_t BitRate) |
 * | Two Bit Rate options:														|
 * | 500Kbps -> _500KBPS														|
 * | 250Kbps -> _250KBPS														|
 * | -------------------------------------------------------------------------- |
 * | 2. Enable/Disable RX pins Interrupt (on RX0B and RX1B pins)				|
 * | Two available options:														|
 * | Enable Interrupts on RX0B and RX1B pins  -> _Enable						|
 * | Disable Interrupts on RX0B and RX1B pins -> _Disable						|
 * |																			|
 * | NOTE: When using the RX interrupts, the interrupt flag MUST be cleared by	|
 * |	   software using CANSPI_RXB0_CLR Function								|
 * | -------------------------------------------------------------------------- |
 * | 3. Use Transmission and Receive functions 									|
 * | > Transmission Function													|
 * |   CAN_AddTxMessage(CAN_HeaderTypeDef *tempCanMsg)							|
 * |																			|
 * | > Receive Function															|
 * |   CAN_GetRxMessage(CAN_HeaderTypeDef *tempCanMsg)							|
 * ------------------------------------------------------------------------------
 */

/* Local Function Prototypes */
static uint32_t convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH, uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL);
static uint32_t convertReg2StandardCANid(uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL) ;
static void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType, id_reg_t *passedIdReg);

/** Local Variables */
ctrl_status_t ctrlStatus;
ctrl_error_status_t errorStatus;
id_reg_t idReg;

/** CAN SPI APIs */

/* Sleep 모드 진입 */
void CANSPI_Sleep(void)
{
	/* Clear CAN bus wakeup interrupt */
	MCP2515_BitModify(MCP2515_CANINTF, 0x40, 0x00);

	/* Enable CAN bus activity wakeup */
	MCP2515_BitModify(MCP2515_CANINTE, 0x40, 0x40);

	MCP2515_SetSleepMode();
}

/* Initialize the SPI CAN */
/* Two Bit Rate options:
 * 500Kbps -> _500KBPS
 * 250Kbps -> _250KBPS
 */
int CANSPI_Initialize(uint8_t BitRate)
{
	/* Enable the MCP2515 Chip */
	HAL_GPIO_WritePin(SPI_CAN_NRST_GPIO_Port, SPI_CAN_NRST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

	RXF0 RXF0reg;
	RXF1 RXF1reg;
	RXF2 RXF2reg;
	RXF3 RXF3reg;
	RXF4 RXF4reg;
	RXF5 RXF5reg;
	RXM0 RXM0reg;
	RXM1 RXM1reg;

	/* Rx Mask values 초기화 */
	RXM0reg.RXM0SIDH = 0x00;
	RXM0reg.RXM0SIDL = 0x00;
	RXM0reg.RXM0EID8 = 0x00;
	RXM0reg.RXM0EID0 = 0x00;

	RXM1reg.RXM1SIDH = 0x00;
	RXM1reg.RXM1SIDL = 0x00;
	RXM1reg.RXM1EID8 = 0x00;
	RXM1reg.RXM1EID0 = 0x00;

	/* Rx Filter values 초기화 */
	RXF0reg.RXF0SIDH = 0x00;
	RXF0reg.RXF0SIDL = 0x00;      //Starndard Filter
	RXF0reg.RXF0EID8 = 0x00;
	RXF0reg.RXF0EID0 = 0x00;

	RXF1reg.RXF1SIDH = 0x00;
	RXF1reg.RXF1SIDL = 0x08;      //Exntended Filter
	RXF1reg.RXF1EID8 = 0x00;
	RXF1reg.RXF1EID0 = 0x00;

	RXF2reg.RXF2SIDH = 0x00;
	RXF2reg.RXF2SIDL = 0x00;
	RXF2reg.RXF2EID8 = 0x00;
	RXF2reg.RXF2EID0 = 0x00;

	RXF3reg.RXF3SIDH = 0x00;
	RXF3reg.RXF3SIDL = 0x00;
	RXF3reg.RXF3EID8 = 0x00;
	RXF3reg.RXF3EID0 = 0x00;

	RXF4reg.RXF4SIDH = 0x00;
	RXF4reg.RXF4SIDL = 0x00;
	RXF4reg.RXF4EID8 = 0x00;
	RXF4reg.RXF4EID0 = 0x00;

	RXF5reg.RXF5SIDH = 0x00;
	RXF5reg.RXF5SIDL = 0x08;
	RXF5reg.RXF5EID8 = 0x00;
	RXF5reg.RXF5EID0 = 0x00;

	/* Initialize MCP2515, check SPI communication status */
	if(!MCP2515_Initialize())
		return -1;

	/* Set to configuration mode */
	if(!MCP2515_SetConfigMode())
		return -2;

	/* Filter & Mask 값 설정 */
	MCP2515_WriteByteSequence(MCP2515_RXM0SIDH, MCP2515_RXM0EID0, &(RXM0reg.RXM0SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXM1SIDH, MCP2515_RXM1EID0, &(RXM1reg.RXM1SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF0SIDH, MCP2515_RXF0EID0, &(RXF0reg.RXF0SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF1SIDH, MCP2515_RXF1EID0, &(RXF1reg.RXF1SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF2SIDH, MCP2515_RXF2EID0, &(RXF2reg.RXF2SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF3SIDH, MCP2515_RXF3EID0, &(RXF3reg.RXF3SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF4SIDH, MCP2515_RXF4EID0, &(RXF4reg.RXF4SIDH));
	MCP2515_WriteByteSequence(MCP2515_RXF5SIDH, MCP2515_RXF5EID0, &(RXF5reg.RXF5SIDH));

	/* Accept All (Standard + Extended) */
	MCP2515_WriteByte(MCP2515_RXB0CTRL, 0x04);    //Enable BUKT, Accept Filter 0
	MCP2515_WriteByte(MCP2515_RXB1CTRL, 0x01);    //Accept Filter 1

	/*
	 * tq = 2 * (brp(0) + 1) / 16000000 = 0.125us
	 * tbit = (SYNC_SEG(1 fixed) + PROP_SEG + PS1 + PS2)
	 * tbit = 1tq + 5tq + 6tq + 4tq = 16tq
	 * 16tq = 2us = 500kbps
	 */

	/* 250kbps */
	if (BitRate == _250KBPS) {
		/* 00(SJW 1tq) 000000 */
		MCP2515_WriteByte(MCP2515_CNF1, 0x00);

		/* 1 1 100(5tq) 101(6tq) */
		MCP2515_WriteByte(MCP2515_CNF2, 0xE5);

		/* 1 0 000 011(4tq) */
		MCP2515_WriteByte(MCP2515_CNF3, 0x83);
	}

	/* 500kbps */
	if (BitRate == _500KBPS) {
		/* 00(SJW 1tq) 000000 */
		MCP2515_WriteByte(MCP2515_CNF1, 0x00);

		/* 1 1 100(5tq) 101(6tq) */
		MCP2515_WriteByte(MCP2515_CNF2, 0x88);

		/* 1 0 000 011(4tq) */
		MCP2515_WriteByte(MCP2515_CNF3, 0x03);
	}

	/* Normal 모드로 설정 */
	if(!MCP2515_SetNormalMode())
		return -3;

	return 1;
}

void CANSPI_RxInterruptPins(uint8_t status) {
	/* This values are related to Table 4-1 and Register 4-3 */
	if (status == _Enable) {
		/* Enable Interrupt when RXB0 register is full ? Check ?*/
		MCP2515_WriteByte (MCP2515_RX0BF, 0x0F);
		/* Receive Buffer 0 Full Interrupt Enable */
		MCP2515_WriteByte (MCP2515_CANINTE, 0x03);
	}
	else if(status == _Disable) {
		MCP2515_WriteByte (MCP2515_RX0BF, 0x00);
	}
}

void CANSPI_RXB0_CLR(void) {
	MCP2515_WriteByte(MCP2515_CANINTF, 0x00);
}

/* CAN 메시지 전송 */
uint8_t CAN_GetRxMessage(CAN_HeaderTypeDef *tempCanMsg)
{
	uint8_t returnValue = 0;

	idReg.tempSIDH = 0;
	idReg.tempSIDL = 0;
	idReg.tempEID8 = 0;
	idReg.tempEID0 = 0;

	ctrlStatus.ctrl_status = MCP2515_ReadStatus();

	/* 현재 Transmission 이 Pending 되지 않은 버퍼를 찾아서 전송한다. */
	if (ctrlStatus.TXB0REQ != 1)
	{
		/* ID Type에 맞게 변환 */
		convertCANid2Reg(tempCanMsg->frame.ID, tempCanMsg->frame.IDType, &idReg);

		/* Tx Buffer에 전송할 데이터 Loading */
		MCP2515_LoadTxSequence(MCP2515_LOAD_TXB0SIDH, &(idReg.tempSIDH), tempCanMsg->frame.DLC, &(tempCanMsg->frame.Data0));

		/* Tx Buffer의 데이터 전송요청 */
		MCP2515_RequestToSend(MCP2515_RTS_TX0);

		returnValue = 1;
	}
	else if (ctrlStatus.TXB1REQ != 1)
	{
		convertCANid2Reg(tempCanMsg->frame.ID, tempCanMsg->frame.IDType, &idReg);

		MCP2515_LoadTxSequence(MCP2515_LOAD_TXB1SIDH, &(idReg.tempSIDH), tempCanMsg->frame.DLC, &(tempCanMsg->frame.Data0));
		MCP2515_RequestToSend(MCP2515_RTS_TX1);

		returnValue = 1;
	}
	else if (ctrlStatus.TXB2REQ != 1)
	{
		convertCANid2Reg(tempCanMsg->frame.ID, tempCanMsg->frame.IDType, &idReg);

		MCP2515_LoadTxSequence(MCP2515_LOAD_TXB2SIDH, &(idReg.tempSIDH), tempCanMsg->frame.DLC, &(tempCanMsg->frame.Data0));
		MCP2515_RequestToSend(MCP2515_RTS_TX2);

		returnValue = 1;
	}

	return (returnValue);
}

/* CAN 메시지 수신 */
uint8_t CAN_AddTxMessage(CAN_HeaderTypeDef *tempCanMsg)
{
	uint8_t returnValue = 0;
	rx_reg_t rxReg;
	ctrl_rx_status_t rxStatus;

	rxStatus.ctrl_rx_status = MCP2515_GetRxStatus();

	/* 버퍼에 수신된 메시지가 있는지 확인 */
	if (rxStatus.rxBuffer != 0)
	{
		/* 어떤 버퍼에 메시지가 있는지 확인 후 처리 */
		if ((rxStatus.rxBuffer == MSG_IN_RXB0)|(rxStatus.rxBuffer == MSG_IN_BOTH_BUFFERS))
		{
			MCP2515_ReadRxSequence(MCP2515_READ_RXB0SIDH, rxReg.rx_reg_array, sizeof(rxReg.rx_reg_array));
		}
		else if (rxStatus.rxBuffer == MSG_IN_RXB1)
		{
			MCP2515_ReadRxSequence(MCP2515_READ_RXB1SIDH, rxReg.rx_reg_array, sizeof(rxReg.rx_reg_array));
		}

		/* Extended 타입 */
		if (rxStatus.msgType == dEXTENDED_CAN_MSG_ID_2_0B)
		{
			tempCanMsg->frame.IDType = (uint8_t) dEXTENDED_CAN_MSG_ID_2_0B;
			tempCanMsg->frame.ID = convertReg2ExtendedCANid(rxReg.RXBnEID8, rxReg.RXBnEID0, rxReg.RXBnSIDH, rxReg.RXBnSIDL);
		}
		else
		{
			/* Standard 타입 */
			tempCanMsg->frame.IDType = (uint8_t) dSTANDARD_CAN_MSG_ID_2_0B;
			tempCanMsg->frame.ID = convertReg2StandardCANid(rxReg.RXBnSIDH, rxReg.RXBnSIDL);
		}

		tempCanMsg->frame.DLC   = rxReg.RXBnDLC;
		tempCanMsg->frame.Data0 = rxReg.RXBnD0;
		tempCanMsg->frame.Data1 = rxReg.RXBnD1;
		tempCanMsg->frame.Data2 = rxReg.RXBnD2;
		tempCanMsg->frame.Data3 = rxReg.RXBnD3;
		tempCanMsg->frame.Data4 = rxReg.RXBnD4;
		tempCanMsg->frame.Data5 = rxReg.RXBnD5;
		tempCanMsg->frame.Data6 = rxReg.RXBnD6;
		tempCanMsg->frame.Data7 = rxReg.RXBnD7;

		returnValue = 1;
	}

	return (returnValue);
}

/* 수신 버퍼에 메시지가 있는지 체크 */
uint8_t CANSPI_messagesInBuffer(void)
{
	uint8_t messageCount = 0;

	ctrlStatus.ctrl_status = MCP2515_ReadStatus();

	if(ctrlStatus.RX0IF != 0)
	{
		messageCount++;
	}

	if(ctrlStatus.RX1IF != 0)
	{
		messageCount++;
	}

	return (messageCount);
}

/* CAN BUS 가 Offline 인지 체크 */
uint8_t CANSPI_isBussOff(void)
{
	uint8_t returnValue = 0;

	errorStatus.error_flag_reg = MCP2515_ReadByte(MCP2515_EFLG);

	if(errorStatus.TXBO == 1)
	{
		returnValue = 1;
	}

	return (returnValue);
}

/* Rx Passive Error 상태인지 체크 */
uint8_t CANSPI_isRxErrorPassive(void)
{
	uint8_t returnValue = 0;

	errorStatus.error_flag_reg = MCP2515_ReadByte(MCP2515_EFLG);

	if(errorStatus.RXEP == 1)
	{
		returnValue = 1;
	}

	return (returnValue);
}

/* Tx Passive Error 상태인지 체크 */
uint8_t CANSPI_isTxErrorPassive(void)
{
	uint8_t returnValue = 0;

	errorStatus.error_flag_reg = MCP2515_ReadByte(MCP2515_EFLG);

	if(errorStatus.TXEP == 1)
	{
		returnValue = 1;
	}

	return (returnValue);
}

/* Register 저장값을 Extended ID 타입으로 변환하기 위한 함수 */
static uint32_t convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH, uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL)
{
	uint32_t returnValue = 0;
	uint32_t ConvertedID = 0;
	uint8_t CAN_standardLo_ID_lo2bits;
	uint8_t CAN_standardLo_ID_hi3bits;

	CAN_standardLo_ID_lo2bits = (tempRXBn_SIDL & 0x03);
	CAN_standardLo_ID_hi3bits = (tempRXBn_SIDL >> 5);
	ConvertedID = (tempRXBn_SIDH << 3);
	ConvertedID = ConvertedID + CAN_standardLo_ID_hi3bits;
	ConvertedID = (ConvertedID << 2);
	ConvertedID = ConvertedID + CAN_standardLo_ID_lo2bits;
	ConvertedID = (ConvertedID << 8);
	ConvertedID = ConvertedID + tempRXBn_EIDH;
	ConvertedID = (ConvertedID << 8);
	ConvertedID = ConvertedID + tempRXBn_EIDL;
	returnValue = ConvertedID;
	return (returnValue);
}

/* Register 저장값을 Standard ID 타입으로 변환하기 위한 함수 */
static uint32_t convertReg2StandardCANid(uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL)
{
	uint32_t returnValue = 0;
	uint32_t ConvertedID;

	ConvertedID = (tempRXBn_SIDH << 3);
	ConvertedID = ConvertedID + (tempRXBn_SIDL >> 5);
	returnValue = ConvertedID;

	return (returnValue);
}

/* CAN ID를 Register에 저장하기 위한 변환 함수 */
static void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType, id_reg_t *passedIdReg)
{
	uint8_t wipSIDL = 0;

	if (canIdType == dEXTENDED_CAN_MSG_ID_2_0B)
	{
		//EID0
		passedIdReg->tempEID0 = 0xFF & tempPassedInID;
		tempPassedInID = tempPassedInID >> 8;

		//EID8
		passedIdReg->tempEID8 = 0xFF & tempPassedInID;
		tempPassedInID = tempPassedInID >> 8;

		//SIDL
		wipSIDL = 0x03 & tempPassedInID;
		tempPassedInID = tempPassedInID << 3;
		wipSIDL = (0xE0 & tempPassedInID) + wipSIDL;
		wipSIDL = wipSIDL + 0x08;
		passedIdReg->tempSIDL = 0xEB & wipSIDL;

		//SIDH
		tempPassedInID = tempPassedInID >> 8;
		passedIdReg->tempSIDH = 0xFF & tempPassedInID;
	}
	else
	{
		passedIdReg->tempEID8 = 0;
		passedIdReg->tempEID0 = 0;
		tempPassedInID = tempPassedInID << 5;
		passedIdReg->tempSIDL = 0xFF & tempPassedInID;
		tempPassedInID = tempPassedInID >> 8;
		passedIdReg->tempSIDH = 0xFF & tempPassedInID;
	}
}

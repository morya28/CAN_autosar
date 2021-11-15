/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "Can.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_FLEXCAN_IRQHandler CAN0_ORed_Message_buffer_IRQHandler
#define LOG_INFO (void)PRINTF


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void EXAMPLE_FLEXCAN_IRQHandler(void);
status_t CAN_TransferSendBlocking(Can_PduType *pTxFrame);
void Can_MainFunction_Write(void);
void Can_MainFunction_Read(void);
void print_rxFrame(Can_PduType rxFrame);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool rxComplete = false;
Can_PduType txFrame, rxFrame;


/*******************************************************************************
 * Code
 ******************************************************************************/

void EXAMPLE_FLEXCAN_IRQHandler(void)
{
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    uint64_t flag = 1U;
#else
    uint32_t flag = 1U;
#endif
    /* If new data arrived. */
    if (0U != FLEXCAN_GetMbStatusFlags(EXAMPLE_CAN, flag << RX_MESSAGE_BUFFER_NUM)) {
        FLEXCAN_ClearMbStatusFlags(EXAMPLE_CAN, flag << RX_MESSAGE_BUFFER_NUM);
#if (defined(USE_CANFD) && USE_CANFD)
        (void)FLEXCAN_ReadFDRxMb(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &rxFrame);
#else
        (void)FLEXCAN_ReadRxMb(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &rxFrame);
#endif
        rxComplete = true;
    }
    SDK_ISR_EXIT_BARRIER;
}


/*!
 * Performs a polling send transaction on the CAN bus.
 *
 * param pTxFrame Pointer to CAN message frame to be sent.
 * retval kStatus_Success - Write Tx Message Buffer Successfully.
 * retval kStatus_Fail    - Tx Message Buffer is currently in use.
 */
status_t CAN_TransferSendBlocking(Can_PduType *pTxFrame) {
    status_t status;
    /* Write Tx Message Buffer to initiate a data sending. */
    if (kStatus_Success == CAN_Write(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, (const Can_PduType *)(uint32_t)pTxFrame)) {
    	Can_MainFunction_Write();
        /* After TX MB transferred success, update the Timestamp from MB[TX_MESSAGE_BUFFER_NUM].CS register */
        pTxFrame->timestamp = (uint16_t)((EXAMPLE_CAN->MB[TX_MESSAGE_BUFFER_NUM].CS & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);
        status = kStatus_Success;
    }
    else {
        status = kStatus_Fail;
    }
    /* Waiting for Message receive finish. */
    while (!rxComplete) { }
    return status;
}


void Can_MainFunction_Write() {
	/* Wait until CAN Message send out. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
	uint64_t u64flag = 1;
	while (0U == CAN_GetMbStatusFlags(EXAMPLE_CAN, u64flag << TX_MESSAGE_BUFFER_NUM))
#else
	uint32_t u32flag = 1;
	while (0U == CAN_GetMbStatusFlags(EXAMPLE_CAN, u32flag << TX_MESSAGE_BUFFER_NUM))
#endif
	{ }
/* Clean Tx Message Buffer Flag. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
	CAN_ClearMbStatusFlags(EXAMPLE_CAN, u64flag << TX_MESSAGE_BUFFER_NUM);
#else
	CAN_ClearMbStatusFlags(EXAMPLE_CAN, u32flag << TX_MESSAGE_BUFFER_NUM);
#endif
}


void Can_MainFunction_Read() {
	/* Assertion. */
	assert(TX_MESSAGE_BUFFER_NUM <= (EXAMPLE_CAN->MCR & CAN_MCR_MAXMB_MASK));
	uint32_t cs_temp;
	uint32_t rx_code;
	status_t status;
	/* Read CS field of Rx Message Buffer to lock Message Buffer. */
	cs_temp = EXAMPLE_CAN->MB[TX_MESSAGE_BUFFER_NUM].CS;
	/* Get Rx Message Buffer Code field. */
	rx_code = (cs_temp & CAN_CS_CODE_MASK) >> CAN_CS_CODE_SHIFT;
	/* Check to see if Rx Message Buffer is full. */
	if (CAN_CS_CODE(kCAN_TxMbDataOrRemote) != (cs_temp & CAN_CS_CODE_MASK)) {
		/* Store Message ID. */
		rxFrame.id = EXAMPLE_CAN->MB[TX_MESSAGE_BUFFER_NUM].ID & (CAN_ID_EXT_MASK | CAN_ID_STD_MASK);
		/* Get the message ID and format. */
		rxFrame.format = (cs_temp & CAN_CS_IDE_MASK) != 0U ? (uint8_t)kFLEXCAN_FrameFormatExtend :
															 (uint8_t)kFLEXCAN_FrameFormatStandard;
		/* Get the message type. */
		rxFrame.type = (cs_temp & CAN_CS_RTR_MASK) != 0U ? (uint8_t)kFLEXCAN_FrameTypeRemote : (uint8_t)kFLEXCAN_FrameTypeData;
		/* Get the message length. */
		rxFrame.length = (uint8_t)((cs_temp & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT);
		/* Get the time stamp. */
		rxFrame.timestamp = (uint16_t)((cs_temp & CAN_CS_TIME_STAMP_MASK) >> CAN_CS_TIME_STAMP_SHIFT);
		/* Store Message Payload. */
		rxFrame.dataWord0 = EXAMPLE_CAN->MB[TX_MESSAGE_BUFFER_NUM].WORD0;
		rxFrame.dataWord1 = EXAMPLE_CAN->MB[TX_MESSAGE_BUFFER_NUM].WORD1;
		/* Read free-running timer to unlock Rx Message Buffer. */
		(void)EXAMPLE_CAN->TIMER;
		if ((uint32_t)kCAN_RxMbFull == rx_code) {
			status = kStatus_Success;
		}
		else {
			status = kStatus_FLEXCAN_RxOverflow;
		}
		print_rxFrame(rxFrame);
	}
	else {
		/* Read free-running timer to unlock Rx Message Buffer. */
		(void)EXAMPLE_CAN->TIMER;
		status = kStatus_Fail;
	}
	LOG_INFO("rx status = %d\r\n", status);
}

void print_rxFrame(Can_PduType frame) {
	/* Store Message ID. */
	LOG_INFO("\r\nRead frame status = %d\r\n", frame.id);
	/* Get the message ID and format. */
	LOG_INFO("Read frame format = %d\r\n", frame.format);
	/* Get the message type. */
	LOG_INFO("Read frame type = %d\r\n", frame.type);
	/* Get the message length. */
	LOG_INFO("Read frame length = %d\r\n", frame.length);
	/* Get the time stamp. */
	LOG_INFO("Read frame timestamp = %d\r\n", frame.timestamp);
	/* Store Message Payload. */
	LOG_INFO("Read frame dataWord0 = 0x%x\r\n", frame.dataWord0);
	LOG_INFO("Read frame dataWord1 = 0x%x\r\n", frame.dataWord1);
}


int main(void) {
    can_rx_mb_config_t mbConfig;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    uint64_t flag = 1U;
#else
    uint32_t flag = 1U;
#endif
    /* Initialize board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    Can_Init(&canConfig_autosar);
    LOG_INFO("CAN loopback functional example - Start.\n\r");
    /* Setup Rx Message Buffer. */
    mbConfig.format = kCAN_FrameFormatStandard;
    mbConfig.type = kCAN_FrameTypeData;
    mbConfig.id = FLEXCAN_ID_STD(0x123);
    CAN_SetRxMbConfig(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);
    /* Setup Tx Message Buffer. */
    CAN_SetTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);
    CAN_EnableMbInterrupts(EXAMPLE_CAN, flag << RX_MESSAGE_BUFFER_NUM);
    (void)EnableIRQ(EXAMPLE_FLEXCAN_IRQn);
    /* Prepare Tx Frame for sending. */
    txFrame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
    txFrame.type = (uint8_t)kFLEXCAN_FrameTypeData;
    txFrame.id = FLEXCAN_ID_STD(0x123);
    txFrame.length = (uint8_t)DLC;
    txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x11) | CAN_WORD0_DATA_BYTE_1(0x22) | CAN_WORD0_DATA_BYTE_2(0x33) |
                        CAN_WORD0_DATA_BYTE_3(0x44);
    txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x55) | CAN_WORD1_DATA_BYTE_5(0x66) | CAN_WORD1_DATA_BYTE_6(0x77) |
                        CAN_WORD1_DATA_BYTE_7(0x88);
    LOG_INFO("Send message from MB%d to MB%d\r\n", TX_MESSAGE_BUFFER_NUM, RX_MESSAGE_BUFFER_NUM);
    LOG_INFO("tx word0 = 0x%x\r\n", txFrame.dataWord0);
    LOG_INFO("tx word1 = 0x%x\r\n", txFrame.dataWord1);
    /* Send data through Tx Message Buffer using polling function. */
    (void)CAN_TransferSendBlocking(&txFrame);
    LOG_INFO("\r\nReceived message from MB%d\r\n", RX_MESSAGE_BUFFER_NUM);
    LOG_INFO("rx word0 = 0x%x\r\n", rxFrame.dataWord0);
    LOG_INFO("rx word1 = 0x%x\r\n", rxFrame.dataWord1);

    Can_MainFunction_Read();
    /* Stop CAN Send & Receive. */
    CAN_DisableMbInterrupts(EXAMPLE_CAN, flag << RX_MESSAGE_BUFFER_NUM);
    LOG_INFO("CAN loopback functional example -- Finish.\r\n");
    while (true) { }
}

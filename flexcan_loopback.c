/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_flexcan.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "Can.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_FLEXCAN_IRQHandler CAN0_ORed_Message_buffer_IRQHandler


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void EXAMPLE_FLEXCAN_IRQHandler(void);


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


int main(void) {
    /* Initialize board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    Can_Init(&canConfig_autosar);
    LOG_INFO("CAN loopback functional example - Start.\n\r");
    /* Prepare Tx Frame for sending. */
    Can_PduType CanPdu;
	CanPdu.pdu_handle.mbIdx = TX_MESSAGE_BUFFER_NUM;
	CanPdu.id = FLEXCAN_ID_STD(0x123);
	CanPdu.length = (uint8_t)DLC;
	CanPdu.sdu.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x11) | CAN_WORD0_DATA_BYTE_1(0x22) | CAN_WORD0_DATA_BYTE_2(0x33) |
						   CAN_WORD0_DATA_BYTE_3(0x44);
	CanPdu.sdu.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x55) | CAN_WORD1_DATA_BYTE_5(0x66) | CAN_WORD1_DATA_BYTE_6(0x77) |
						   CAN_WORD1_DATA_BYTE_7(0x88);
	LOG_INFO("Send message from MB%d to MB%d\r\n", TX_MESSAGE_BUFFER_NUM, RX_MESSAGE_BUFFER_NUM);
	LOG_INFO("tx word0 = 0x%x\r\n", CanPdu.sdu.dataWord0);
	LOG_INFO("tx word1 = 0x%x\r\n", CanPdu.sdu.dataWord1);
	/* Transmit message */
	status_t status;
	status = CAN_Write(EXAMPLE_CAN, &CanPdu);
    Can_MainFunction_Write();

    /* Waiting for Message receive finish. */
    LOG_INFO("\r\nReceived message from MB%d\r\n", RX_MESSAGE_BUFFER_NUM);
    LOG_INFO("rx word0 = 0x%x\r\n", rxFrame.sdu.dataWord0);
    LOG_INFO("rx word1 = 0x%x\r\n", rxFrame.sdu.dataWord1);
    Can_MainFunction_Read();

    LOG_INFO("CAN loopback functional example -- Finish.\r\n");
    while (true) { }
}

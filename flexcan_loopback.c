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

// void EXAMPLE_FLEXCAN_IRQHandler(void);

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


int main(void)
{
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
    (void)CAN_TransferSendBlocking(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, &txFrame);
    /* Waiting for Message receive finish. */
    while (!rxComplete) { }
    LOG_INFO("\r\nReceived message from MB%d\r\n", RX_MESSAGE_BUFFER_NUM);
    LOG_INFO("rx word0 = 0x%x\r\n", rxFrame.dataWord0);
    LOG_INFO("rx word1 = 0x%x\r\n", rxFrame.dataWord1);
    /* Stop CAN Send & Receive. */
    CAN_DisableMbInterrupts(EXAMPLE_CAN, flag << RX_MESSAGE_BUFFER_NUM);
    LOG_INFO("CAN loopback functional example -- Finish.\r\n");
    while (true) { }
}

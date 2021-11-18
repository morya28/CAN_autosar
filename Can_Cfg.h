/*
 * Can_Cfg.h
 *
 *  Created on: Oct 15, 2021
 *      Author: Maximiliano Catalan
 */

#ifndef CAN_CFG_H_
#define CAN_CFG_H_

#include "stdbool.h"
#include "stdint.h"
#include "MK64F12.h"
#include "fsl_clock.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_CAN CAN0
#define EXAMPLE_CAN_CLK_SOURCE (kFLEXCAN_ClkSrc1_autosar)
#define EXAMPLE_CAN_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define USE_IMPROVED_TIMING_CONFIG (1U)
#define EXAMPLE_FLEXCAN_IRQn CAN0_ORed_Message_buffer_IRQn
#define RX_MESSAGE_BUFFER_NUM (9)
#define TX_MESSAGE_BUFFER_NUM (8)
#define DLC (8)


typedef enum _can_clock_source {
    kFLEXCAN_ClkSrcOsc_autosar  = 0x0U, /*!< FlexCAN Protocol Engine clock from Oscillator. */
    kFLEXCAN_ClkSrcPeri_autosar = 0x1U, /*!< FlexCAN Protocol Engine clock from Peripheral Clock. */
    kFLEXCAN_ClkSrc0_autosar = 0x0U, /*!< FlexCAN Protocol Engine clock selected by user as SRC == 0. */
    kFLEXCAN_ClkSrc1_autosar = 0x1U, /*!< FlexCAN Protocol Engine clock selected by user as SRC == 1. */
} can_clock_source_t;

typedef enum _can_wake_up_source {
    kFLEXCAN_WakeupSrcUnfiltered_autosar = 0x0U, /*!< FlexCAN uses unfiltered Rx input to detect edge. */
    kFLEXCAN_WakeupSrcFiltered_autosar   = 0x1U, /*!< FlexCAN uses filtered Rx input to detect edge. */
} can_wake_up_source_t;

typedef struct _can_timing_config {
    uint16_t preDivider; /*!< Classic CAN or CAN FD nominal phase bit rate prescaler. */
    uint8_t rJumpwidth;  /*!< Classic CAN or CAN FD nominal phase Re-sync Jump Width. */
    uint8_t phaseSeg1;   /*!< Classic CAN or CAN FD nominal phase Segment 1. */
    uint8_t phaseSeg2;   /*!< Classic CAN or CAN FD nominal phase Segment 2. */
    uint8_t propSeg;     /*!< Classic CAN or CAN FD nominal phase Propagation Segment. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    uint16_t fpreDivider; /*!< CAN FD data phase bit rate prescaler. */
    uint8_t frJumpwidth;  /*!< CAN FD data phase Re-sync Jump Width. */
    uint8_t fphaseSeg1;   /*!< CAN FD data phase Phase Segment 1. */
    uint8_t fphaseSeg2;   /*!< CAN FD data phase Phase Segment 2. */
    uint8_t fpropSeg;     /*!< CAN FD data phase Propagation Segment. */
#endif
} can_timing_config_t;

typedef  uint32_t (*CanClockFreq)(clock_name_t);

typedef struct _flexcan_config_autosar {
	CAN_Type *CAN_base;
	CanClockFreq CAN_clock_freq;
    union {
        struct {
            uint32_t baudRate; /*!< FlexCAN bit rate in bps, for classical CAN or CANFD nominal phase. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
            uint32_t baudRateFD; /*!< FlexCAN FD bit rate in bps, for CANFD data phase. */
#endif
        };
        struct {
            uint32_t bitRate; /*!< FlexCAN bit rate in bps, for classical CAN or CANFD nominal phase. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
            uint32_t bitRateFD; /*!< FlexCAN FD bit rate in bps, for CANFD data phase. */
#endif
        };
    };
    can_clock_source_t clkSrc;      /*!< Clock source for FlexCAN Protocol Engine. */
    can_wake_up_source_t wakeupSrc; /*!< Wake up source selection. */
    uint8_t maxMbNum;                   /*!< The maximum number of Message Buffers used by user. */
    bool enableLoopBack;                /*!< Enable or Disable Loop Back Self Test Mode. */
    bool enableTimerSync;               /*!< Enable or Disable Timer Synchronization. */
    bool enableSelfWakeup;              /*!< Enable or Disable Self Wakeup Mode. */
    bool enableIndividMask;             /*!< Enable or Disable Rx Individual Mask and Queue feature. */
    bool disableSelfReception;          /*!< Enable or Disable Self Reflection. */
    bool enableListenOnlyMode;          /*!< Enable or Disable Listen Only Mode. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT) && FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT)
    bool enableDoze; /*!< Enable or Disable Doze Mode. */
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
    bool enablePretendedeNetworking; /*!< Enable or Disable the Pretended Networking mode. */
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    bool enableMemoryErrorControl; /*!< Enable or Disable the memory errors detection and correction mechanism. */
    bool enableNonCorrectableErrorEnterFreeze; /*!< Enable or Disable Non-Correctable Errors In FlexCAN Access Put
                                                    Device In Freeze Mode. */
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
    bool enableTransceiverDelayMeasure; /*!< Enable or Disable the transceiver delay measurement, when it is enabled,
                                             then the secondary sample point position is determined by the sum of the
                                             transceiver delay measurement plus the enhanced TDC offset. */
#endif
    can_timing_config_t timingConfig; /* Protocol timing . */
} Can_ConfigType;

/*! CAN message frame structure. */
typedef struct _can_PduType {
	struct PduIdType{
		uint8_t mbIdx;
	} pdu_handle;
	uint32_t length : 4;     /*!< CAN frame data length in bytes (Range: 0~8). */
    struct {
        uint32_t id : 29; /*!< CAN Frame Identifier, should be set using FLEXCAN_ID_EXT() or FLEXCAN_ID_STD() macro. */
        uint32_t : 3;     /*!< Reserved. */
    };
    union {
        struct {
            uint32_t dataWord0; /*!< CAN Frame payload word0. */
            uint32_t dataWord1; /*!< CAN Frame payload word1. */
        };
        struct {
            uint8_t dataByte3; /*!< CAN Frame payload byte3. */
            uint8_t dataByte2; /*!< CAN Frame payload byte2. */
            uint8_t dataByte1; /*!< CAN Frame payload byte1. */
            uint8_t dataByte0; /*!< CAN Frame payload byte0. */
            uint8_t dataByte7; /*!< CAN Frame payload byte7. */
            uint8_t dataByte6; /*!< CAN Frame payload byte6. */
            uint8_t dataByte5; /*!< CAN Frame payload byte5. */
            uint8_t dataByte4; /*!< CAN Frame payload byte4. */
        };
    } sdu;
} Can_PduType;

typedef enum _can_frame_format {
    kCAN_FrameFormatStandard = 0x0U, /*!< Standard frame format attribute. */
    kCAN_FrameFormatExtend   = 0x1U, /*!< Extend frame format attribute. */
} can_frame_format_t;

typedef enum _can_frame_type {
    kCAN_FrameTypeData   = 0x0U, /*!< Data frame type attribute. */
    kCAN_FrameTypeRemote = 0x1U, /*!< Remote frame type attribute. */
} can_frame_type_t;

/*!
 * This structure is used as the parameter of CAN_SetRxMbConfig() function.
 * The CAN_SetRxMbConfig() function is used to configure CAN Receive
 * Message Buffer. The function abort previous receiving process, clean the
 * Message Buffer and activate the Rx Message Buffer using given Message Buffer
 * setting.
 */
typedef struct _can_rx_mb_config {
    uint32_t id; /*!< CAN Message Buffer Frame Identifier, should be set using FLEXCAN_ID_EXT() or FLEXCAN_ID_STD() macro. */
    can_frame_format_t format; /*!< CAN Frame Identifier format(Standard of Extend). */
    can_frame_type_t type;     /*!< CAN Frame Type(Data or Remote). */
} can_rx_mb_config_t;

extern Can_ConfigType canConfig_autosar;

#endif /* CAN_CFG_H_ */

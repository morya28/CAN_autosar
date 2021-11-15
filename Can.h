/*
 * Can.h
 *
 *  Created on: Oct 15, 2021
 *      Author: Maximiliano Catalan
 */

#ifndef CAN_H_
#define CAN_H_

#include "assert.h"
#include "stddef.h"
#include "Can_Cfg.h"


#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
#ifndef CAN_CLOCK_CHECK_NO_AFFECTS
/* If no define such MACRO, it mean that the CAN in current device have no clock affect issue. */
#define CAN_CLOCK_CHECK_NO_AFFECTS (true)
#endif /* CAN_CLOCK_CHECK_NO_AFFECTS */
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/* Array of FlexCAN clock name. */
static const clock_ip_name_t s_flexcanClock[] = FLEXCAN_CLOCKS;
#if defined(FLEXCAN_PERIPH_CLOCKS)
/* Array of FlexCAN serial clock name. */
static const clock_ip_name_t s_flexcanPeriphClock[] = FLEXCAN_PERIPH_CLOCKS;
#endif /* FLEXCAN_PERIPH_CLOCKS */
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
/* Array of CAN peripheral base address. */
static CAN_Type *const s_flexcanBases[] = CAN_BASE_PTRS;


// Definitions
#define MAX_PRESDIV (CAN_CTRL1_PRESDIV_MASK >> CAN_CTRL1_PRESDIV_SHIFT)
/* TSEG1 corresponds to the sum of xPROPSEG and xPSEG1, TSEG2 corresponds to the xPSEG2 value. */
#define MIN_TIME_SEGMENT1 (2U)
#define MIN_TIME_SEGMENT2 (2U)
/* Define the max value of bit timing segments when use different timing register. */
#define MAX_PROPSEG           (CAN_CTRL1_PROPSEG_MASK >> CAN_CTRL1_PROPSEG_SHIFT)
#define MAX_PSEG1             (CAN_CTRL1_PSEG1_MASK >> CAN_CTRL1_PSEG1_SHIFT)
#define MAX_PSEG2             (CAN_CTRL1_PSEG2_MASK >> CAN_CTRL1_PSEG2_SHIFT)
#define MAX_RJW               (CAN_CTRL1_RJW_MASK >> CAN_CTRL1_RJW_SHIFT)
#define MAX_PRESDIV           (CAN_CTRL1_PRESDIV_MASK >> CAN_CTRL1_PRESDIV_SHIFT)
#define CTRL1_MAX_TIME_QUANTA (1U + MAX_PROPSEG + 1U + MAX_PSEG1 + 1U + MAX_PSEG2 + 1U)
#define CTRL1_MIN_TIME_QUANTA (8U)
#define IDEAL_SP_FACTOR (1000U)
/* According to CiA doc 301 v4.2.0 and previous version. */
#define IDEAL_SP_LOW  (750U)
#define IDEAL_SP_MID  (800U)
#define IDEAL_SP_HIGH (875U)
/* Define maximum CAN bit rate supported by CAN. */
#define MAX_CAN_BITRATE   (1000000U)


/*! CAN message buffer CODE for Rx buffers. */
enum _flexcan_mb_code_rx
{
    kCAN_RxMbInactive = 0x0, /*!< MB is not active.*/
    kCAN_RxMbFull = 0x2, /*!< MB is full.*/
    kCAN_RxMbEmpty = 0x4, /*!< MB is active and empty.*/
    kCAN_RxMbOverrun = 0x6, /*!< MB is overwritten into a full buffer.*/
    kCAN_RxMbBusy = 0x8, /*!< CAN is updating the contents of the MB, The CPU must not access the MB.*/
    kCAN_RxMbRanswer = 0xA, /*!< A frame was configured to recognize a Remote Request Frame and transmit a Response Frame in return.*/
    kCAN_RxMbNotUsed = 0xF,  /*!< Not used.*/
};


enum _can_mb_code_tx {
    kCAN_TxMbInactive     = 0x8, /*!< MB is not active.*/
    kCAN_TxMbAbort        = 0x9, /*!< MB is aborted.*/
    kCAN_TxMbDataOrRemote = 0xC, /*!< MB is a TX Data Frame(when MB RTR = 0) or MB is a TX Remote Request
                                      Frame (when MB RTR = 1).*/
    kCAN_TxMbTanswer = 0xE,      /*!< MB is a TX Response Request Frame from an incoming Remote Request Frame.*/
    kCAN_TxMbNotUsed = 0xF,      /*!< Not used.*/
};


// Function prototypes
uint32_t CAN_GetInstance(
	CAN_Type *base
);
static inline void CAN_Enable(
	CAN_Type *base, bool enable
);
static void CAN_SetBitRate(
	CAN_Type *base,
	uint32_t sourceClock_Hz,
	uint32_t bitRate_Bps,
	can_timing_config_t timingConfig
);
void CAN_ExitFreezeMode(
	CAN_Type *base
);
void CAN_EnterFreezeMode(
	CAN_Type *base
);
void CAN_SetTimingConfig(
	CAN_Type *base,
	const can_timing_config_t *pConfig
);
static void CAN_Reset(
	CAN_Type *base
);
void CAN_SetRxMbConfig(
	CAN_Type *base,
	uint8_t mbIdx,
	const can_rx_mb_config_t *pRxMbConfig,
	bool enable
);
void CAN_SetTxMbConfig(
	CAN_Type *base,
	uint8_t mbIdx,
	bool enable
);
status_t CAN_Write(
	CAN_Type *base,
	uint8_t mbIdx,
	const Can_PduType *pTxFrame
);
bool CAN_CalculateImprovedTimingValues(
	CAN_Type *base,
	uint32_t bitRate,
	uint32_t sourceClock_Hz,
	can_timing_config_t *pTimingConfig
);
static void CAN_GetSegments(
	CAN_Type *base,
    uint32_t bitRate,
	uint32_t tqNum,
	can_timing_config_t *pTimingConfig
);
void Can_Init(
	const Can_ConfigType* Config
);


/*!
 * This function clears the interrupt flags of a given Message Buffers.
 *
 * @param base CAN peripheral base address.
 * @param mask The ORed CAN Message Buffer mask.
 */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
static inline void FLEXCAN_ClearMbStatusFlags(CAN_Type *base, uint64_t mask)
#else
static inline void CAN_ClearMbStatusFlags(CAN_Type *base, uint32_t mask)
#endif
{
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    base->IFLAG1 = (uint32_t)(mask & 0xFFFFFFFFU);
    base->IFLAG2 = (uint32_t)(mask >> 32);
#else
    base->IFLAG1 = mask;
#endif
}


/*!
 * This function gets the interrupt flags of a given Message Buffers.
 *
 * @param base CAN peripheral base address.
 * @param mask The ORed CAN Message Buffer mask.
 * @return The status of given Message Buffers.
 */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
static inline uint64_t CAN_GetMbStatusFlags(CAN_Type *base, uint64_t mask)
#else
static inline uint32_t CAN_GetMbStatusFlags(CAN_Type *base, uint32_t mask)
#endif
{
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    uint64_t tempflag = (uint64_t)base->IFLAG1;
    return (tempflag | (((uint64_t)base->IFLAG2) << 32)) & mask;
#else
    return (base->IFLAG1 & mask);
#endif
}


/*!
 * This function enables the interrupts of given Message Buffers.
 *
 * @param base CAN peripheral base address.
 * @param mask The ORed CAN Message Buffer mask.
 */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
static inline void CAN_EnableMbInterrupts(CAN_Type *base, uint64_t mask)
#else
static inline void CAN_EnableMbInterrupts(CAN_Type *base, uint32_t mask)
#endif
{
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    base->IMASK1 |= (uint32_t)(mask & 0xFFFFFFFFU);
    base->IMASK2 |= (uint32_t)(mask >> 32);
#else
    base->IMASK1 |= mask;
#endif
}


/*!
 * This function disables the interrupts of given Message Buffers.
 *
 * @param base CAN peripheral base address.
 * @param mask The ORed CAN Message Buffer mask.
 */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
static inline void CAN_DisableMbInterrupts(CAN_Type *base, uint64_t mask)
#else
static inline void CAN_DisableMbInterrupts(CAN_Type *base, uint32_t mask)
#endif
{
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    base->IMASK1 &= ~((uint32_t)(mask & 0xFFFFFFFFU));
    base->IMASK2 &= ~((uint32_t)(mask >> 32));
#else
    base->IMASK1 &= ~mask;
#endif
}

#endif /* CAN_H_ */

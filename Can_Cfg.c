/*
 * Can_Cfg.c
 *
 *  Created on: Oct 15, 2021
 *      Author: Maximiliano Catalan
 */

#include "Can_Cfg.h"

#define EXAMPLE_CAN CAN0

Can_ConfigType canConfig_autosar = {
	.CAN_base = EXAMPLE_CAN,
	.CAN_clock_freq = CLOCK_GetFreq,
	.clkSrc = EXAMPLE_CAN_CLK_SOURCE,
	.bitRate = 1000000U,
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    .bitRateFD = 2000000U,
#endif
    .maxMbNum = 16,
    .enableLoopBack = true,
    .enableTimerSync = true,
    .enableSelfWakeup = false,
    .wakeupSrc = kFLEXCAN_WakeupSrcUnfiltered_autosar,
    .enableIndividMask = false,
    .disableSelfReception = false,
    .enableListenOnlyMode = false,
#if (defined(FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT) && FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT)
    .enableDoze = false,
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
    .enablePretendedeNetworking = false,
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    .enableMemoryErrorControl = true,
    .enableNonCorrectableErrorEnterFreeze = true,
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
    .enableTransceiverDelayMeasure = true,
#endif
    /* Default protocol timing configuration, nominal bit time quantum is 10 (80% SP), data bit time quantum is 5
     * (60%). */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    .timingConfig.phaseSeg1 = 1,
    .timingConfig.phaseSeg2 = 1,
    .timingConfig.propSeg = 4,
    .timingConfig.rJumpwidth = 1,
    .timingConfig.fphaseSeg1 = 1,
    .timingConfig.fphaseSeg2 = 1,
    .timingConfig.fpropSeg = 0,
    .timingConfig.frJumpwidth = 1,
#else
    .timingConfig.phaseSeg1 = 1,
    .timingConfig.phaseSeg2 = 1,
    .timingConfig.propSeg = 4,
    .timingConfig.rJumpwidth = 1
#endif
};


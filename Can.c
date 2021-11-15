/*
 * Can.c
 *
 *  Created on: Oct 15, 2021
 *      Author: Maximiliano Catalan
 */

#include "Can.h"


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


/*!
 * Get the CAN instance from peripheral base address.
 *
 * base: CAN peripheral base address.
 * return: CAN instance.
 */
uint32_t CAN_GetInstance(CAN_Type *base) {
    uint32_t instance;
    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_flexcanBases); instance++) {
        if (s_flexcanBases[instance] == base) {
        	break;
        }
    }
    assert(instance < ARRAY_SIZE(s_flexcanBases));
    return instance;
}


/*!
 * This function enables or disables the CAN module.
 *
 * base: CAN base pointer.
 * enable: true to enable, false to disable.
 */
static inline void CAN_Enable(CAN_Type *base, bool enable) {
    if (enable) {
        base->MCR &= ~CAN_MCR_MDIS_MASK;
        /* Wait CAN exit from low-power mode. */
        while (0U != (base->MCR & CAN_MCR_LPMACK_MASK)) { }
    }
    else {
        base->MCR |= CAN_MCR_MDIS_MASK;
        /* Wait CAN enter low-power mode. */
        while (0U == (base->MCR & CAN_MCR_LPMACK_MASK)) { }
    }
}


/*!
 * This function set the bit rate CAN frame nominal phase base on the value of the parameter passed in. Users need to ensure
 * that the timing segment values (phaseSeg1, phaseSeg2 and propSeg) match the clock and bit rate, if not match, the final
 * output bit rate may not equal the bitRate_Bps value.
 *
 * base: CAN peripheral base address.
 * sourceClock_Hz: Source Clock in Hz.
 * bitRate_Bps: Bit rate in Bps.
 * timingConfig: CAN timingConfig.
 */
static void CAN_SetBitRate(CAN_Type *base, uint32_t sourceClock_Hz, uint32_t bitRate_Bps, can_timing_config_t timingConfig) {
    /* CAN frame nominal phase timing setting formula: quantum = 1 + (phaseSeg1 + 1) + (phaseSeg2 + 1) + (propSeg + 1); */
    uint32_t quantum = (1U + ((uint32_t)timingConfig.phaseSeg1 + 1U) + ((uint32_t)timingConfig.phaseSeg2 + 1U) +
                        ((uint32_t)timingConfig.propSeg + 1U));
    /* Assertion: Desired bit rate is too high. */
    assert(bitRate_Bps <= 1000000U);
    /* Assertion: Source clock should greater than or equal to bit rate * quantum. */
    assert((bitRate_Bps * quantum) <= sourceClock_Hz);
    /* Assertion: Desired bit rate is too low, the bit rate * quantum * max prescaler divider value should greater than
       or equal to source clock. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    if (0 != FSL_FEATURE_FLEXCAN_INSTANCE_HAS_FLEXIBLE_DATA_RATEn(base)) {
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
        assert((bitRate_Bps * quantum * MAX_ENPRESDIV) >= sourceClock_Hz);
#else
        assert((bitRate_Bps * quantum * MAX_EPRESDIV) >= sourceClock_Hz);
#endif
    }
    else {
        assert((bitRate_Bps * quantum * MAX_PRESDIV) >= sourceClock_Hz);
    }
#else
    assert((bitRate_Bps * quantum * MAX_PRESDIV) >= sourceClock_Hz);
#endif
    if (quantum < (MIN_TIME_SEGMENT1 + MIN_TIME_SEGMENT2 + 1U)) {
        /* No valid timing configuration. */
        timingConfig.preDivider = 0U;
    }
    else {
        timingConfig.preDivider = (uint16_t)((sourceClock_Hz / (bitRate_Bps * quantum)) - 1U);
    }
    /* Update actual timing characteristic. */
    CAN_SetTimingConfig(base, (const can_timing_config_t *)(uint32_t)&timingConfig);
}


/*!
 * This function makes the CAN leave Freeze Mode.
 *
 * base: CAN peripheral base address.
 */
void CAN_ExitFreezeMode(CAN_Type *base) {
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    /* Clean FlexCAN Access With Non-Correctable Error Interrupt Flag to avoid be put in freeze mode. */
    FLEXCAN_ClearStatusFlags(base, (uint64_t)kFLEXCAN_FlexCanAccessNonCorrectableErrorIntFlag |
                                   (uint64_t)kFLEXCAN_FlexCanAccessNonCorrectableErrorOverrunFlag);
#endif
    /* Clear Freeze, Halt bits. */
    base->MCR &= ~CAN_MCR_HALT_MASK;
    base->MCR &= ~CAN_MCR_FRZ_MASK;
    /* Wait until the CAN Module exit freeze mode. */
    while (0U != (base->MCR & CAN_MCR_FRZACK_MASK)) { }
}


/*!
 * This function makes the CAN work under Freeze Mode.
 *
 * base: CAN peripheral base address.
 */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_9595) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_9595)
void CAN_EnterFreezeMode(CAN_Type *base) {
    uint32_t u32TimeoutCount = 0U;
    uint32_t u32TempMCR      = 0U;
    uint32_t u32TempIMASK1   = 0U;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    uint32_t u32TempIMASK2 = 0U;
#endif
    /* Step1: set FRZ enable in MCR. */
    base->MCR |= CAN_MCR_FRZ_MASK;
    /* Step2: to check if MDIS bit set in MCR. if yes, clear it. */
    if (0U != (base->MCR & CAN_MCR_MDIS_MASK)) {
        base->MCR &= ~CAN_MCR_MDIS_MASK;
    }
    /* Step3: polling LPMACK. */
    u32TimeoutCount = (uint32_t)FLEXCAN_WAIT_TIMEOUT;
    while ((0U == (base->MCR & CAN_MCR_LPMACK_MASK)) && (u32TimeoutCount > 0U)) {
        u32TimeoutCount--;
    }
    /* Step4: to check FLTCONF in ESR1 register */
    if (0U == (base->ESR1 & CAN_ESR1_FLTCONF_BUSOFF)) {
        /* Step5B: Set Halt bits. */
        base->MCR |= CAN_MCR_HALT_MASK;
        /* Step6B: Poll the MCR register until the Freeze Acknowledge (FRZACK) bit is set, timeout need more than 178
         * CAN bit length, so 20 multiply timeout is enough. */
        u32TimeoutCount = (uint32_t)FLEXCAN_WAIT_TIMEOUT * 20U;
        while ((0U == (base->MCR & CAN_MCR_FRZACK_MASK)) && (u32TimeoutCount > 0U)) {
            u32TimeoutCount--;
        }
    }
    else {
        /* backup MCR and IMASK register. Errata document not descript it, but we need backup for step 8A and 9A. */
        u32TempMCR    = base->MCR;
        u32TempIMASK1 = base->IMASK1;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        u32TempIMASK2 = base->IMASK2;
#endif
        /* Step5A: Set the Soft Reset bit ((SOFTRST) in the MCR.*/
        base->MCR |= CAN_MCR_SOFTRST_MASK;
        /* Step6A: Poll the MCR register until the Soft Reset (SOFTRST) bit is cleared. */
        u32TimeoutCount = (uint32_t)FLEXCAN_WAIT_TIMEOUT;
        while ((CAN_MCR_SOFTRST_MASK == (base->MCR & CAN_MCR_SOFTRST_MASK)) && (u32TimeoutCount > 0U))
        {
            u32TimeoutCount--;
        }
        /* Step7A: Poll the MCR register until the Freeze Acknowledge (FRZACK) bit is set. */
        u32TimeoutCount = (uint32_t)FLEXCAN_WAIT_TIMEOUT;
        while ((0U == (base->MCR & CAN_MCR_FRZACK_MASK)) && (u32TimeoutCount > 0U)) {
            u32TimeoutCount--;
        }
        /* Step8A: reconfig MCR. */
        base->MCR = u32TempMCR;
        /* Step9A: reconfig IMASK. */
        base->IMASK1 = u32TempIMASK1;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        base->IMASK2 = u32TempIMASK2;
#endif
    }
}
#elif (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_8341) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_8341)
void CAN_EnterFreezeMode(CAN_Type *base)
{
    uint32_t u32TimeoutCount = 0U;
    uint32_t u32TempMCR      = 0U;
    uint32_t u32TempIMASK1   = 0U;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
    uint32_t u32TempIMASK2   = 0U;
#endif
    /* Step1: set FRZ and HALT bit enable in MCR. */
    base->MCR |= CAN_MCR_FRZ_MASK;
    base->MCR |= CAN_MCR_HALT_MASK;
    /* Step2: to check if MDIS bit set in MCR. if yes, clear it. */
    if (0U != (base->MCR & CAN_MCR_MDIS_MASK)) {
        base->MCR &= ~CAN_MCR_MDIS_MASK;
    }
    /* Step3: Poll the MCR register until the Freeze Acknowledge (FRZACK) bit is set. */
    u32TimeoutCount = (uint32_t)FLEXCAN_WAIT_TIMEOUT * 100U;
    while ((0U == (base->MCR & CAN_MCR_FRZACK_MASK)) && (u32TimeoutCount > 0U)) {
        u32TimeoutCount--;
    }
    /* Step4: check whether the timeout reached. if no skip step5 to step8. */
    if (0U == u32TimeoutCount) {
        /* backup MCR and IMASK register. Errata document not descript it, but we need backup for step 8A and 9A. */
        u32TempMCR    = base->MCR;
        u32TempIMASK1 = base->IMASK1;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        u32TempIMASK2 = base->IMASK2;
#endif
        /* Step5: Set the Soft Reset bit ((SOFTRST) in the MCR.*/
        base->MCR |= CAN_MCR_SOFTRST_MASK;
        /* Step6: Poll the MCR register until the Soft Reset (SOFTRST) bit is cleared. */
        while (CAN_MCR_SOFTRST_MASK == (base->MCR & CAN_MCR_SOFTRST_MASK)) {}
        /* Step7: reconfig MCR. */
        base->MCR = u32TempMCR;
        /* Step8: reconfig IMASK. */
        base->IMASK1 = u32TempIMASK1;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER)) && (FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER > 0)
        base->IMASK2 = u32TempIMASK2;
#endif
    }
}
#else
void CAN_EnterFreezeMode(CAN_Type *base) {
    /* Set Freeze, Halt bits. */
    base->MCR |= CAN_MCR_FRZ_MASK;
    base->MCR |= CAN_MCR_HALT_MASK;
    while (0U == (base->MCR & CAN_MCR_FRZACK_MASK)) { }
}
#endif


/*!
 * This function gives user settings to CAN nominal phase timing characteristic.
 *
 * base: CAN peripheral base address.
 * pConfig: Pointer to the timing configuration structure.
 */
void CAN_SetTimingConfig(CAN_Type *base, const can_timing_config_t *pConfig) {
    /* Assertion. */
    assert(NULL != pConfig);
    /* Enter Freeze Mode. */
    CAN_EnterFreezeMode(base);
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    if (0 != FSL_FEATURE_FLEXCAN_INSTANCE_HAS_FLEXIBLE_DATA_RATEn(base))
    {
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
        /* Enable extended Bit Timing register ENCBT. */
        base->CTRL2 |= CAN_CTRL2_BTE_MASK;

        /* Updating Timing Setting according to configuration structure. */
        base->EPRS  = (base->EPRS & (~CAN_EPRS_ENPRESDIV_MASK)) | CAN_EPRS_ENPRESDIV(pConfig->preDivider);
        base->ENCBT = CAN_ENCBT_NRJW(pConfig->rJumpwidth) |
                      CAN_ENCBT_NTSEG1(pConfig->phaseSeg1 + pConfig->propSeg + 1U) |
                      CAN_ENCBT_NTSEG2(pConfig->phaseSeg2);
#else
        /* Enable Bit Timing register CBT, updating Timing Setting according to configuration structure. */
        base->CBT = CAN_CBT_BTF_MASK | CAN_CBT_EPRESDIV(pConfig->preDivider) | CAN_CBT_ERJW(pConfig->rJumpwidth) |
                    CAN_CBT_EPSEG1(pConfig->phaseSeg1) | CAN_CBT_EPSEG2(pConfig->phaseSeg2) |
                    CAN_CBT_EPROPSEG(pConfig->propSeg);
#endif
    }
    else {
        /* Cleaning previous Timing Setting. */
        base->CTRL1 &= ~(CAN_CTRL1_PRESDIV_MASK | CAN_CTRL1_RJW_MASK | CAN_CTRL1_PSEG1_MASK | CAN_CTRL1_PSEG2_MASK |
                         CAN_CTRL1_PROPSEG_MASK);

        /* Updating Timing Setting according to configuration structure. */
        base->CTRL1 |= (CAN_CTRL1_PRESDIV(pConfig->preDivider) | CAN_CTRL1_RJW(pConfig->rJumpwidth) |
                        CAN_CTRL1_PSEG1(pConfig->phaseSeg1) | CAN_CTRL1_PSEG2(pConfig->phaseSeg2) |
                        CAN_CTRL1_PROPSEG(pConfig->propSeg));
    }
#else
    /* Cleaning previous Timing Setting. */
    base->CTRL1 &= ~(CAN_CTRL1_PRESDIV_MASK | CAN_CTRL1_RJW_MASK | CAN_CTRL1_PSEG1_MASK | CAN_CTRL1_PSEG2_MASK |
                     CAN_CTRL1_PROPSEG_MASK);
    /* Updating Timing Setting according to configuration structure. */
    base->CTRL1 |= (CAN_CTRL1_PRESDIV(pConfig->preDivider) | CAN_CTRL1_RJW(pConfig->rJumpwidth) |
                    CAN_CTRL1_PSEG1(pConfig->phaseSeg1) | CAN_CTRL1_PSEG2(pConfig->phaseSeg2) |
                    CAN_CTRL1_PROPSEG(pConfig->propSeg));
#endif
    /* Exit Freeze Mode. */
    CAN_ExitFreezeMode(base);
}


/*!
 * Restores the CAN module to reset state, notice that this function
 * will set all the registers to reset state so the CAN module can not work
 * after calling this API.
 *
 * base: CAN peripheral base address.
 */
static void CAN_Reset(CAN_Type *base) {
    /* The module must should be first exit from low power mode, and then soft reset can be applied. */
    assert(0U == (base->MCR & CAN_MCR_MDIS_MASK));
    uint8_t i;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT) && FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT)
    if (0 != (FSL_FEATURE_FLEXCAN_INSTANCE_HAS_DOZE_MODE_SUPPORTn(base))) {
        /* De-assert DOZE Enable Bit. */
        base->MCR &= ~CAN_MCR_DOZE_MASK;
    }
#endif
    /* Wait until CAN exit from any Low Power Mode. */
    while (0U != (base->MCR & CAN_MCR_LPMACK_MASK)) { }
    /* Assert Soft Reset Signal. */
    base->MCR |= CAN_MCR_SOFTRST_MASK;
    /* Wait until CAN reset completes. */
    while (0U != (base->MCR & CAN_MCR_SOFTRST_MASK)) { }
/* Reset MCR register. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_GLITCH_FILTER) && FSL_FEATURE_FLEXCAN_HAS_GLITCH_FILTER)
    base->MCR |= CAN_MCR_WRNEN_MASK | CAN_MCR_WAKSRC_MASK |
                 CAN_MCR_MAXMB((uint32_t)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base) - 1U);
#else
    base->MCR |=
        CAN_MCR_WRNEN_MASK | CAN_MCR_MAXMB((uint32_t)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base) - 1U);
#endif
/* Reset CTRL1 and CTRL2 register. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    /* SMP bit cannot be asserted when CAN FD is enabled */
    if (0 != FSL_FEATURE_FLEXCAN_INSTANCE_HAS_FLEXIBLE_DATA_RATEn(base)) {
        base->CTRL1 = 0x0;
    }
    else {
        base->CTRL1 = CAN_CTRL1_SMP_MASK;
    }
#else
    base->CTRL1 = CAN_CTRL1_SMP_MASK;
#endif
    base->CTRL2 = CAN_CTRL2_TASD(0x16) | CAN_CTRL2_RRS_MASK | CAN_CTRL2_EACEN_MASK;
    base->CTRL1 = CAN_CTRL1_SMP_MASK;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
    /* Clean all Wake Up Message Buffer memory. */
    (void)memset((void *)&base->WMB[0], 0, sizeof(base->WMB));
#endif
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    /* Enable unrestricted write access to FlexCAN memory. */
    base->CTRL2 |= CAN_CTRL2_WRMFRZ_MASK;
    /* Do memory initialization for all FlexCAN RAM in order to have the parity bits in memory properly
       updated. */
    *(volatile uint32_t *)CAN_INIT_RXFIR = 0x0U;
    (void)memset((void *)CAN_INIT_MEMORY_BASE_1, 0, CAN_INIT_MEMORY_SIZE_1);
    (void)memset((void *)CAN_INIT_MEMORY_BASE_2, 0, CAN_INIT_MEMORY_SIZE_2);
    /* Disable unrestricted write access to FlexCAN memory. */
    base->CTRL2 &= ~CAN_CTRL2_WRMFRZ_MASK;
    /* Clean all memory error flags. */
    FLEXCAN_ClearStatusFlags(base, (uint64_t)kFLEXCAN_AllMemoryErrorFlag);
#else
    /* Only need clean all Message Buffer memory. */
    (void)memset((void *)&base->MB[0], 0, sizeof(base->MB));
#endif
    /* Clean all individual Rx Mask of Message Buffers. */
    for (i = 0; i < (uint32_t)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base); i++) {
        base->RXIMR[i] = 0x3FFFFFFF;
    }
    /* Clean Global Mask of Message Buffers. */
    base->RXMGMASK = 0x3FFFFFFF;
    /* Clean Global Mask of Message Buffer 14. */
    base->RX14MASK = 0x3FFFFFFF;
    /* Clean Global Mask of Message Buffer 15. */
    base->RX15MASK = 0x3FFFFFFF;
    /* Clean Global Mask of Rx FIFO. */
    base->RXFGMASK = 0x3FFFFFFF;
}


#if !defined(NDEBUG)
/*!
 * This function check if Message Buffer is occupied by Rx FIFO.
 *
 * param base CAN peripheral base address.
 * param mbIdx CAN Message Buffer index.
 * return TRUE if the index MB is occupied by Rx FIFO, FALSE if the index MB not occupied by Rx FIFO.
 */
static bool CAN_IsMbOccupied(CAN_Type *base, uint8_t mbIdx) {
    uint8_t lastOccupiedMb;
    bool fgRet;
    /* Is Rx FIFO enabled? */
    if (0U != (base->MCR & CAN_MCR_RFEN_MASK)) {
        /* Get RFFN value. */
        lastOccupiedMb = (uint8_t)((base->CTRL2 & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
        /* Calculate the number of last Message Buffer occupied by Rx FIFO. */
        lastOccupiedMb = ((lastOccupiedMb + 1U) * 2U) + 5U;

#if ((defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) || \
     (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829))
        /* the first valid MB should be occupied by ERRATA 5461 or 5829. */
        lastOccupiedMb += 1U;
#endif
        fgRet = (mbIdx <= lastOccupiedMb);
    }
    else {
#if ((defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) || \
     (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829))
        if (0U == mbIdx)
        {
            fgRet = true;
        }
        else
#endif
        {
            fgRet = false;
        }
    }
    return fgRet;
}
#endif


/*!
 * This function cleans a CAN build-in Message Buffer and configures it
 * as a Receive Message Buffer.
 *
 * param base CAN peripheral base address.
 * param mbIdx The Message Buffer index.
 * param pRxMbConfig Pointer to the CAN Message Buffer configuration structure.
 * param enable Enable/disable Rx Message Buffer.
 */
void CAN_SetRxMbConfig(CAN_Type *base, uint8_t mbIdx, const can_rx_mb_config_t *pRxMbConfig, bool enable) {
    /* Assertion. */
    assert(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
    assert(((NULL != pRxMbConfig) || (false == enable)));
#if !defined(NDEBUG)
    assert(!CAN_IsMbOccupied(base, mbIdx));
#endif
    uint32_t cs_temp = 0;
    /* Deactivate Message Buffer. */
    base->MB[mbIdx].CS = 0;
    /* Clean Message Buffer content. */
    base->MB[mbIdx].ID    = 0x0;
    base->MB[mbIdx].WORD0 = 0x0;
    base->MB[mbIdx].WORD1 = 0x0;
    if (enable) {
        /* Setup Message Buffer ID. */
        base->MB[mbIdx].ID = pRxMbConfig->id;
        /* Setup Message Buffer format. */
        if (kCAN_FrameFormatExtend == pRxMbConfig->format) {
            cs_temp |= CAN_CS_IDE_MASK;
        }
        /* Setup Message Buffer type. */
        if (kCAN_FrameTypeRemote == pRxMbConfig->type) {
            cs_temp |= CAN_CS_RTR_MASK;
        }
        /* Activate Rx Message Buffer. */
        cs_temp |= CAN_CS_CODE(kCAN_RxMbEmpty);
        base->MB[mbIdx].CS = cs_temp;
    }
}


enum _can_mb_code_tx {
    kCAN_TxMbInactive     = 0x8, /*!< MB is not active.*/
    kCAN_TxMbAbort        = 0x9, /*!< MB is aborted.*/
    kCAN_TxMbDataOrRemote = 0xC, /*!< MB is a TX Data Frame(when MB RTR = 0) or MB is a TX Remote Request
                                      Frame (when MB RTR = 1).*/
    kCAN_TxMbTanswer = 0xE,      /*!< MB is a TX Response Request Frame from an incoming Remote Request Frame.*/
    kCAN_TxMbNotUsed = 0xF,      /*!< Not used.*/
};


/*!
 * This function aborts the previous transmission, cleans the Message Buffer, and
 * configures it as a Transmit Message Buffer.
 *
 * param base CAN peripheral base address.
 * param mbIdx The Message Buffer index.
 * param enable Enable/disable Tx Message Buffer.
 *  - true: Enable Tx Message Buffer.
 *  - false: Disable Tx Message Buffer.
 */
void CAN_SetTxMbConfig(CAN_Type *base, uint8_t mbIdx, bool enable) {
    /* Assertion. */
    assert(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
#if !defined(NDEBUG)
    assert(!CAN_IsMbOccupied(base, mbIdx));
#endif
    /* Deactivate Message Buffer. */
    if (enable) {
        base->MB[mbIdx].CS = CAN_CS_CODE(kCAN_TxMbInactive);
    }
    else {
        base->MB[mbIdx].CS = 0;
    }
    /* Clean Message Buffer content. */
    base->MB[mbIdx].ID    = 0x0;
    base->MB[mbIdx].WORD0 = 0x0;
    base->MB[mbIdx].WORD1 = 0x0;
}


/*!
 * Writes a CAN Message to the Transmit Message Buffer.
 *
 * This function writes a CAN Message to the specified Transmit Message Buffer
 * and changes the Message Buffer state to start CAN Message transmit. After
 * that the function returns immediately.
 *
 * param base CAN peripheral base address.
 * param mbIdx The CAN Message Buffer index.
 * param pTxFrame Pointer to CAN message frame to be sent.
 * retval kStatus_Success - Write Tx Message Buffer Successfully.
 * retval kStatus_Fail - Tx Message Buffer is currently in use.
 */
status_t CAN_Write(CAN_Type *base, uint8_t mbIdx, const Can_PduType *pTxFrame) {
    /* Assertion. */
    assert(mbIdx <= (base->MCR & CAN_MCR_MAXMB_MASK));
    assert(NULL != pTxFrame);
    assert(pTxFrame->length <= 8U);
#if !defined(NDEBUG)
    assert(!CAN_IsMbOccupied(base, mbIdx));
#endif
    uint32_t cs_temp = 0;
    status_t status;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_6032) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_6032)
    FLEXCAN_ERRATA_6032(base, &(base->MB[mbIdx].CS));
#endif
    /* Check if Message Buffer is available. */
    if (CAN_CS_CODE(kCAN_TxMbDataOrRemote) != (base->MB[mbIdx].CS & CAN_CS_CODE_MASK)) {
        /* Inactive Tx Message Buffer. */
        base->MB[mbIdx].CS = (base->MB[mbIdx].CS & ~CAN_CS_CODE_MASK) | CAN_CS_CODE(kCAN_TxMbInactive);
        /* Fill Message ID field. */
        base->MB[mbIdx].ID = pTxFrame->id;
        /* Fill Message Format field. */
        if ((uint32_t)kCAN_FrameFormatExtend == pTxFrame->format) {
            cs_temp |= CAN_CS_SRR_MASK | CAN_CS_IDE_MASK;
        }
        /* Fill Message Type field. */
        if ((uint32_t)kCAN_FrameTypeRemote == pTxFrame->type) {
            cs_temp |= CAN_CS_RTR_MASK;
        }
        cs_temp |= CAN_CS_CODE(kCAN_TxMbDataOrRemote) | CAN_CS_DLC(pTxFrame->length);
        /* Load Message Payload. */
        base->MB[mbIdx].WORD0 = pTxFrame->dataWord0;
        base->MB[mbIdx].WORD1 = pTxFrame->dataWord1;
        /* Activate Tx Message Buffer. */
        base->MB[mbIdx].CS = cs_temp;
#if ((defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641) || \
     (defined(FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829) && FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829))
        base->MB[FLEXCAN_GetFirstValidMb(base)].CS = CAN_CS_CODE(kFLEXCAN_TxMbInactive);
        base->MB[FLEXCAN_GetFirstValidMb(base)].CS = CAN_CS_CODE(kFLEXCAN_TxMbInactive);
#endif
        status = kStatus_Success;
    }
    else {
        /* Tx Message Buffer is activated, return immediately. */
        status = kStatus_Fail;
    }
    return status;
}


/*!
 * This function calculates the CAN timing values according to the given bit rate. The calculated
 * timing values will be set in CTRL1/CBT/ENCBT register. The calculation is based on the recommendation
 * of the CiA 301 v4.2.0 and previous version document.
 *
 * param base CAN peripheral base address.
 * param bitRate  The classical CAN speed in bps defined by user, should be less than or equal to 1Mbps.
 * param sourceClock_Hz The Source clock frequency in Hz.
 * param pTimingConfig Pointer to the CAN timing configuration structure.
 *
 * return TRUE if timing configuration found, FALSE if failed to find configuration.
 */
bool CAN_CalculateImprovedTimingValues(CAN_Type *base, uint32_t bitRate, uint32_t sourceClock_Hz,
                                       can_timing_config_t *pTimingConfig) {
    /* Observe bit rate maximums. */
    assert(bitRate <= MAX_CAN_BITRATE);
    uint32_t clk;
    uint32_t tqNum, tqMin, pdivMAX;
    uint32_t spTemp = 1000U;
    can_timing_config_t configTemp = {0};
    bool fgRet = false;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    if (0 != FSL_FEATURE_FLEXCAN_INSTANCE_HAS_FLEXIBLE_DATA_RATEn(base)) {
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
        /*  Auto Improved Protocal timing for ENCBT. */
        tqNum   = ENCBT_MAX_TIME_QUANTA;
        tqMin   = ENCBT_MIN_TIME_QUANTA;
        pdivMAX = MAX_ENPRESDIV;
#else
        /*  Auto Improved Protocal timing for CBT. */
        tqNum = CBT_MAX_TIME_QUANTA;
        tqMin = CBT_MIN_TIME_QUANTA;
        pdivMAX = MAX_PRESDIV;
#endif
    }
    else {
        /*  Auto Improved Protocal timing for CTRL1. */
        tqNum   = CTRL1_MAX_TIME_QUANTA;
        tqMin   = CTRL1_MIN_TIME_QUANTA;
        pdivMAX = MAX_PRESDIV;
    }
#else
    /*  Auto Improved Protocal timing for CTRL1. */
    tqNum = CTRL1_MAX_TIME_QUANTA;
    tqMin = CTRL1_MIN_TIME_QUANTA;
    pdivMAX = MAX_PRESDIV;
#endif
    do {
        clk = bitRate * tqNum;
        if (clk > sourceClock_Hz) {
            continue; /* tqNum too large, clk has been exceed sourceClock_Hz. */
        }
        if ((sourceClock_Hz / clk * clk) != sourceClock_Hz) {
            continue; /* Non-supporting: the frequency of clock source is not divisible by target bit rate, the user
                      should change a divisible bit rate. */
        }
        configTemp.preDivider = (uint16_t)(sourceClock_Hz / clk) - 1U;
        if (configTemp.preDivider > pdivMAX) {
            break; /* The frequency of source clock is too large or the bit rate is too small, the pre-divider could
                      not handle it. */
        }
        /* Calculates the best timing configuration under current tqNum. */
        CAN_GetSegments(base, bitRate, tqNum, &configTemp);
        /* Determine whether the calculated timing configuration can get the optimal sampling point. */
        if (((((uint32_t)configTemp.phaseSeg2 + 1U) * 1000U) / tqNum) < spTemp) {
            spTemp = (((uint32_t)configTemp.phaseSeg2 + 1U) * 1000U) / tqNum;
            (void)memcpy(pTimingConfig, &configTemp, sizeof(configTemp));
        }
        fgRet = true;
    } while (--tqNum >= tqMin);
    return fgRet;
}


/*!
 * This function calculates the CAN segment values which will be set in CTRL1/CBT/ENCBT register.
 *
 * param bitRate The classical CAN bit rate in bps.
 * param base CAN peripheral base address.
 * param tqNum Number of time quantas per bit, range in 8 ~ 25 when use CTRL1, range in 8 ~ 129 when use CBT, range in
 *             8 ~ 385 when use ENCBT. param pTimingConfig Pointer to the CAN timing configuration structure.
 */
static void CAN_GetSegments(CAN_Type *base, uint32_t bitRate, uint32_t tqNum,
                            can_timing_config_t *pTimingConfig) {
    uint32_t ideal_sp;
    uint32_t seg1Max, proSegMax, sjwMAX;
    uint32_t seg1Temp;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    if (0 != FSL_FEATURE_FLEXCAN_INSTANCE_HAS_FLEXIBLE_DATA_RATEn(base)) {
#if (defined(FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG) && FSL_FEATURE_FLEXCAN_HAS_ENHANCED_BIT_TIMING_REG)
        /* Maximum value allowed in ENCBT register. */
        seg1Max   = MAX_NTSEG2 + 1U;
        proSegMax = MAX_NTSEG1 - MAX_NTSEG2;
        sjwMAX    = MAX_NRJW + 1U;
#else
        /* Maximum value allowed in CBT register. */
        seg1Max = MAX_EPSEG1 + 1U;
        proSegMax = MAX_EPROPSEG + 1U;
        sjwMAX = MAX_ERJW + 1U;
#endif
    }
    else {
        /* Maximum value allowed in CTRL1 register. */
        seg1Max   = MAX_PSEG1 + 1U;
        proSegMax = MAX_PROPSEG + 1U;
        sjwMAX    = MAX_RJW + 1U;
    }
#else
    /* Maximum value allowed in CTRL1 register. */
    seg1Max   = MAX_PSEG1 + 1U;
    proSegMax = MAX_PROPSEG + 1U;
    sjwMAX    = MAX_RJW + 1U;
#endif
    /* Try to find the ideal sample point, according to CiA 301 doc.*/
    if (bitRate == 1000000U) {
        ideal_sp = IDEAL_SP_LOW;
    }
    else if (bitRate >= 800000U) {
        ideal_sp = IDEAL_SP_MID;
    }
    else {
        ideal_sp = IDEAL_SP_HIGH;
    }
    /* Calculates phaseSeg2. */
    pTimingConfig->phaseSeg2 = (uint8_t)(tqNum - (tqNum * ideal_sp) / (uint32_t)IDEAL_SP_FACTOR);
    if (pTimingConfig->phaseSeg2 < MIN_TIME_SEGMENT2) {
        pTimingConfig->phaseSeg2 = MIN_TIME_SEGMENT2;
    }
    else if (pTimingConfig->phaseSeg2 > seg1Max) {
        pTimingConfig->phaseSeg2 = (uint8_t)seg1Max;
    }
    else {
        ; /* Intentional empty */
    }
    /* Calculates phaseSeg1 and propSeg. */
    seg1Temp = tqNum - pTimingConfig->phaseSeg2 - 1U;
    if (seg1Temp > (seg1Max + proSegMax)) {
        pTimingConfig->phaseSeg2 += (uint8_t)(seg1Temp - seg1Max - proSegMax);
        pTimingConfig->propSeg   = (uint8_t)proSegMax;
        pTimingConfig->phaseSeg1 = (uint8_t)seg1Max;
    }
    else if (seg1Temp > proSegMax) {
        pTimingConfig->propSeg   = (uint8_t)proSegMax;
        pTimingConfig->phaseSeg1 = (uint8_t)(seg1Temp - proSegMax);
    }
    else {
        pTimingConfig->propSeg   = (uint8_t)(seg1Temp - 1U);
        pTimingConfig->phaseSeg1 = 1U;
    }
    /* try to make phaseSeg1 equal to phaseSeg2*/
    if (pTimingConfig->phaseSeg1 < pTimingConfig->phaseSeg2) {
        seg1Temp =
            ((pTimingConfig->phaseSeg2 - pTimingConfig->phaseSeg1) > ((uint8_t)proSegMax - pTimingConfig->propSeg)) ?
                (proSegMax - pTimingConfig->propSeg) :
                (pTimingConfig->phaseSeg2 - pTimingConfig->phaseSeg1);
        pTimingConfig->propSeg -= (uint8_t)seg1Temp;
        pTimingConfig->phaseSeg1 += (uint8_t)seg1Temp;
    }
    else {
        seg1Temp =
            ((pTimingConfig->phaseSeg1 - pTimingConfig->phaseSeg2) > ((uint8_t)proSegMax - pTimingConfig->propSeg)) ?
                (proSegMax - pTimingConfig->propSeg) :
                (pTimingConfig->phaseSeg1 - pTimingConfig->phaseSeg2);
        pTimingConfig->propSeg += (uint8_t)seg1Temp;
        pTimingConfig->phaseSeg1 -= (uint8_t)seg1Temp;
    }
    /* rJumpwidth (sjw) is the minimum value of phaseSeg1 and phaseSeg2. */
    pTimingConfig->rJumpwidth =
        (pTimingConfig->phaseSeg1 > pTimingConfig->phaseSeg2) ? pTimingConfig->phaseSeg2 : pTimingConfig->phaseSeg1;
    if (pTimingConfig->rJumpwidth > sjwMAX) {
        pTimingConfig->rJumpwidth = (uint8_t)sjwMAX;
    }
    pTimingConfig->phaseSeg1 -= 1U;
    pTimingConfig->phaseSeg2 -= 1U;
    pTimingConfig->propSeg -= 1U;
    pTimingConfig->rJumpwidth -= 1U;
}


void Can_Init(const Can_ConfigType *pConfig) {
#if (defined(USE_IMPROVED_TIMING_CONFIG) && USE_IMPROVED_TIMING_CONFIG)
    can_timing_config_t timing_config;
    memset(&timing_config, 0, sizeof(can_timing_config_t));
    if (CAN_CalculateImprovedTimingValues(EXAMPLE_CAN, pConfig->baudRate, EXAMPLE_CAN_CLK_FREQ, &timing_config)) {
		/* Update the improved timing configuration*/
		memcpy(&(pConfig->timingConfig), &timing_config, sizeof(can_timing_config_t));
	}
#endif
    /* Assertion. */
    assert(NULL != pConfig);
    assert((pConfig->maxMbNum > 0U) &&
           (pConfig->maxMbNum <= (uint8_t)FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(base)));
    assert(pConfig->bitRate > 0U);
    uint32_t mcrTemp;
    uint32_t ctrl1Temp;
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    uint32_t instance;
#endif
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    instance = CAN_GetInstance(pConfig->CAN_base);
    /* Enable CAN clock. */
    (void)CLOCK_EnableClock(s_flexcanClock[instance]);
    /*
     * Check the CAN clock in this device whether affected by Other clock gate
     * If it affected, we'd better to change other clock source,
     * If user insist on using that clock source, user need open these gate at same time,
     * In this scene, User need to care the power consumption.
     */
    assert(CAN_CLOCK_CHECK_NO_AFFECTS);
#if defined(FLEXCAN_PERIPH_CLOCKS)
    /* Enable FlexCAN serial clock. */
    (void)CLOCK_EnableClock(s_flexcanPeriphClock[instance]);
#endif /* FLEXCAN_PERIPH_CLOCKS */
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
#if defined(CAN_CTRL1_CLKSRC_MASK)
#if (defined(FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE) && FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE)
    if (0 == FSL_FEATURE_FLEXCAN_INSTANCE_SUPPORT_ENGINE_CLK_SEL_REMOVEn(base))
#endif /* FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE */
    {
        /* Disable FlexCAN Module. */
    	CAN_Enable(pConfig->CAN_base, false);
        /* Protocol-Engine clock source selection, This bit must be set when CAN module in Disable Mode. */
    	pConfig->CAN_base->CTRL1 = (
    		kFLEXCAN_ClkSrc0_autosar == pConfig->clkSrc
		) ? (pConfig->CAN_base->CTRL1 & ~CAN_CTRL1_CLKSRC_MASK) :
            (pConfig->CAN_base->CTRL1 | CAN_CTRL1_CLKSRC_MASK);
    }
#endif /* CAN_CTRL1_CLKSRC_MASK */
    CAN_Enable(pConfig->CAN_base, true);  /* Enable CAN Module for configuration. */
    CAN_Reset(pConfig->CAN_base);  /* Reset to known status. */
#if (defined(FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL) && FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL)
    /* Enable to update in MCER. */
    base->CTRL2 |= CAN_CTRL2_ECRWRE_MASK;
    base->MECR &= ~CAN_MECR_ECRWRDIS_MASK;
    /* Enable/Disable Memory Error Detection and Correction.*/
    base->MECR = (pConfig->enableMemoryErrorControl) ? (base->MECR & ~CAN_MECR_ECCDIS_MASK) :
                                                       (base->MECR | CAN_MECR_ECCDIS_MASK);
    /* Enable/Disable Non-Correctable Errors In FlexCAN Access Put Device In Freeze Mode. */
    base->MECR = (pConfig->enableNonCorrectableErrorEnterFreeze) ? (base->MECR | CAN_MECR_NCEFAFRZ_MASK) :
                                                                   (base->MECR & ~CAN_MECR_NCEFAFRZ_MASK);
    /* Lock MCER register. */
    base->CTRL2 &= ~CAN_CTRL2_ECRWRE_MASK;
#endif
    /* Save current CTRL1 value and enable to enter Freeze mode(enabled by default). */
    ctrl1Temp = pConfig->CAN_base->CTRL1;
    /* Save current MCR value and enable to enter Freeze mode(enabled by default). */
    mcrTemp = pConfig->CAN_base->MCR;
    /* Enable Loop Back Mode? */
    ctrl1Temp = (pConfig->enableLoopBack) ? (ctrl1Temp | CAN_CTRL1_LPB_MASK) : (ctrl1Temp & ~CAN_CTRL1_LPB_MASK);
    /* Enable Timer Sync? */
    ctrl1Temp = (pConfig->enableTimerSync) ? (ctrl1Temp | CAN_CTRL1_TSYN_MASK) : (ctrl1Temp & ~CAN_CTRL1_TSYN_MASK);
    /* Enable Listen Only Mode? */
    ctrl1Temp = (pConfig->enableListenOnlyMode) ? ctrl1Temp | CAN_CTRL1_LOM_MASK : ctrl1Temp & ~CAN_CTRL1_LOM_MASK;
    /* Set the maximum number of Message Buffers */
    mcrTemp = (mcrTemp & ~CAN_MCR_MAXMB_MASK) | CAN_MCR_MAXMB((uint32_t)pConfig->maxMbNum - 1U);
    /* Enable Self Wake Up Mode and configure the wake up source. */
    mcrTemp = (pConfig->enableSelfWakeup) ? (mcrTemp | CAN_MCR_SLFWAK_MASK) : (mcrTemp & ~CAN_MCR_SLFWAK_MASK);
    mcrTemp = (kFLEXCAN_WakeupSrcFiltered_autosar == pConfig->wakeupSrc) ? (mcrTemp | CAN_MCR_WAKSRC_MASK) :
                                                                           (mcrTemp & ~CAN_MCR_WAKSRC_MASK);
#if (defined(FSL_FEATURE_FLEXCAN_HAS_PN_MODE) && FSL_FEATURE_FLEXCAN_HAS_PN_MODE)
    /* Enable Pretended Networking Mode? When Pretended Networking mode is set, Self Wake Up feature must be disabled.*/
    mcrTemp = (pConfig->enablePretendedeNetworking) ? ((mcrTemp & ~CAN_MCR_SLFWAK_MASK) | CAN_MCR_PNET_EN_MASK) :
                                                      (mcrTemp & ~CAN_MCR_PNET_EN_MASK);
#endif
    /* Enable Individual Rx Masking and Queue feature? */
    mcrTemp = (pConfig->enableIndividMask) ? (mcrTemp | CAN_MCR_IRMQ_MASK) : (mcrTemp & ~CAN_MCR_IRMQ_MASK);
    /* Disable Self Reception? */
    mcrTemp = (pConfig->disableSelfReception) ? mcrTemp | CAN_MCR_SRXDIS_MASK : mcrTemp & ~CAN_MCR_SRXDIS_MASK;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT) && FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT)
    if (0 != FSL_FEATURE_FLEXCAN_INSTANCE_HAS_DOZE_MODE_SUPPORTn(base)) {
        /* Enable Doze Mode? */
        mcrTemp = (pConfig->enableDoze) ? (mcrTemp | CAN_MCR_DOZE_MASK) : (mcrTemp & ~CAN_MCR_DOZE_MASK);
    }
#endif
    /* Write back CTRL1 Configuration to register. */
    pConfig->CAN_base->CTRL1 = ctrl1Temp;
    /* Write back MCR Configuration to register. */
    pConfig->CAN_base->MCR = mcrTemp;
    /* Bit Rate Configuration.*/
    CAN_SetBitRate(pConfig->CAN_base, pConfig->CAN_clock_freq(kCLOCK_BusClk), pConfig->bitRate, pConfig->timingConfig);
}

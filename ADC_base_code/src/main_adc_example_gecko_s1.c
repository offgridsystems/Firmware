/**************************************************************************//**
 * @file
 * @brief ADC Examples for EFM32 Gecko Series 1
 * @version 1.1.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include <stdio.h>
#include "em_adc.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_ldma.h"
#include "em_prs.h"
#include "em_rtcc.h"

#include "bspconfig.h"
#include "display.h"
#include "textdisplay.h"
#include "retargettextdisplay.h"

/* Definess for ADC */
#define ADC_CLOCK               1000000                 /* ADC conversion clock */
#define ADC_ASYNC_CLOCK         cmuAUXHFRCOFreq_4M0Hz   /* Clock for ASYNC mode */
#define ADC_INPUT0              adcPosSelAPORT3XCH8     /* PA0 */
#define ADC_INPUT1              adcPosSelAPORT3YCH9     /* PA1 */
#define ADC_DIFF_INPUT1         adcNegSelAPORT3YCH9     /* PA1 negative */
#define ADC_INPUT2              adcPosSelAPORT3XCH2     /* PD10 */
#define ADC_INPUT3              adcPosSelAPORT3YCH3     /* PD11 */
#define ADC_CAL_INPUT           adcPosSelAPORT3XCH8     /* PA0 */
#define ADC_NEG_OFFSET_VALUE    0xfff0                  /* Negative offset calibration */
#define ADC_GAIN_CAL_VALUE      0xffd0                  /* Gain calibration */
#define ADC_PRS_CH_SELECT       adcPRSSELCh0
#define ADC_SINGLE_DVL          4
#define ADC_SCAN_DVL            4
#define ADC_SCAN_DIFF_DVL       2
#define ADC_BUFFER_SIZE         64
#define ADC_VIN_ATT             9                       /* VIN attenuation factor */
#define ADC_VREF_ATT            0                       /* VREF attenuation factor */
#define ADC_SE_VFS              (float)3.3              /* AVDD */
#define ADC_DIFF_VFS            (float)6.6              /* 2xAVDD */
#define ADC_SCALE_VFS           (float)2.2              /* Scaled VFS */
#define ADC_12BIT_MAX           4096                    /* 2^12 */
#define ADC_16BIT_MAX           65536                   /* 2^16 */
#define ADC_CMP_GT_VALUE        3724                    /* ~3.0V for 3.3V AVDD */                   
#define ADC_CMP_LT_VALUE        620                     /* ~0.5V for 3.3V AVDD */
#define ADC_DMA_CHANNEL         0
#define ADC_DMA_CH_MASK         (1 << ADC_DMA_CHANNEL)

/* Defines for RTCC */
#define RTCC_CC_CHANNEL         1
#define RTCC_PRS_CHANNEL        0                       /* =ADC_PRS_CH_SELECT */ 
#define RTCC_PRS_CH_SELECT      rtccPRSCh0              /* =ADC_PRS_CH_SELECT */
#define RTCC_WAKEUP_MS          10
#define RTCC_WAKEUP_COUNT       (((32768 * RTCC_WAKEUP_MS) / 1000) - 1)

/* Menu items */
#define SING_SCAN               0
#define SING_SCAN_OVS           1
#define SING_SCAN_DIFF          2
#define SING_INT_EM1            3
#define SING_INT_EM2            4
#define SCAN_INT_EM1            5
#define SCAN_INT_EM2            6
#define SING_LDMA_EM1           7
#define SING_LDMA_EM2           8
#define SCAN_LDMA_EM1           9
#define SCAN_LDMA_EM2           10
#define SING_VFS                11
#define SING_PRNG               12
#define SING_CAL                13
#define MENU_MAX                14

/* Display device handle */
DISPLAY_Device_t displayDevice;

/* Menu and key operation */
volatile bool runKey;
volatile bool menuKey;
volatile uint8_t menuLevel;

/* Buffer for ADC interrupt flag */
volatile uint32_t adcIntFlag;

/* Buffer for ADC single and scan conversion */
uint32_t adcBuffer[ADC_BUFFER_SIZE];

/* Descriptor for LDMA transfer */
LDMA_Descriptor_t descLink0;

/**********************************************************
 * @brief Interrupt handler for push button BTN1. 
 **********************************************************/
void GPIO_ODD_IRQHandler(void)
{
  GPIO->IFC = GPIO->IF;

  if (!menuKey)
  {
    menuKey = true;
    if (++menuLevel == MENU_MAX)
    {
      menuLevel = 0;
    }
  }
}

/**********************************************************
 * @brief Interrupt handler for push button BTN0. 
 **********************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  GPIO->IFC = GPIO->IF;
  runKey = true;
}

/**************************************************************************//**
 * @brief ADC0 IRQ handler.
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  /* Single or scan DVL trigger */
  if ((ADC0->IEN & ADC_IEN_SINGLE) && (ADC0->IF & ADC_IF_SINGLE))
  {
    /* Read SINGLEDATA will clear SINGLE IF flag */
    adcBuffer[0] = ADC_DataSingleGet(ADC0);
    adcBuffer[1] = ADC_DataSingleGet(ADC0);
    adcBuffer[2] = ADC_DataSingleGet(ADC0);
    adcBuffer[3] = ADC_DataSingleGet(ADC0);
  }

  if ((ADC0->IEN & ADC_IEN_SCAN) && (ADC0->IF & ADC_IF_SCAN))
  {
    /* Read SCANDATA will clear SCAN IF flag */
    adcBuffer[0] = ADC0->SCANDATAX;
    adcBuffer[1] = ADC0->SCANDATAX;
    adcBuffer[2] = ADC0->SCANDATAX;
    adcBuffer[3] = ADC0->SCANDATAX;
  }
  
  /* SINGLECMP or SCANCMP trigger */
  /* The SCANCMP and SINGLECMP interrupt flags in ADCn_IF are not clearable */
  /* in certain scenarios (Errata ADC_E208). */
  /* Workaround is to clear CMPEN before clearing the SINGLECMP IF flag */
  /* but SINGLECTRL register can only be accessed on ADC SYNC mode. */
  /* Alternative is to disable ADC interrupt to avoid multiple SCANCMP or */
  /* SINGLECMP interrupts from the same source. */
  if ((ADC0->IF & ADC_IF_SINGLECMP) || (ADC0->IF & ADC_IF_SCANCMP))
  {
    /* Stop PRS trigger from RTCC, read interrupt flag with clear */
    RTCC_Enable(false);  
    NVIC_DisableIRQ(ADC0_IRQn);
    adcIntFlag = ADC0->IFC;
  }
}

/***************************************************************************//**
 * @brief LDMA IRQ handler.
 ******************************************************************************/
void LDMA_IRQHandler( void )
{
  uint32_t pending;

  /* Read and clear interrupt source */
  pending = LDMA->IFC;

  if (pending & ADC_DMA_CH_MASK)
  {
    /* Setup LDMA for next transfer, common for single and scan mode */
    LDMA->CH[ADC_DMA_CHANNEL].CTRL = LDMA_CH_CTRL_SIZE_WORD +
                                     LDMA_CH_CTRL_SRCINC_NONE +
                                     LDMA_CH_CTRL_IGNORESREQ + 
                                     LDMA_CH_CTRL_BLOCKSIZE_UNIT4 +
                                     ((ADC_BUFFER_SIZE - 1) << _LDMA_CH_CTRL_XFERCNT_SHIFT);
    LDMA->CH[ADC_DMA_CHANNEL].DST = (uint32_t)&adcBuffer;
    LDMA->CHEN = ADC_DMA_CH_MASK;
  }

  /* Check for LDMA error */
  if ( pending & LDMA_IF_ERROR )
  {
    /* Loop here to enable the debugger to see what has happened */
    while (1)
      ;
  }
}

/**************************************************************************//**
* @brief Dummy funtion for Memory LCD initialization.
*        RTCC is used for PRS trigger so no toggle on
*        Memory LCD external COM inversion signal pin.
*****************************************************************************/
EMSTATUS PAL_GpioPinAutoToggle (unsigned int gpioPort,
                                unsigned int gpioPin,
                                unsigned int frequency)
{
  EMSTATUS status = EMSTATUS_OK;
  return status;
}

/**************************************************************************//**
 * @brief Setup GPIO interrupt for pushbuttons.
 *****************************************************************************/
void gpioSetup(void)
{
  /* Enable GPIO clock */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure push button 0 as input and enable interrupt  */
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPull, 1);
  GPIO_ExtIntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, BSP_GPIO_PB0_PIN, false, true, true);

  /* Configure push button 1 as input and enable interrupt */
  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInputPull, 1);
  GPIO_ExtIntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, BSP_GPIO_PB1_PIN, false, true, true);

  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/**************************************************************************//**
 * @brief Setup RTCC as PRS source to trigger ADC
 *****************************************************************************/
void rtccSetup(void)
{
  RTCC_Init_TypeDef rtccInit = RTCC_INIT_DEFAULT;
  RTCC_CCChConf_TypeDef rtccInitCompareChannel = RTCC_CH_INIT_COMPARE_DEFAULT;

  /* Enabling clock to the interface of the low energy modules */
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Routing the LFXO clock to the RTCC */
  CMU_ClockSelectSet(cmuClock_LFE, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_RTCC, true);
   
  rtccInitCompareChannel.prsSel = RTCC_PRS_CH_SELECT;
  
  /* Setting the compare value of the RTCC */
  RTCC_ChannelInit(RTCC_CC_CHANNEL, &rtccInitCompareChannel);
  RTCC_ChannelCCVSet(RTCC_CC_CHANNEL, RTCC_WAKEUP_COUNT);
  
  /* Clear counter on compare match */
  rtccInit.cntWrapOnCCV1 = true;        
  rtccInit.presc = rtccCntPresc_1;
  rtccInit.enable = false;

  /* Initialize the RTCC */
  RTCC_Init(&rtccInit);
}

/**************************************************************************//**
 * @brief Initialize ADC for single and scan conversion
 *****************************************************************************/
void adcSetup(void)
{
  /* Enable ADC clock */
  CMU_ClockEnable(cmuClock_ADC0, true);

  /* Select AUXHFRCO for ADC ASYNC mode so that ADC can run on EM2 */
  CMU->ADCCTRL = CMU_ADCCTRL_ADC0CLKSEL_AUXHFRCO;

  /* Initialize compare threshold for both single and scan conversion */
  ADC0->CMPTHR = _ADC_CMPTHR_RESETVALUE;
  ADC0->CMPTHR = (ADC_CMP_GT_VALUE << _ADC_CMPTHR_ADGT_SHIFT) +
                 (ADC_CMP_LT_VALUE << _ADC_CMPTHR_ADLT_SHIFT);
}

/***************************************************************************//**
 * @brief Initialize the LDMA controller.
 ******************************************************************************/
void ldmaSetup(void)
{
  LDMA_Init_t init = LDMA_INIT_DEFAULT;

  LDMA_Init(&init);
  CMU_ClockEnable(cmuClock_LDMA, false);
}

/***************************************************************************//**
 * @brief Setup the LDMA controller for ADC.
 * @param[in] scanMode
 *   False for single mode ADC, True for scan mode ADC.
 ******************************************************************************/
void adcLdmaSetup(bool scanMode)
{
  /* Macro for single mode ADC */
  LDMA_TransferCfg_t adcSingleTx =
    LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_ADC0_SINGLE);

  /* Macro for scan mode ADC */
  LDMA_TransferCfg_t adcScanTx =
    LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_ADC0_SCAN);
  
  /* Macro for ADC data transfer, common for single and scan mode */
  LDMA_Descriptor_t xfer =
    LDMA_DESCRIPTOR_SINGLE_P2M_BYTE(&ADC0->SINGLEDATA, &adcBuffer, ADC_BUFFER_SIZE);
    
  CMU_ClockEnable(cmuClock_LDMA, true);
  
  /* Initialize descriptor for ADC LDMA transfer */
  descLink0 = xfer;
  descLink0.xfer.doneIfs = 0;
  descLink0.xfer.blockSize = ldmaCtrlBlockSizeUnit4;
  descLink0.xfer.ignoreSrec = 1;
  descLink0.xfer.size = ldmaCtrlSizeWord; 

  /* Start ADC LMDA transfer */
  if (!scanMode)
  {
    LDMA_StartTransfer(ADC_DMA_CHANNEL, (void*)&adcSingleTx, (void*)&descLink0);
  }
  else
  {
    descLink0.xfer.srcAddr = (uint32_t)&ADC0->SCANDATAX;
    LDMA_StartTransfer(ADC_DMA_CHANNEL, (void*)&adcScanTx, (void*)&descLink0);
  }
}

/***************************************************************************//**
 * @brief
 *   Calibrate offset and gain for the specified reference.
 *   Supports currently only single ended gain calibration.
 *   Could easily be expanded to support differential gain calibration.
 *
 * @details
 *   The offset calibration routine measures 0 V with the ADC, and adjust
 *   the calibration register until the converted value equals 0.
 *   The gain calibration routine needs an external reference voltage equal
 *   to the top value for the selected reference. For example if the 2.5 V
 *   reference is to be calibrated, the external supply must also equal 2.5V.
 *
 * @param[in] adc
 *   Pointer to ADC peripheral register block.
 *
 * @param[in] ref
 *   Reference used during calibration. Can be both external and internal
 *   references.
 *   For the VDD reference and external reference, there is no hardware gain
 *   calibration. Calibration can be done by software after taking a sample.
 *
 * @return
 *   The final value of the calibration register, note that the calibration
 *   register gets updated with this value during the calibration.
 *   No need to load the calibration values after the function returns.
 ******************************************************************************/
uint32_t ADC_Calibration(ADC_TypeDef *adc, ADC_Ref_TypeDef ref)
{
  int32_t  sample;
  uint32_t cal;

  /* Binary search variables */
  uint8_t high;
  uint8_t mid;
  uint8_t low;

  /* Reset ADC to be sure we have default settings and wait for ongoing */
  /* conversions to be complete. */
  ADC_Reset(adc);

  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  /* Init common settings for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(ADC_CLOCK, 0);

  /* Set an oversampling rate for more accuracy */
  init.ovsRateSel = adcOvsRateSel4096;
  ADC_Init(adc, &init);

  /* Init for single conversion use, connect POSSEL and NEGSEL to VSS. */
  singleInit.reference = ref;
  singleInit.posSel    = adcPosSelVSS;
  singleInit.negSel    = adcNegSelVSS;
  singleInit.acqTime   = adcAcqTime16;
  singleInit.fifoOverwrite = true;
  /* Enable oversampling rate */
  singleInit.resolution = adcResOVS;

  ADC_InitSingle(adc, &singleInit);

  /* Positive single ended offset calibration, scan from higher ADC results */
  mid = 15;
  adc->CAL |= ADC_CAL_CALEN;
  adc->SINGLEFIFOCLEAR = ADC_SINGLEFIFOCLEAR_SINGLEFIFOCLEAR;

  while (1)
  {
    /* Write to calibration register */
    cal      = adc->CAL & ~(_ADC_CAL_SINGLEOFFSET_MASK | _ADC_CAL_SCANOFFSET_MASK);
    cal     |= (uint8_t)(mid) << _ADC_CAL_SINGLEOFFSET_SHIFT;
    cal     |= (uint8_t)(mid) << _ADC_CAL_SCANOFFSET_SHIFT;
    adc->CAL = cal;
    
    /* Start ADC single conversion */
    ADC_Start(ADC0, adcStartSingle);
    while ((ADC0->IF & ADC_IF_SINGLE) == 0)
      ;
    
    sample = ADC_DataSingleGet(ADC0);
    if ((sample == 0) || (mid == 0))
    {
      break;
    }
    mid--;
  }

  /* Negative single ended offset calibration, scan from lower ADC results */
  mid = 0;
  adc->CAL |= ADC_CAL_OFFSETINVMODE;
  adc->SINGLEFIFOCLEAR = ADC_SINGLEFIFOCLEAR_SINGLEFIFOCLEAR;

  while (1)
  {
    /* Write to calibration register */
    cal      = adc->CAL & ~(_ADC_CAL_SINGLEOFFSETINV_MASK | _ADC_CAL_SCANOFFSETINV_MASK);
    cal     |= (uint8_t)(mid) << _ADC_CAL_SINGLEOFFSETINV_SHIFT;
    cal     |= (uint8_t)(mid) << _ADC_CAL_SCANOFFSETINV_SHIFT;
    adc->CAL = cal;
    
    /* Start ADC single conversion */
    ADC_Start(ADC0, adcStartSingle);
    while ((ADC0->IF & ADC_IF_SINGLE) == 0)
      ;
    
    sample = ADC_DataSingleGet(ADC0);
    if ((sample >= ADC_NEG_OFFSET_VALUE) || (mid == 15))
    {
      break;
    }
    mid++;
  }

  /* Positive single ended gain calibration */
  adc->SINGLECTRL &= ~(_ADC_SINGLECTRL_POSSEL_MASK);
  adc->SINGLECTRL |= (ADC_CAL_INPUT<< _ADC_SINGLECTRL_POSSEL_SHIFT);
  adc->CAL &= ~ADC_CAL_OFFSETINVMODE;
  adc->SINGLEFIFOCLEAR = ADC_SINGLEFIFOCLEAR_SINGLEFIFOCLEAR;

  high = 128;
  low  = 0;

  /* Do binary search for gain calibration */
  while (low < high)
  {
    /* Calculate midpoint and write to calibration register */
    mid = low + (high - low) / 2;
    cal      = adc->CAL & ~(_ADC_CAL_SINGLEGAIN_MASK | _ADC_CAL_SCANGAIN_MASK);
    cal     |= mid << _ADC_CAL_SINGLEGAIN_SHIFT;
    cal     |= mid << _ADC_CAL_SCANGAIN_SHIFT;
    adc->CAL = cal;

    /* Do a conversion */
    ADC_Start(adc, adcStartSingle);
    while ((ADC0->IF & ADC_IF_SINGLE) == 0)
      ;

    /* Get ADC result */
    sample = ADC_DataSingleGet(adc);

    /* Check result and decide in which part to repeat search */
    /* Compare with a value atleast one LSB's less than top to avoid overshooting */
    /* Since oversampling is used, the result is 16 bits, but a couple of lsb's */
    /* applies to the 12 bit result value, if 0xffd is the top value in 12 bit, this */
    /* is in turn 0xffd0 in the 16 bit result. */
    /* Calibration register has negative effect on result */
    if (sample > ADC_GAIN_CAL_VALUE)
    {
      /* Repeat search in top half. */
      low = mid + 1;
    }
    else if (sample < ADC_GAIN_CAL_VALUE)
    {
      /* Repeat search in bottom half. */
      high = mid;
    }
    else
    {
      /* Found it, exit while loop */
      break;
    }
  }

  /* Mask off CALEN bit */
  return(adc->CAL &= ~ADC_CAL_CALEN);
}

/**************************************************************************//**
 * @brief Reset ADC related registers and parameters to default values.
 *****************************************************************************/
void adcReset(void)
{
  uint32_t i;
  
  /* Switch the ADCCLKMODE to SYNC */
  NVIC_DisableIRQ(ADC0_IRQn);
  ADC0->CTRL &= ~ADC_CTRL_ADCCLKMODE_ASYNC;

  /* Rest ADC registers */
  ADC_Reset(ADC0);

  /* Fill buffer and clear flag */
  for (i=0; i<ADC_BUFFER_SIZE; i++)
  {
    adcBuffer[i] = ADC_CMP_LT_VALUE;
  }
  adcIntFlag = 0;
  
  /* Reset AUXHFRCO to default */
  CMU_AUXHFRCOFreqSet(cmuAUXHFRCOFreq_19M0Hz);
  
  /* Disable clock */
  CMU_ClockEnable(cmuClock_PRS, false);
  CMU_ClockEnable(cmuClock_LDMA, false);

  /* Enable key interrupt */
  runKey = false;
  menuKey = false;
  GPIO->IFC = _GPIO_IFC_MASK;
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/***************************************************************************//**
 * @brief Setup RTCC as PRS producer to trigger ADC in EM1 and EM2.
 * @param[in] emode2
 *   False for EM1, True for EM2.
 ******************************************************************************/
void adcEmodePrs(bool emode2)
{
  CMU_ClockEnable(cmuClock_PRS, true);
  if (!emode2)
  {
    /* Use RTCC as SYNC PRS to trigger ADC in EM1 */
    PRS_SourceSignalSet(RTCC_PRS_CHANNEL, PRS_CH_CTRL_SOURCESEL_RTCC, PRS_CH_CTRL_SIGSEL_RTCCCCV1, prsEdgePos);
  }
  else
  {
    /* Use RTCC as ASYNC PRS to trigger ADC in EM2 */
    PRS_SourceAsyncSignalSet(RTCC_PRS_CHANNEL, PRS_CH_CTRL_SOURCESEL_RTCC, PRS_CH_CTRL_SIGSEL_RTCCCCV1);
    /* PRS clock is not required in ASYNC mode */
    CMU_ClockEnable(cmuClock_PRS, false);

    /* Power down LCD display and disable GPIO clock to reduce current consumption in EM2 */
    displayDevice.pDisplayPowerOn(&displayDevice, false);
    CMU_ClockEnable(PAL_SPI_USART_CLOCK, false);
    CMU_ClockEnable(cmuClock_GPIO, false);
  }    
}

/**************************************************************************//**
 * @brief ADC single and scan conversion example (Single-ended mode)
 * @param[in] ovs
 *   False for 12 bit ADC, True for 16 bit oversampling ADC.
 *****************************************************************************/
void adcSingleScan(bool ovs)
{
  uint32_t id, sample, adcMax;

  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;
  ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;

  adcMax = ADC_12BIT_MAX;
  if (ovs)
  {
    adcMax = ADC_16BIT_MAX;
  }
    
  /* Init common issues for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(ADC_CLOCK, 0);
  if (ovs)
  {
    /* Set oversampling rate */
    init.ovsRateSel = adcOvsRateSel256;
  }
  ADC_Init(ADC0, &init);

  /* Initialize for single conversion */
  singleInit.reference = adcRefVDD;
  singleInit.posSel = ADC_INPUT0;
  singleInit.negSel = adcNegSelVSS;
  if (ovs)
  {
    /* Enable oversampling rate */
    singleInit.resolution = adcResOVS;
  }
  ADC_InitSingle(ADC0, &singleInit);
  
  /* Setup scan channels, define DEBUG_EFM in debug build to identify invalid channel range */
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, ADC_INPUT0);
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, ADC_INPUT1);
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, ADC_INPUT2);
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, ADC_INPUT3);

  /* Initialize for scan conversion */
  scanInit.reference = adcRefVDD;
  if (ovs)
  {
    /* Enable oversampling rate */
    scanInit.resolution = adcResOVS;
  }
  ADC_InitScan(ADC0, &scanInit);
  
  /* Set scan data valid level to trigger */
  ADC0->SCANCTRLX |= (ADC_SCAN_DVL - 1) << _ADC_SCANCTRLX_DVL_SHIFT;

  /* Start ADC single conversion */
  ADC_Start(ADC0, adcStartSingle);
  while ((ADC0->IF & ADC_IF_SINGLE) == 0)
    ;
  
  /* Get ADC single result */
  sample = ADC_DataSingleGet(ADC0);
  printf("Single PA0: %1.4fV\n",((float)sample * ADC_SE_VFS)/adcMax); 

  /* Start ADC scan conversion */
  ADC_Start(ADC0, adcStartScan);
  while ((ADC0->IF & ADC_IF_SCAN) == 0)
    ;
  
  /* Get ADC scan results */
  sample = ADC_DataIdScanGet(ADC0, &id);
  printf("Scan PA%lu: %1.4fV\n", id, ((float)(sample) * ADC_SE_VFS)/adcMax); 
  sample = ADC_DataIdScanGet(ADC0, &id);
  printf("Scan PA%lu: %1.4fV\n", id, ((float)(sample) * ADC_SE_VFS)/adcMax); 
  sample = ADC_DataIdScanGet(ADC0, &id);
  printf("Scan PD%lu: %1.4fV\n", id, ((float)(sample) * ADC_SE_VFS)/adcMax); 
  sample = ADC_DataIdScanGet(ADC0, &id);
  printf("Scan PD%lu: %1.4fV\n", id, ((float)(sample) * ADC_SE_VFS)/adcMax);
  adcReset();
}

/**************************************************************************//**
 * @brief ADC single and scan conversion example (Differential mode)
 *****************************************************************************/
void adcSingleScanDiff(void)
{
  uint32_t id;
  int16_t sample;       /* 16 bit signed interger for differential result */

  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;
  ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;

  /* Init common issues for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(ADC_CLOCK, 0);
  ADC_Init(ADC0, &init);

  /* Initialize for differential single conversion */
  singleInit.reference = adcRef2xVDD;
  singleInit.posSel = ADC_INPUT0;
  singleInit.negSel = ADC_DIFF_INPUT1;
  singleInit.diff = true;
  ADC_InitSingle(ADC0, &singleInit);
  
  /* Setup differential scan channels, define DEBUG_EFM in debug build to identify invalid channel range */
  ADC_ScanDifferentialInputAdd(&scanInit, adcScanInputGroup0, ADC_INPUT0, adcScanNegInputDefault);
  ADC_ScanDifferentialInputAdd(&scanInit, adcScanInputGroup1, ADC_INPUT2, adcScanNegInputDefault);
  
  /* Initialize for differential scan conversion */
  scanInit.reference = adcRef2xVDD;
  scanInit.diff = true;
  ADC_InitScan(ADC0, &scanInit);
  
  /* Set differential scan data valid level to trigger */
  ADC0->SCANCTRLX |= (ADC_SCAN_DIFF_DVL - 1) << _ADC_SCANCTRLX_DVL_SHIFT;

  /* Start ADC single conversion */
  ADC_Start(ADC0, adcStartSingle);
  while ((ADC0->IF & ADC_IF_SINGLE) == 0)
    ;
  
  /* Get ADC single result */
  sample = ADC_DataSingleGet(ADC0);
  printf("Single Differential\n");
  printf("PA0-PA1: %1.4fV\n",((float)sample * ADC_DIFF_VFS)/ADC_12BIT_MAX); 

  /* Start ADC scan conversion */
  ADC_Start(ADC0, adcStartScan);
  while ((ADC0->IF & ADC_IF_SCAN) == 0)
    ;
  
  /* Get ADC scan results */
  sample = ADC_DataIdScanGet(ADC0, &id);
  printf("\nScan Differential\n");
  printf("PA%lu-PA1: %1.4fV\n", id, ((float)(sample) * ADC_DIFF_VFS)/ADC_12BIT_MAX); 
  sample = ADC_DataIdScanGet(ADC0, &id);
  printf("PD%lu-PD11: %1.4fV\n", id, ((float)(sample) * ADC_DIFF_VFS)/ADC_12BIT_MAX); 
  adcReset();
}

/**************************************************************************//**
 * @brief ADC single conversion interrupt example
 * @param[in] emode2
 *   False to run ADC in EM1, True to run ADC in EM2.
 *****************************************************************************/
void adcSingleInt(bool emode2)
{
  uint32_t i;
  
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  /* Initialize for single conversion */
  singleInit.prsSel = ADC_PRS_CH_SELECT;
  singleInit.reference = adcRefVDD;
  singleInit.posSel = ADC_INPUT0;
  singleInit.negSel = adcNegSelVSS;
  singleInit.prsEnable = true;
  singleInit.fifoOverwrite = true;
  ADC_InitSingle(ADC0, &singleInit);
  
  /* Enable single window compare */
  ADC0->SINGLECTRL |= ADC_SINGLECTRL_CMPEN;
  
  /* Set single data valid level (DVL) to trigger */
  ADC0->SINGLECTRLX |= (ADC_SINGLE_DVL - 1) << _ADC_SINGLECTRLX_DVL_SHIFT;
  
  /* Enable ADC Interrupt when reaching DVL and window compare */
  ADC_IntEnable(ADC0, ADC_IEN_SINGLE + ADC_IEN_SINGLECMP);

  if (emode2)
  {
    /* Use LOWACC if not using bandgap reference to reduce current consumption */ 
    ADC0->BIASPROG = ADC_BIASPROG_GPBIASACC;
      
    /* Switch the ADCCLKMODE to ASYNC at the end of initialization */
    init.em2ClockConfig = adcEm2ClockOnDemand;

    /* Use AUXHFRCO frequency to setup ADC if run on EM2 */
    CMU_AUXHFRCOFreqSet(ADC_ASYNC_CLOCK);
    init.timebase = ADC_TimebaseCalc(CMU_AUXHFRCOBandGet());
    init.prescale = ADC_PrescaleCalc(ADC_CLOCK, CMU_AUXHFRCOBandGet());
  }
  else
  {
    /* Use HFPERCLK frequency to setup ADC if run on EM1 */
    init.timebase = ADC_TimebaseCalc(0);
    init.prescale = ADC_PrescaleCalc(ADC_CLOCK, 0);
  }
  /* Init common issues for both single conversion and scan mode */
  ADC_Init(ADC0, &init);

  /* Clear the FIFOs and pending interrupt */
  ADC0->SINGLEFIFOCLEAR = ADC_SINGLEFIFOCLEAR_SINGLEFIFOCLEAR;
  NVIC_ClearPendingIRQ(ADC0_IRQn);
  NVIC_EnableIRQ(ADC0_IRQn);
  
  /* Start PRS from RTCC */
  RTCC_Enable(true);  
  
  while(1)
  {
    if (emode2)
    {
      EMU_EnterEM2(false);
    }
    else
    {
      EMU_EnterEM1();
    }
    if (adcIntFlag & ADC_IF_SINGLECMP)
    {
      /* Exit if ADC outside the compare window */
      break;
    }
  }
  
  if (emode2)
  {
    /* Enable GPIO clock and power on LCD display */
    CMU_ClockEnable(PAL_SPI_USART_CLOCK, true);
    CMU_ClockEnable(cmuClock_GPIO, true);
    displayDevice.pDisplayPowerOn(&displayDevice, true);
    printf("\f");
    printf("Example %d\n", menuLevel+1);
    printf("ADC Single Conversion\n");
    printf("Interrupt (EM2) - Run\n\n"); 
  }
  
  /* Print ADC value after SINGLECMP interrrupt */
  if (ADC0->SINGLEFIFOCOUNT == 0)
  {
    printf("FIFO 0: %1.4fV\n", ((float)(adcBuffer[0]) * ADC_SE_VFS)/ADC_12BIT_MAX); 
    printf("FIFO 1: %1.4fV\n", ((float)(adcBuffer[1]) * ADC_SE_VFS)/ADC_12BIT_MAX); 
    printf("FIFO 2: %1.4fV\n", ((float)(adcBuffer[2]) * ADC_SE_VFS)/ADC_12BIT_MAX); 
    printf("FIFO 3: %1.4fV\n", ((float)(adcBuffer[3]) * ADC_SE_VFS)/ADC_12BIT_MAX);
  }
  else
  {
    i = 0;
    while (ADC0->SINGLEFIFOCOUNT)
    {
      printf("FIFO %lu: %1.4fV\n", i, ((float)(ADC_DataSingleGet(ADC0)) * ADC_SE_VFS)/ADC_12BIT_MAX);
      i++;
    }
  }
  adcReset();
}

/**************************************************************************//**
 * @brief ADC scan conversion interrupt example
 * @param[in] emode2
 *   False to run ADC in EM1, True to run ADC in EM2.
 *****************************************************************************/
void adcScanInt(bool emode2)
{
  uint32_t i;
  
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;

  /* Setup scan channels, define DEBUG_EFM in debug build to identify invalid channel range */
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, ADC_INPUT0);
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, ADC_INPUT1);
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, ADC_INPUT2);
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, ADC_INPUT3);
  
  /* Initialize for scan conversion */
  scanInit.prsSel = ADC_PRS_CH_SELECT;
  scanInit.reference = adcRefVDD;
  scanInit.prsEnable = true;
  scanInit.fifoOverwrite = true;
  ADC_InitScan(ADC0, &scanInit);

  /* Enable scan window compare */
  ADC0->SCANCTRL |= ADC_SCANCTRL_CMPEN;
  
  /* Set scan data valid level (DVL) to trigger */
  ADC0->SCANCTRLX |= (ADC_SCAN_DVL - 1) << _ADC_SCANCTRLX_DVL_SHIFT;
  
  /* Enable ADC Interrupt when reaching DVL and window compare */
  ADC_IntEnable(ADC0, ADC_IEN_SCAN + ADC_IEN_SCANCMP);

  if (emode2)
  {
    /* Use LOWACC if not using bandgap reference to reduce current consumption */ 
    ADC0->BIASPROG = ADC_BIASPROG_GPBIASACC;

    /* Switch the ADCCLKMODE to ASYNC at the end of initialization */
    init.em2ClockConfig = adcEm2ClockOnDemand;

    /* Use AUXHFRCO frequency to setup ADC if run on EM2 */
    CMU_AUXHFRCOFreqSet(ADC_ASYNC_CLOCK);
    init.timebase = ADC_TimebaseCalc(CMU_AUXHFRCOBandGet());
    init.prescale = ADC_PrescaleCalc(ADC_CLOCK, CMU_AUXHFRCOBandGet());
  }
  else
  {
    /* Use HFPERCLK frequency to setup ADC if run on EM1 */
    init.timebase = ADC_TimebaseCalc(0);
    init.prescale = ADC_PrescaleCalc(ADC_CLOCK, 0);
  }  
  /* Init common issues for both single conversion and scan mode */
  ADC_Init(ADC0, &init);

  /* Clear the FIFOs and pending interrupt */
  ADC0->SCANFIFOCLEAR = ADC_SCANFIFOCLEAR_SCANFIFOCLEAR;
  NVIC_ClearPendingIRQ(ADC0_IRQn);
  NVIC_EnableIRQ(ADC0_IRQn);
  
  /* Start PRS from RTCC */
  RTCC_Enable(true);
  
  while(1)
  {
    if (emode2)
    {
      EMU_EnterEM2(false);
    }
    else
    {
      EMU_EnterEM1();
    }
    if (adcIntFlag & ADC_IF_SCANCMP)
    {
      /* Exit if ADC outside the compare window */
      break;
    }
  }
  
  if (emode2)
  {
    /* Enable GPIO clock and power on LCD display */
    CMU_ClockEnable(PAL_SPI_USART_CLOCK, true);
    CMU_ClockEnable(cmuClock_GPIO, true);
    displayDevice.pDisplayPowerOn(&displayDevice, true);
    printf("\f");
    printf("Example %d\n", menuLevel+1);
    printf("ADC Scan Conversion\n"); 
    printf("Interrupt (EM2) - Run\n\n"); 
  }

  /* Print ADC value after SCANCMP interrrupt */
  if (ADC0->SCANFIFOCOUNT == 0)
  {
    for (i=0; i<4; i++)
    {
      adcBuffer[4] = (adcBuffer[i] & _ADC_SCANDATAX_SCANINPUTID_MASK) >> _ADC_SCANDATAX_SCANINPUTID_SHIFT;
      adcBuffer[5] = adcBuffer[i] & _ADC_SCANDATAX_DATA_MASK;
      if (adcBuffer[4] < 8)
      {
        printf("Scan PA%lu: %1.4fV\n", adcBuffer[4], ((float)(adcBuffer[5]) * ADC_SE_VFS)/ADC_12BIT_MAX);
      }
      else
      {
        printf("Scan PD%lu: %1.4fV\n", adcBuffer[4], ((float)(adcBuffer[5]) * ADC_SE_VFS)/ADC_12BIT_MAX);
      }
    }
  }
  else
  {
    while (ADC0->SCANFIFOCOUNT)
    {
      adcBuffer[5] = ADC_DataIdScanGet(ADC0, &adcBuffer[4]);
      if (adcBuffer[4] < 8)
      {
        printf("Scan PA%lu: %1.4fV\n", adcBuffer[4], ((float)(adcBuffer[5]) * ADC_SE_VFS)/ADC_12BIT_MAX);
      }
      else
      {
        printf("Scan PD%lu: %1.4fV\n", adcBuffer[4], ((float)(adcBuffer[5]) * ADC_SE_VFS)/ADC_12BIT_MAX);
      }
    }
  }
  adcReset();
}

/**************************************************************************//**
 * @brief ADC single conversion LDMA example
 * @param[in] emode2
 *   False to run ADC in EM1, True to run ADC in EM2.
 *****************************************************************************/
void adcSingleLdma(bool emode2)
{
  uint32_t i;
  
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  /* Initialize for single conversion */
  singleInit.prsSel = ADC_PRS_CH_SELECT;
  singleInit.reference = adcRefVDD;
  singleInit.posSel = ADC_INPUT0;
  singleInit.negSel = adcNegSelVSS;
  singleInit.prsEnable = true;
  singleInit.fifoOverwrite = true;
  ADC_InitSingle(ADC0, &singleInit);
  
  /* Enable single window compare */
  ADC0->SINGLECTRL |= ADC_SINGLECTRL_CMPEN;
  
  /* Set single data valid level (DVL) to trigger */
  ADC0->SINGLECTRLX |= (ADC_SINGLE_DVL - 1) << _ADC_SINGLECTRLX_DVL_SHIFT;
  
  /* Enable window compare interrupt only */
  ADC_IntEnable(ADC0, ADC_IEN_SINGLECMP);

  if (emode2)
  {
    /* Use LOWACC if not using bandgap reference to reduce current consumption */ 
    ADC0->BIASPROG = ADC_BIASPROG_GPBIASACC;

    /* Use AUXHFRCO frequency to setup ADC if run on EM2 */
    CMU_AUXHFRCOFreqSet(ADC_ASYNC_CLOCK);
    init.timebase = ADC_TimebaseCalc(CMU_AUXHFRCOBandGet());
    init.prescale = ADC_PrescaleCalc(ADC_CLOCK, CMU_AUXHFRCOBandGet());
  }
  else
  {
    /* Use HFPERCLK frequency to setup ADC if run on EM1 */
    init.timebase = ADC_TimebaseCalc(0);
    init.prescale = ADC_PrescaleCalc(ADC_CLOCK, 0);
  }
  /* Init common issues for both single conversion and scan mode */
  ADC_Init(ADC0, &init);
  
  /* Switch the ADCCLKMODE to ASYNC at the end of initialization */
  if (emode2)
  {
    /* Set DMA availability in EM2 and ADC EM2 clock configuration */
    BUS_RegBitWrite(&ADC0->CTRL, _ADC_CTRL_SINGLEDMAWU_SHIFT, 1);
    BUS_RegMaskedWrite(&ADC0->CTRL,
                       _ADC_CTRL_ADCCLKMODE_MASK | _ADC_CTRL_ASYNCCLKEN_MASK,
                       adcEm2ClockOnDemand);
  }

  /* Clear the FIFOs and pending interrupt */
  ADC0->SINGLEFIFOCLEAR = ADC_SINGLEFIFOCLEAR_SINGLEFIFOCLEAR;
  NVIC_ClearPendingIRQ(ADC0_IRQn);
  NVIC_EnableIRQ(ADC0_IRQn);
  
  /* Setup LDMA and start PRS from RTCC */
  adcLdmaSetup(false);
  RTCC_Enable(true);
  
  while(1)
  {
    if (emode2)
    {
      EMU_EnterEM2(false);
    }
    else
    {
      EMU_EnterEM1();
    }
    if (adcIntFlag & ADC_IF_SINGLECMP)
    {
      /* Exit if ADC outside the compare window */
      break;
    }
  }
  
  if (emode2)
  {
    /* Enable GPIO clock and power on LCD display */
    CMU_ClockEnable(PAL_SPI_USART_CLOCK, true);
    CMU_ClockEnable(cmuClock_GPIO, true);
    displayDevice.pDisplayPowerOn(&displayDevice, true);
    printf("\f");
    printf("Example %d\n", menuLevel+1);
    printf("ADC Single Conversion\n");
    printf("LDMA (EM2) - Run\n\n"); 
  }

  /* Print ADC value after SINGLECMP interrrupt */
  if (ADC0->SINGLEFIFOCOUNT == 0)
  {
    /* Get LMDA XFERCNT */
    i = (LDMA->CH[ADC_DMA_CHANNEL].CTRL & _LDMA_CH_CTRL_XFERCNT_MASK)
        >> _LDMA_CH_CTRL_XFERCNT_SHIFT;
 
    if (i == (ADC_BUFFER_SIZE - 1))
    {
      /* LDMA DONE interrupt has just triggered, data in the last DVL */
      i = ADC_BUFFER_SIZE - ADC_SINGLE_DVL;
    }
    else
    {
      /* Get data pointer if LDMA has not yet finished */ 
      i = ADC_BUFFER_SIZE - (i + 1) - ADC_SINGLE_DVL; 
    }
    printf("FIFO 0: %1.4fV\n", ((float)(adcBuffer[i++]) * ADC_SE_VFS)/ADC_12BIT_MAX); 
    printf("FIFO 1: %1.4fV\n", ((float)(adcBuffer[i++]) * ADC_SE_VFS)/ADC_12BIT_MAX); 
    printf("FIFO 2: %1.4fV\n", ((float)(adcBuffer[i++]) * ADC_SE_VFS)/ADC_12BIT_MAX); 
    printf("FIFO 3: %1.4fV\n", ((float)(adcBuffer[i]) * ADC_SE_VFS)/ADC_12BIT_MAX);
  }
  else
  {
    i = 0;
    while (ADC0->SINGLEFIFOCOUNT)
    {
      printf("FIFO %lu: %1.4fV\n", i, ((float)(ADC_DataSingleGet(ADC0)) * ADC_SE_VFS)/ADC_12BIT_MAX);
      i++;
    }
  }
  adcReset();
}

/**************************************************************************//**
 * @brief ADC scan conversion LDMA example
 * @param[in] emode2
 *   False to run ADC in EM1, True to run ADC in EM2.
 *****************************************************************************/
void adcScanLdma(bool emode2)
{
  uint32_t i, chId, chSample;
  
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;

  /* Setup scan channels, define DEBUG_EFM in debug build to identify invalid channel range */
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, ADC_INPUT0);
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, ADC_INPUT1);
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, ADC_INPUT2);
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, ADC_INPUT3);
  
  /* Initialize for scan conversion */
  scanInit.prsSel = ADC_PRS_CH_SELECT;
  scanInit.reference = adcRefVDD;
  scanInit.prsEnable = true;
  scanInit.fifoOverwrite = true;
  ADC_InitScan(ADC0, &scanInit);

  /* Enable scan window compare */
  ADC0->SCANCTRL |= ADC_SCANCTRL_CMPEN;
  
  /* Set scan data valid level (DVL) to trigger */
  ADC0->SCANCTRLX |= (ADC_SCAN_DVL - 1) << _ADC_SCANCTRLX_DVL_SHIFT;
  
  /* Enable window compare interrupt only */
  ADC_IntEnable(ADC0, ADC_IEN_SCANCMP);

  if (emode2)
  {
    /* Use LOWACC if not using bandgap reference to reduce current consumption */ 
    ADC0->BIASPROG = ADC_BIASPROG_GPBIASACC;

    /* Use AUXHFRCO frequency to setup ADC if run on EM2 */
    CMU_AUXHFRCOFreqSet(ADC_ASYNC_CLOCK);
    init.timebase = ADC_TimebaseCalc(CMU_AUXHFRCOBandGet());
    init.prescale = ADC_PrescaleCalc(ADC_CLOCK, CMU_AUXHFRCOBandGet());
  }
  else
  {
    /* Use HFPERCLK frequency to setup ADC if run on EM1 */
    init.timebase = ADC_TimebaseCalc(0);
    init.prescale = ADC_PrescaleCalc(ADC_CLOCK, 0);
  }  
  /* Init common issues for both single conversion and scan mode */
  ADC_Init(ADC0, &init);

  /* Switch the ADCCLKMODE to ASYNC at the end of initialization */
  if (emode2)
  {
    /* Set DMA availability in EM2 and ADC EM2 clock configuration */
    BUS_RegBitWrite(&ADC0->CTRL, _ADC_CTRL_SCANDMAWU_SHIFT, 1);
    BUS_RegMaskedWrite(&ADC0->CTRL,
                       _ADC_CTRL_ADCCLKMODE_MASK | _ADC_CTRL_ASYNCCLKEN_MASK,
                       adcEm2ClockOnDemand);
  }

  /* Clear the FIFOs and pending interrupt */
  ADC0->SCANFIFOCLEAR = ADC_SCANFIFOCLEAR_SCANFIFOCLEAR;
  NVIC_ClearPendingIRQ(ADC0_IRQn);
  NVIC_EnableIRQ(ADC0_IRQn);

  /* Setup LDMA and start PRS from RTCC */
  adcLdmaSetup(true);
  RTCC_Enable(true);

  while(1)
  {
    if (emode2)
    {
      EMU_EnterEM2(false);
    }
    else
    {
      EMU_EnterEM1();
    }
    if (adcIntFlag & ADC_IF_SCANCMP)
    {
      /* Exit if ADC outside the compare window */
      break;
    }
  }

  if (emode2)
  {
    /* Enable GPIO clock and power on LCD display */
    CMU_ClockEnable(PAL_SPI_USART_CLOCK, true);
    CMU_ClockEnable(cmuClock_GPIO, true);
    displayDevice.pDisplayPowerOn(&displayDevice, true);
    printf("\f");
    printf("Example %d\n", menuLevel+1);
    printf("ADC Scan Conversion\n"); 
    printf("LDMA (EM2) - Run\n\n"); 
  }
  
  /* Wait DMA data fetch finish */
  while (ADC0->SCANFIFOCOUNT != 0)
    ;

  /* Get LMDA XFERCNT */
  i = (LDMA->CH[ADC_DMA_CHANNEL].CTRL & _LDMA_CH_CTRL_XFERCNT_MASK)
    >> _LDMA_CH_CTRL_XFERCNT_SHIFT;
  
  if (i == (ADC_BUFFER_SIZE - 1))
  {
    /* LDMA DONE interrupt has just triggered, data in the last DVL */
    i = ADC_BUFFER_SIZE - ADC_SCAN_DVL;
  }
  else
  {
    /* Get data pointer if LDMA has not yet finished */ 
    i = ADC_BUFFER_SIZE - (i + 1) - ADC_SCAN_DVL; 
  }

  /* Print ADC value after SINGLECMP interrrupt */
  for (adcIntFlag=0; adcIntFlag<4; adcIntFlag++)
  {
    chId = (adcBuffer[i] & _ADC_SCANDATAX_SCANINPUTID_MASK) >> _ADC_SCANDATAX_SCANINPUTID_SHIFT;
    chSample = adcBuffer[i++] & _ADC_SCANDATAX_DATA_MASK;
    if (chId < 8)
    {
      printf("Scan PA%lu: %1.4fV\n", chId, ((float)(chSample) * ADC_SE_VFS)/ADC_12BIT_MAX); 
    }
    else
    {
      printf("Scan PD%lu: %1.4fV\n", chId, ((float)(chSample) * ADC_SE_VFS)/ADC_12BIT_MAX); 
    }
  }
  adcReset();
}

/**************************************************************************//**
 * @brief ADC advanced VFS configuration
 *****************************************************************************/
void adcVfsConfig(void)
{
  uint32_t sample;
  
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  /* Init common issues for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(ADC_CLOCK, 0);
  ADC_Init(ADC0, &init);

  /* Initialize for single conversion */
  singleInit.reference = adcRefVddxAtt;
  singleInit.posSel = ADC_INPUT0;
  singleInit.negSel = adcNegSelVSS;
  ADC_InitSingle(ADC0, &singleInit);

  /* Set VINATT and VREFATT to scale the AVDD reference */
  ADC0->SINGLECTRLX |= (ADC_VIN_ATT << _ADC_SINGLECTRLX_VINATT_SHIFT);
  ADC0->SINGLECTRLX |= (ADC_VREF_ATT << _ADC_SINGLECTRLX_VREFATT_SHIFT);
 
  /* Start ADC single conversion */
  ADC_Start(ADC0, adcStartSingle);
  while ((ADC0->IF & ADC_IF_SINGLE) == 0)
    ;
  
  /* Get ADC single result */
  sample = ADC_DataSingleGet(ADC0);
  printf("Single PA0: %1.4fV\n",((float)sample * ADC_SCALE_VFS)/ADC_12BIT_MAX); 
  adcReset();
}

/**************************************************************************//**
 * @brief ADC random number generator
 *****************************************************************************/
void adcPrng(void)
{
  uint32_t i, randomNumber = 0;
  
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

  /* Init common issues for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(ADC_CLOCK, 0);
  ADC_Init(ADC0, &init);

  /* Initialize for single conversion */
  singleInit.reference = adcRefVEntropy;
  singleInit.diff = true;
  singleInit.posSel = adcPosSelVSS;
  singleInit.negSel = adcNegSelVSS;
  ADC_InitSingle(ADC0, &singleInit);

  /* Set VINATT to maximum value and clear FIFO */
  ADC0->SINGLECTRLX |= _ADC_SINGLECTRLX_VINATT_MASK;
  ADC0->SINGLEFIFOCLEAR = ADC_SINGLEFIFOCLEAR_SINGLEFIFOCLEAR;
  
  /* Random number generation (30 bit) */
  for (i=0; i<10; i++)
  {
    ADC_Start(ADC0, adcStartSingle); 
    while ((ADC0->IF & ADC_IF_SINGLE) == 0)
      ;
    randomNumber |= ((ADC_DataSingleGet(ADC0) & 0x07) << (i * 3)); 
  }

  printf("Random Number:\n"); 
  printf("%lu", randomNumber); 
  adcReset();
}

/**************************************************************************//**
 * @brief Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  /* Init DCDC regulator if available */
#if defined( _EMU_DCDCCTRL_MASK )  
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;
  EMU_DCDCInit(&dcdcInit);
#endif
  
  /* Setup MCU clock to 4 MHz */
  CMU_HFRCOFreqSet(cmuHFRCOFreq_4M0Hz);

  /* Enable atomic read-clear operation on reading IFC register */
  MSC->CTRL |= MSC_CTRL_IFCREADCLEAR;
  
  /* Initialize the display module. */
  DISPLAY_Init();

  /* Retrieve the properties of the display. */
  if (DISPLAY_DeviceGet(0, &displayDevice ) != DISPLAY_EMSTATUS_OK)
  {
    /* Unable to get display handle. */
    while (1);
  }
  
  /* Retarget stdio to a text display. */
  if (RETARGET_TextDisplayInit() != TEXTDISPLAY_EMSTATUS_OK)
  {
    /* Text display initialization failed. */
    while (1) ;
  }
  
  /* Initialize GPIO */
  gpioSetup();

  /* Initialize RTCC */
  rtccSetup();
  
  /* Initialize LDMA */
  ldmaSetup();
  
  /* Initialize ADC */
  adcSetup();
  
  /* Display first item */
  printf("\f");
  printf("Example %d\n", menuLevel+1);
  printf("ADC Single and\n");
  printf("Scan Conversion\n"); 
  printf("\nPress BTN1 to next\n");
  printf("menu\n");
  printf("Press BTN0 to start"); 
  
  /* Wait key press to process */
  while (1)
  {
    EMU_EnterEM2(false);

    /* Push button BTN1 to browse menu */
    if (menuKey)
    {
      printf("\f");
      printf("Example %d\n", menuLevel+1);
      
      switch (menuLevel)
      {
      case SING_SCAN:
        printf("ADC Single and\n");
        printf("Scan Conversion\n");
        printf("\nPress BTN1 to next\n");
        printf("menu\n");
        printf("Press BTN0 to start"); 
        break;
        
      case SING_SCAN_OVS:
        printf("ADC Single and\n");
        printf("Scan Conversion\n"); 
        printf("(Oversampling)\n"); 
        printf("\nPress BTN1 to next\n");
        printf("menu\n");
        printf("Press BTN0 to start"); 
        break;
        
      case SING_SCAN_DIFF:
        printf("ADC Single and\n");
        printf("Scan Conversion\n"); 
        printf("(Differential Inputs)\n"); 
        printf("\nPress BTN1 to next\n");
        printf("menu\n");
        printf("Press BTN0 to start"); 
        break;

      case SING_INT_EM1:
        printf("ADC Single Conversion\n");
        printf("Interrupt (EM1)\n"); 
        printf("\nPress BTN1 to next\n");
        printf("menu\n");
        printf("Press BTN0 to start"); 
        break;        

      case SING_INT_EM2:
        printf("ADC Single Conversion\n");
        printf("Interrupt (EM2)\n\n"); 
        printf("Exit if voltage in\n");
        printf("PA0 is outside the\n");
        printf("compare window\n");
        printf("(0.5V-3V)\n");
        printf("\nPress BTN1 to next\n");
        printf("menu\n");
        printf("Press BTN0 to start"); 
        break;

      case SCAN_INT_EM1:
        printf("ADC Scan Conversion\n"); 
        printf("Interrupt (EM1)\n"); 
        printf("\nPress BTN1 to next\n");
        printf("menu\n");
        printf("Press BTN0 to start"); 
        break;
        
      case SCAN_INT_EM2:
        printf("ADC Scan Conversion\n"); 
        printf("Interrupt (EM2)\n\n"); 
        printf("Exit if voltage in\n");
        printf("any scan channels is\n");
        printf("outside the compare\n");
        printf("window (0.5V-3V)\n");
        printf("\nPress BTN1 to next\n");
        printf("menu\n");
        printf("Press BTN0 to start"); 
        break;

      case SING_LDMA_EM1:
        printf("ADC Single Conversion\n");
        printf("LDMA (EM1)\n"); 
        printf("\nPress BTN1 to next\n");
        printf("menu\n");
        printf("Press BTN0 to start"); 
        break;        

      case SING_LDMA_EM2:
        printf("ADC Single Conversion\n");
        printf("LDMA (EM2)\n\n"); 
        printf("Exit if voltage in\n");
        printf("PA0 is outside the\n");
        printf("compare window\n");
        printf("(0.5V-3V)\n");
        printf("\nPress BTN1 to next\n");
        printf("menu\n");
        printf("Press BTN0 to start"); 
        break;

      case SCAN_LDMA_EM1:
        printf("ADC Scan Conversion\n"); 
        printf("LDMA (EM1)\n"); 
        printf("\nPress BTN1 to next\n");
        printf("menu\n");
        printf("Press BTN0 to start"); 
        break;
        
      case SCAN_LDMA_EM2:
        printf("ADC Scan Conversion\n"); 
        printf("LDMA (EM2)\n\n"); 
        printf("Exit if voltage in\n");
        printf("any scan channels is\n");
        printf("outside the compare\n");
        printf("window (0.5V-3V)\n");
        printf("\nPress BTN1 to next\n");
        printf("menu\n");
        printf("Press BTN0 to start"); 
        break;

      case SING_VFS:
        printf("ADC VFS\n");
        printf("Configuration\n");
        printf("\nPress BTN1 to next\n");
        printf("menu\n");
        printf("Press BTN0 to start"); 
        break;

      case SING_PRNG:
        printf("ADC Random Number\n");
        printf("Generator\n");
        printf("\nPress BTN1 to next\n");
        printf("menu\n");
        printf("Press BTN0 to start"); 
        break;

      case SING_CAL:
        printf("ADC Calibration\n"); 
        printf("\nPress BTN1 to next\n");
        printf("menu\n");
        printf("Press BTN0 to start"); 
        break;

      default:
        break;
      }
      menuKey = false;
    }

    /* Push button BTN0 to run the selected menu */
    if (runKey)
    {
      runKey = false;
      printf("\f");
      printf("Example %d\n", menuLevel+1);
      
      /* Disable key interrupt during run */
      NVIC_DisableIRQ(GPIO_EVEN_IRQn);
      NVIC_DisableIRQ(GPIO_ODD_IRQn);
      
      switch (menuLevel)
      {
      case SING_SCAN:
        printf("ADC Single and\n");
        printf("Scan Conversion\n\n"); 
        adcSingleScan(false);
        break;
        
      case SING_SCAN_OVS:
        printf("ADC Single and\n");
        printf("Scan Conversion\n"); 
        printf("(Oversampling)\n\n"); 
        adcSingleScan(true);
        break;
        
      case SING_SCAN_DIFF:
        printf("ADC Single and\n");
        printf("Scan Conversion\n"); 
        printf("(Differential Inputs)\n\n"); 
        adcSingleScanDiff();
        break;

      case SING_INT_EM1:
        printf("ADC Single Conversion\n");
        printf("Interrupt (EM1) - Run\n\n");
        printf("Exit if voltage in\n");
        printf("PA0 is outside the\n");
        printf("compare window\n");
        printf("(0.5V-3V)\n\n");
        adcEmodePrs(false);
        adcSingleInt(false);
        break;        

      case SING_INT_EM2:
        adcEmodePrs(true);
        adcSingleInt(true);
        break;

      case SCAN_INT_EM1:
        printf("ADC Scan Conversion\n"); 
        printf("Interrupt (EM1) - Run\n\n"); 
        printf("Exit if voltage in\n");
        printf("any scan channels is\n");
        printf("outside the compare\n");
        printf("window (0.5V-3V)\n\n");
        adcEmodePrs(false);
        adcScanInt(false);
        break;
        
      case SCAN_INT_EM2:
        adcEmodePrs(true);
        adcScanInt(true);
        break;

      case SING_LDMA_EM1:
        printf("ADC Single Conversion\n");
        printf("LDMA (EM1) - Run\n\n"); 
        printf("Exit if voltage in\n");
        printf("PA0 is outside the\n");
        printf("compare window\n");
        printf("(0.5V-3V)\n\n");
        adcEmodePrs(false);
        adcSingleLdma(false);
        break;        

      case SING_LDMA_EM2:
        adcEmodePrs(true);
        adcSingleLdma(true);
        break;

      case SCAN_LDMA_EM1:
        printf("ADC Scan Conversion\n"); 
        printf("LDMA (EM1) - Run\n\n"); 
        printf("Exit if voltage in\n");
        printf("any scan channels is\n");
        printf("outside the compare\n");
        printf("window (0.5V-3V)\n\n");
        adcEmodePrs(false);
        adcScanLdma(false);
        break;
        
      case SCAN_LDMA_EM2:
        adcEmodePrs(true);
        adcScanLdma(true);
        break;

      case SING_VFS:
        printf("ADC VFS\n");
        printf("Configuration - Run\n\n");
        adcVfsConfig();
        break;

      case SING_PRNG:
        printf("ADC Random Number\n");
        printf("Generator - Run\n\n");
        adcPrng();
        break;

      case SING_CAL:
        printf("ADC Calibration - Run\n\n");
        printf("ADC CAL Register\n");
        printf("before calibration\n");
        printf("(1.25V): 0x%08lX\n", ADC0->CAL);
        printf("\nADC CAL Register\n");
        printf("after calibration\n");
        printf("(1.25V): 0x%08lX\n", ADC_Calibration(ADC0, adcRef1V25));
        adcReset();
        break;

      default:
        break;
      }
    }
  }
}

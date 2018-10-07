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
/* Tempature Inputs */
#define ADC_INPUT0              adcPosSelAPORT3XCH8     // PA0 	Radio Board P9
#define ADC_INPUT1              adcPosSelAPORT3YCH9     // PA1	Radio Board P11
#define ADC_INPUT2              adcPosSelAPORT3XCH12    // PA4 	Radio Board P14
#define ADC_INPUT3              adcPosSelAPORT3XCH2     // PD10 Radio Board P4
#define ADC_INPUT4              adcPosSelAPORT3YCH3     // PD11	Radio Board P6
#define ADC_INPUT5              adcPosSelAPORT3XCH4     // PD12	Radio Board P8
#define ADC_INPUT6              adcPosSelAPORT1XCH10    // PC10	Radio Board	P12
#define ADC_INPUT7              adcPosSelAPORT1YCH11    // PC11	Radio Board P13

#define ADC_CAL_INPUT           adcPosSelAPORT3XCH8     /* PA0 */
#define ADC_NEG_OFFSET_VALUE    0xfff0                  /* Negative offset calibration */
#define ADC_GAIN_CAL_VALUE      0xffd0                  /* Gain calibration */
#define ADC_PRS_CH_SELECT       adcPRSSELCh0

#define ADC_SCAN_DVL            4

#define ADC_BUFFER_SIZE         64
#define ADC_VIN_ATT             9                       /* VIN attenuation factor */
#define ADC_VREF_ATT            0                       /* VREF attenuation factor */
#define ADC_SE_VFS              (float)3.3              /* AVDD */
#define ADC_DIFF_VFS            (float)6.6              /* 2xAVDD */
#define ADC_SCALE_VFS           (float)2.2              /* Scaled VFS */
#define ADC_12BIT_MAX           4096                    /* 2^12 */
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
#define SCAN_LDMA		        0
#define MENU_MAX                1

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
  /* Macro for scan mode ADC */
  LDMA_TransferCfg_t adcScanTx = LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_ADC0_SCAN);
  
  /* Macro for ADC data transfer, common for single and scan mode */
  LDMA_Descriptor_t xfer = LDMA_DESCRIPTOR_SINGLE_P2M_BYTE(&ADC0->SINGLEDATA, &adcBuffer, ADC_BUFFER_SIZE);
    
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
 * @brief ADC scan conversion LDMA
 *
 *****************************************************************************/
void adcScanLdma(void)
{
  uint32_t i, chId, chSample;
  
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;

  /* Setup scan channels, define DEBUG_EFM in debug build to identify invalid channel range */
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, ADC_INPUT0);
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, ADC_INPUT1);
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, ADC_INPUT2);
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, ADC_INPUT3);
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, ADC_INPUT4);
  ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, ADC_INPUT5);
  //ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup3, ADC_INPUT6);
  //ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup3, ADC_INPUT7);
  
  /* Initialize for scan conversion */
  scanInit.prsSel = ADC_PRS_CH_SELECT;
  scanInit.reference = adcRefVDD;
  scanInit.prsEnable = true;
  scanInit.fifoOverwrite = false;
  ADC_InitScan(ADC0, &scanInit);

  /* Enable scan window compare */
  //ADC0->SCANCTRL |= ADC_SCANCTRL_CMPEN;
  
  /* Set scan data valid level (DVL) to trigger */
  ADC0->SCANCTRLX |= (ADC_SCAN_DVL - 1) << _ADC_SCANCTRLX_DVL_SHIFT;
  
  /* Enable window compare interrupt only */
  ADC_IntEnable(ADC0, ADC_IEN_SCANCMP);

  /* Use HFPERCLK frequency to setup ADC if run on EM1 */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(ADC_CLOCK, 0);
  init.ovsRateSel = adcOvsRateSel2;

  /* Init common issues for both single conversion and scan mode */
  ADC_Init(ADC0, &init);

  /* Clears FIFO Registers */
  ADC0->SCANFIFOCLEAR = ADC_SCANFIFOCLEAR_SCANFIFOCLEAR;
  /* Clears Nested Vector Interrupt Controller ADC0 IRQ */
  NVIC_ClearPendingIRQ(ADC0_IRQn);
  /* Enable NVIC ADC IRQ */
  NVIC_EnableIRQ(ADC0_IRQn);

  /* Setup LDMA and start PRS from RTCC */
  adcLdmaSetup(true);
  RTCC_Enable(true);
  
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

    printf("Scan P%lu: %1.4fV\n", chId, ((float)(chSample) * ADC_SE_VFS)/ADC_12BIT_MAX);

  }
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
  printf("ADC Scan LDMA\n");
  printf("Press BTN0 to start"); 
  
  /* Wait key press to process */
  while (1)
  {
    EMU_EnterEM2(false);

    /* Push button BTN1 to browse menu */
    if (menuKey)
    {
      switch (menuLevel)
      {

      case SCAN_LDMA:
        printf("ADC Scan LDMA\n");
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

      case SCAN_LDMA:
        printf("ADC Scan LDMA\n");
        adcEmodePrs(false);
        adcScanLdma();
        break;

      default:
        break;
      }
    }
  }
}

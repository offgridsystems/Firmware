/*
 * example.c
 *
 *  Created on: Oct 8, 2018
 *      Author: nick
 */


/**************************************************************************//**
 * @file
 * @brief Simple LED Blink Demo for SLSTK3401A
 * @version 4.4.0
 ******************************************************************************
 * @section License
 * <b>Copyright 2015 Silicon Labs, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "em_adc.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_ldma.h"
#include "em_prs.h"
#include "em_rtcc.h"
#include "em_timer.h"
#include "em_cryotimer.h"

#include "bsp.h"

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

// Timer defines
#define TRIGGER_RATE			20000 //Rate at which timer is triggered
#define TIMER_PRESCALE			1

#define ONE_MS_BASE_VALUE_COUNT 1000

#define ONE_SECOND_TIMER_COUNT	40000000/TIMER_PRESCALE
#define TIMER_REFRESH_RATE		ONE_SECOND_TIMER_COUNT/TRIGGER_RATE
#define BASE_VALUE_COUNT		ONE_MS_BASE_VALUE_COUNT/TRIGGER_RATE
#define MILLISECOND_DIVISOR		ONE_SECOND_TIMER_COUNT / ONE_MS_BASE_VALUE_COUNT

volatile uint32_t msTicks; /* counts 1ms timeTicks */

/* Buffer for ADC interrupt flag */
volatile uint32_t adcIntFlag;

/* Buffer for ADC single and scan conversion */
uint32_t adcBuffer[ADC_BUFFER_SIZE];

/***************************************************************************//**
 * @brief
 *   LDMA IRQ handler.
 ******************************************************************************/
uint32_t ldma_count = 0;
uint32_t group_count = 0;
void LDMA_IRQHandler( void )
{
	uint32_t pending, chnum, chmask;

	/* Get all pending and enabled interrupts */
	pending  = LDMA->IF;
	pending &= LDMA->IEN;

	/* Check for LDMA error */
	if ( pending & LDMA_IF_ERROR )
	{
		/* Loop here to enable the debugger to see what has happened */
		while (1)
		  ;
	}

	/* Iterate over all LDMA channels. */
	for ( chnum = 0,                chmask = 1;
		chnum < DMA_CHAN_COUNT;
		chnum++,                  chmask <<= 1 )
	{
		if ( pending & chmask )
		{
			/* Clear interrupt flag. */
			LDMA->IFC = chmask;

			/* Do more stuff here, execute callbacks etc. */
			/* Setup LDMA for next transfer, common for single and scan mode */
//			LDMA->CH[ADC_DMA_CHANNEL].CTRL = LDMA_CH_CTRL_SIZE_WORD +
//		                                     LDMA_CH_CTRL_SRCINC_NONE +
//		                                     LDMA_CH_CTRL_IGNORESREQ +
//		                                     LDMA_CH_CTRL_BLOCKSIZE_UNIT4 +
//		                                     ((16 - 1) << _LDMA_CH_CTRL_XFERCNT_SHIFT);
//		    LDMA->CH[ADC_DMA_CHANNEL].DST = (uint32_t)&adcBuffer;
//		    LDMA->CHEN = ADC_DMA_CH_MASK;
		}
	}
}


/**************************************************************************//**
 * @brief Setup timer as PRS source to trigger ADC
 *****************************************************************************/
void timerSetup(void)
{
	CMU_ClockEnable(cmuClock_TIMER0, true);
	/* Use default timer settings */
	TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;

	/* Change prescaler to 64, gives roughly 3 overflows per
	* second at 14MHz with 0xffff as top value */
	timerInit.prescale = timerPrescale1;
	TIMER_IntEnable(TIMER0, TIMER_IF_OF);

	TIMER_TopSet(TIMER0, TIMER_REFRESH_RATE - 1);
	//TIMER_Init(TIMER0, &timerInit);
}

void timerDisable(void) {
	TIMER_IntDisable(TIMER0, TIMER_IF_OF);
	CMU_ClockEnable(cmuClock_TIMER0, false);
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
}

/***************************************************************************//**
 * @brief
 *   Initialize the LDMA controller.
 ******************************************************************************/
void ldmaSetup(void)
{
  LDMA_Init_t init = LDMA_INIT_DEFAULT;
  LDMA_Init( &init );
}

/***************************************************************************//**
 * @brief Setup the LDMA controller for ADC.
 * @param[in] scanMode
 *   False for single mode ADC, True for scan mode ADC.
 ******************************************************************************/
void adcTimerSetup(int group) {
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitScan_TypeDef scanInit = ADC_INITSCAN_DEFAULT;

	/* Setup scan channels, define DEBUG_EFM in debug build to identify invalid channel range */
	if (group == 0) {
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, adcPosSelAPORT3YCH1);
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, adcPosSelAPORT3XCH2);
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, adcPosSelAPORT3YCH3);
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, adcPosSelAPORT3XCH4);
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, adcPosSelAPORT3XCH28);
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, adcPosSelAPORT3YCH29);
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, adcPosSelAPORT3XCH30);
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, adcPosSelAPORT3YCH31);
	}
	else {
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, adcPosSelAPORT1XCH6);
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup0, adcPosSelAPORT1YCH7);
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, adcPosSelAPORT1XCH8);
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup1, adcPosSelAPORT1YCH9);
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup2, adcPosSelAPORT1XCH18);
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup2, adcPosSelAPORT1YCH19);
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup2, adcPosSelAPORT1XCH20);
		ADC_ScanSingleEndedInputAdd(&scanInit, adcScanInputGroup2, adcPosSelAPORT1YCH21);
	}

	/* Initialize for scan conversion */
	scanInit.reference = adcRefVDD;
	scanInit.prsEnable = true;
	scanInit.fifoOverwrite = false;
	ADC_InitScan(ADC0, &scanInit);

	init.timebase = ADC_TimebaseCalc(0);
	init.prescale = ADC_PrescaleCalc(ADC_CLOCK, 0);
	init.ovsRateSel = adcOvsRateSel4;
	ADC_Init(ADC0, &init);

	/* Set scan data valid level to trigger */
	ADC0->SCANCTRLX |= (ADC_SCAN_DVL - 1) << _ADC_SCANCTRLX_DVL_SHIFT;

	// Setup timer as PRS signal
	CMU_ClockEnable(cmuClock_PRS, true);
	PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_TIMER0, PRS_CH_CTRL_SIGSEL_TIMER0OF, prsEdgePos);
	timerSetup();
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
}


/* Descriptor linked list for LDMA transfer */
#define LIST0_SIZE		4
#define ADC_PRS_CHANNEL	0
#define ADC_FIFO_DEPTH	4
LDMA_Descriptor_t descLinkN0[LIST0_SIZE];
/**************************************************************************//**
 * @brief Setup LDMA to scan 8 ADC channel measurements
 * @param[in] emode2
 *   False to run ADC in EM1, True to run ADC in EM2.
 *****************************************************************************/
void ldmaSingleDescLoop(int group)
{
  uint32_t i;

  /* Fill buffer with dummy value */
  for (i=0; i<ADC_BUFFER_SIZE; i++)
  {
    adcBuffer[i] = 256;
  }

  /* Use looped peripheral transfer configuration macro */
  LDMA_TransferCfg_t periTransferTx =
        LDMA_TRANSFER_CFG_PERIPHERAL_LOOP(ldmaPeripheralSignal_ADC0_SCAN, 1);

  /* Use LINK descriptor macro for initialization and looping */
  LDMA_Descriptor_t xfer[] =
  {
  	// Number of linked descriptors depends on how many channels needs to read at a time
  	// For this example, 8 channels are read per group
    LDMA_DESCRIPTOR_LINKREL_P2M_BYTE(&ADC0->SCANDATAX, &adcBuffer, ADC_FIFO_DEPTH, 1),
    LDMA_DESCRIPTOR_SINGLE_P2M_BYTE(&ADC0->SCANDATAX, 0, ADC_FIFO_DEPTH),
  };

  descLinkN0[0] = xfer[0];
  descLinkN0[1] = xfer[1];

  /* First descriptor to initialize transfer */
  descLinkN0[0].xfer.doneIfs = 0;
  descLinkN0[0].xfer.blockSize = ldmaCtrlBlockSizeUnit4;
  descLinkN0[0].xfer.ignoreSrec = 1;
  descLinkN0[0].xfer.reqMode = ldmaCtrlReqModeAll;
  descLinkN0[0].xfer.size = ldmaCtrlSizeWord;

  /* Second descriptor for single descriptor looping */
  descLinkN0[1].xfer.decLoopCnt = 1;
  descLinkN0[1].xfer.doneIfs = 0;
  descLinkN0[1].xfer.blockSize = ldmaCtrlBlockSizeUnit4;
  descLinkN0[1].xfer.ignoreSrec = 1;
  descLinkN0[1].xfer.reqMode = ldmaCtrlReqModeAll;
  descLinkN0[1].xfer.size = ldmaCtrlSizeWord;

  /* Use relative addressing to keep destination address, stop after looping */
  descLinkN0[1].xfer.dstAddrMode = ldmaCtrlSrcAddrModeRel;
  descLinkN0[1].xfer.link = 0;
  LDMA_StartTransfer(0, (void*)&periTransferTx, (void*)&descLinkN0);

  adcTimerSetup(group);
}

/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *****************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}

/**************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/
void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
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

	/* Initialize LDMA */
	ldmaSetup();

	/* Initialize ADC */
	adcSetup();

	/* Start ADC Scan */
    ldmaSingleDescLoop(0);

	/* Wait key press to process */
	while (1)
	{
		// Wait for LDMA finish to return
		while (!LDMA_TransferDone(0));
		uint32_t sample[32], id[32];
		for (int i = 0; i < 32; i++) {
			sample[i] = 0x0000FFFF & adcBuffer[i];
			id[i] = (0x00FF0000 & adcBuffer[i]) >> 16;
		}
	}
}

/*********************************************************************
*               SEGGER MICROCONTROLLER GmbH & Co. KG                 *
*       Solutions for real time microcontroller applications         *
**********************************************************************
*                                                                    *
*       (c) 2014 - 2017  SEGGER Microcontroller GmbH & Co. KG        *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start
*/

#include <stdio.h>
#include <stdlib.h>
#if 0
/*********************************************************************
*
*       main()
*
*  Function description
*   Application entry point.
*/
void main(void) {
  int i;

  for (i = 0; i < 100; i++) {
    printf("Hello World %d!\n", i);
  }
  do {
    i++;
  } while (1);
}

/*************************** End of file ****************************/
#else
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <avr/interrupt.h>
//#include <util/delay.h>

#include "ds18b20/ds18b20.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_device.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "segmentlcd.h"
#include "rtcdriver.h"
//#include "em_dma.h"
//#include "em_prs.h"
#include "em_system.h"
#include "em_timer.h"
//#include "dmactrl.h"

#define PWM_FREQ 1000000
uint16_t topValue;

/** Timer used for bringing the system back to EM0. */
static RTCDRV_TimerID_t xTimerForWakeUp;



void init(void) {
  /* Initialize chip */
  CHIP_Init();

  MSC->READCTRL =
      //                  MSC_READCTRL_MODE_WS2SCBTP  // 2 oaeoa caaa??ee e ?ac?aoaiea i?aaauai?ee aey iaieo aaoaae ia?aoiaa
      MSC_READCTRL_MODE_WS2          // 2 oaeoa caaa??ee aac ?ac?aoaiea i?aaauai?ee aey iaieo aaoaae ia?aoiaa
      | MSC_READCTRL_AIDIS           // cai?ao aaoiiaoe?aneiai na?ina eaoa eiiaia i?e caiene ai oeao
                                     //                | MSC_READCTRL_PREFETCH       // ?ac?aoeou iia?a?a?uaa ?oaiea
                                     //                | MSC_READCTRL_RAMCEN         // ?ac?aoeou eaoe?iaaiea eiiaia a RAM
      | MSC_READCTRL_BUSSTRATEGY_DMA // o AIA i?ei?eoao ia?auiey e iao?eoa oei
                                     //                | MSC_READCTRL_BUSSTRATEGY_CPU  // o CPU i?ei?eoao ia?auiey e iao?eoa oei
      | MSC_READCTRL_IFCDIS          //Ioee??eou eyo eiiaia aey aioo?aiiae oeyo-iaiyoe
      ;

  //SystemHFXOClock = EFM32_HFXO_FREQ;
  //SystemLFXOClock = EFM32_LFXO_FREQ;
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_HFXOBOOST_MASK) | CMU_CTRL_HFXOBOOST_100PCENT;

  /* Enable HFXO as high frequency clock, HFCLK (depending on external oscillator this will probably be 32MHz) */
  CMU->OSCENCMD = CMU_OSCENCMD_HFXOEN;
  while (!(CMU->STATUS & CMU_STATUS_HFXORDY))
    ;
  CMU->CMD = CMU_CMD_HFCLKSEL_HFXO;

  /* LFXO setup */
  CMU->HFCORECLKDIV |= CMU_HFCORECLKDIV_HFCORECLKLEDIV; /* Enable DIV4 factor for peripheral clock */
  CMU->CTRL |= CMU_CTRL_HFLE;                           //High-Frequency LE Interface

  CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_LFXOBOOST_MASK) | CMU_CTRL_LFXOBOOST_70PCENT;
  EMU->AUXCTRL = (EMU->AUXCTRL & ~_EMU_AUXCTRL_REDLFXOBOOST_MASK) | EMU_AUXCTRL_REDLFXOBOOST;

  /* Enable LE clock and LFXO oscillator */
  CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_LE;
  CMU->OSCENCMD |= CMU_OSCENCMD_LFXOEN;
  /* Wait until LFXO ready */
  /* Note that this could be done more energy friendly with an interrupt in EM1 */
  while (!(CMU->STATUS & CMU_STATUS_LFXORDY))
    ;

  /* Select LFXO as clock source for LFACLK */
  CMU->LFCLKSEL = (CMU->LFCLKSEL & ~_CMU_LFCLKSEL_LFA_MASK) | CMU_LFCLKSEL_LFA_LFXO;

  /* Select LFXO as clock source for LFBCLK */
  CMU->LFCLKSEL = (CMU->LFCLKSEL & ~_CMU_LFCLKSEL_LFB_MASK) | CMU_LFCLKSEL_LFB_HFCORECLKLEDIV2;
}

void setupTimerB(void) {

  /* Enable clock for TIMER0 module */
  CMU_ClockEnable(cmuClock_TIMER3, true);
  TIMER_Reset(TIMER3);

  /* Select timer parameters */
  TIMER_Init_TypeDef timerInit =
      {
          .enable = true,
          .debugRun = false,
          .prescale = timerPrescale1, //timerPrescale64,
          .clkSel = timerClkSelHFPerClk,
          .fallAction = timerInputActionNone,
          .riseAction = timerInputActionNone,
          .mode = timerModeUp,
          .dmaClrAct = false, /* Clear DMA request when selected channel is active */
          .quadModeX4 = false,
          .oneShot = false,
          .sync = false,
      };

  //TIMER_TopSet(TIMER3, topValue);

  TIMER_Init(TIMER3, &timerInit);
  topValue = CMU_ClockFreqGet(cmuClock_HFPER) / PWM_FREQ;
}
uint16_t ticker_read(void) {
  return TIMER3->CNT;
}

void _delay_us(int us) {
  uint16_t us1 = us * topValue;
  //    uint16_t start = TIMER3->CNT;
  //    while ((TIMER3->CNT - start) < (uint16_t)us1);
  TIMER3->CNT = 0;
  while (TIMER3->CNT < (uint16_t)us1)
    ;
}

void _delay_ms(int ms) {
  for (; ms; --ms)
    _delay_us(1000);
}

volatile uint32_t interval = 10;
void gpioCallback(uint8_t pin)
{
  if (pin == 9) {
    //BSP_LedToggle(1);
    if(++interval>9999) interval=9999;
  } else if (pin == 10) {
    if(--interval<=0) interval=1;
    //BSP_LedToggle(0);
  }
}

/***************************************************************************//**
 * @brief Setup GPIO interrupt to change temp. display
 ******************************************************************************/
void gpioSetup(void)
{
  /* Enable GPIO in CMU */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Initialize GPIO interrupt dispatcher */
//  GPIOINT_Init();

  /* Configure PB9 and PB10 as input */
  GPIO_PinModeSet(gpioPortB, 9, gpioModeInput, 0);
  GPIO_PinModeSet(gpioPortB, 10, gpioModeInput, 0);

  /* Register callbacks before setting up and enabling pin interrupt. */
  GPIOINT_CallbackRegister(9,  gpioCallback);
  GPIOINT_CallbackRegister(10, gpioCallback);

  /* Set falling edge interrupt for both ports */
  GPIO_IntConfig(gpioPortB, 9, false, true, true);
  GPIO_IntConfig(gpioPortB, 10, false, true, true);

  /* Enable interrupt in core for even and odd gpio interrupts */
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

}



volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;


int main(void) {
  char printbuff[100];
  int i;
  double d = 0;

  init();
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_PinModeSet(DS18B20_PORT, DS18B20_DQ, gpioModeWiredAnd, 1);
  setupTimerB();

  /* Initialize RTC timer. */
  RTCDRV_Init();
  RTCDRV_AllocateTimer(&xTimerForWakeUp);

  /* Initialize LCD controller without boost */
  SegmentLCD_Init(false);
  SegmentLCD_AllOff();

  /* Enable board control interrupts (key) */
  gpioSetup();

#if 0
	for (;;) {
          GPIO->P[DS18B20_PORT].DOUTSET = 1 << DS18B20_DQ;
          _delay_us(1);
          GPIO->P[DS18B20_PORT].DOUTCLR = 1 << DS18B20_DQ;
          _delay_us(1);
        }
#endif
  for (;;) {
    d = ds18b20_gettemp();

    printf("%8.2f\n", d);

    //_delay_ms(500);

    /* Show Celsius on alphanumeric part of display */
    i = (int)(d * 10);
    snprintf(printbuff, 8, "%2d,%1d%%C", (i / 10), abs(i % 10));
    /* Show Fahrenheit on numeric part of display */
    //i = (int)(convertToFahrenheit(temp) * 10);
    //SegmentLCD_Number(i * 10);
    SegmentLCD_Symbol(LCD_SYMBOL_DP10, 1);
    SegmentLCD_Symbol(LCD_SYMBOL_DEGC, 0);
    SegmentLCD_Symbol(LCD_SYMBOL_DEGF, 1);
    SegmentLCD_Number(interval);

    SegmentLCD_Write(printbuff);

    /* Sleep for 2 seconds in EM 2 */
    RTCDRV_StartTimer(xTimerForWakeUp, rtcdrvTimerTypeOneshot, interval*1000, NULL, NULL);
    EMU_EnterEM2(true);
  }
  return 0;
}
#endif
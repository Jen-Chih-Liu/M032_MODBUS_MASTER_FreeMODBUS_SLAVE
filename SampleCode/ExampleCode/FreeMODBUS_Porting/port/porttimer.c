/**************************************************************************//**
 * @file     portserial .c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/06/08 7:28p $
 * @brief    port time function for FreeMODBUS
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/* ----------------------- Platform includes --------------------------------*/
#include "NuMicro.h"
#include "port.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"


/* ----------------------- Start implementation -----------------------------*/
/**
  * @brief      xMBPortTimersInit
  * @param[in]  USHORT usTim1Timerout50us
  * @return     TURE or FALSE
  * @details    This function to Open UART port and set ulBaudRate, ucDataBits, eParity.
  */
BOOL xMBPortTimersInit(USHORT usTim1Timerout50us)
{
    float freq_exp = 0;
    freq_exp = 1.0 / ((float)usTim1Timerout50us * 0.00005);//convert time to frequence

    /* Enable Timer0 clock source */
    CLK_EnableModuleClock(TMR0_MODULE);
    /* Select Timer0 clock source as internal 48M */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Reset IP TMR0 */
    SYS_ResetModule(TMR0_RST);

    /* Open Timer0 in periodic mode, enable interrupt and interrupt tick per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, (uint32_t)freq_exp);

    /* Enable Timer0 interrupt */
    TIMER_EnableInt(TIMER0);
    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);
    return TRUE;
}

/**
  * @brief      vMBPortTimersEnable
  * @param[in]  None
  * @return     None
  * @details    This function to enable timer and time count reset.
  */
void vMBPortTimersEnable(void)
{
    /* Reset Timer0 counter */
    TIMER_ResetCounter(TIMER0);
    /* Enable Timer0 */
    TIMER_Start(TIMER0);
}

/**
  * @brief      vMBPortTimersDisable
  * @param[in]  None
  * @return     None
  * @details    This function to disable timer and time count reset.
  */
void vMBPortTimersDisable(void)
{
    /* Disable Timer0 */
    TIMER_Stop(TIMER0);
    /* Reset Timer0 counter */
    TIMER_ResetCounter(TIMER0);
}

/**
  * @brief      TMR0_IRQHandler
  * @param[in]  None
  * @return     None
  * @details    This function is timer interrupt handler
  */
void TMR0_IRQHandler(void)
{
    /* Clear Timer0 interrupt flag */
    TIMER_ClearIntFlag(TIMER0);
    (void)pxMBPortCBTimerExpired();
}

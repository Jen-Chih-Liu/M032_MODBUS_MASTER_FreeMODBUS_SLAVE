/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    The modbus master demo code
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "stdio.h"
#include "NuMicro.h"
#include "modbus_master.h"
/*---------------------------------------------------------------------------*/
/* Global variables                                                          */
/*---------------------------------------------------------------------------*/
uint16_t  Input_Result[2];
volatile uint32_t u32_TimeTick = 0;

/*---------------------------------------------------------------------------*/
/* Functions                                                                 */
/*---------------------------------------------------------------------------*/
void HAL_Delay(uint32_t Delay);
void UART0_Init(void);
void SYS_Init(void);
void Timer_Init(void);
extern void Modbus_Master_Rece_Handler(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf and test */
    UART0_Init();

    /* Init timer function */
    Timer_Init();

    ModbusMaster_begin();

    while (1)
    {
        uint8_t result;
        result = ModbusMaster_readInputRegisters(0x0a, 0x3E7, 0X4); //master send readInputRegisters command to slave

        if (result == 0x00)
        {
            //user can read input register  from adress 0x02 and get 2 byte context
            Input_Result[0] = ModbusMaster_getResponseBuffer(0x00);
            Input_Result[1] = ModbusMaster_getResponseBuffer(0x01);
        }

        HAL_Delay(1000);
    }

}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();
}

void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 38400);

    /* Enable UART RDA Interrupt */
    UART0->INTEN |= UART_INTEN_RDAIEN_Msk;

    /* Enable UART02 NVIC */
    NVIC_EnableIRQ(UART02_IRQn);
}

void Timer_Init(void)
{
    /* Open Timer0 in periodic mode, enable interrupt and 1000 interrupt tick per second */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);

    /* Enable Timer0 interrupt */
    TIMER_EnableInt(TIMER0);

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Start Timer0 counting */
    TIMER_Start(TIMER0);
}

void TMR0_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        u32_TimeTick++;
    }
}

uint32_t HAL_GetTick(void)
{
    return u32_TimeTick;
}

//This function provides accurate delay (in milliseconds) based on variable incremented.
void HAL_Delay(uint32_t Delay)
{
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = Delay;

    while ((HAL_GetTick() - tickstart) < wait)
    {
    }
}

void UART02_IRQHandler(void)
{
    uint32_t u32IntSts = UART0->INTSTS;

    if (u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        Modbus_Master_Rece_Handler();
    }
}



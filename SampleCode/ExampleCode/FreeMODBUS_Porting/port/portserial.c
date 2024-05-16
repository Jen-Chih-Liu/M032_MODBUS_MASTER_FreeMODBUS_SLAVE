/**************************************************************************//**
 * @file     portserial .c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/06/08 7:28p $
 * @brief    port serial function for FreeMODBUS
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "NuMicro.h"
#include "port.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

//static void     prvvUARTTxReadyISR(void);
//static void     prvvUARTRxISR(void);

/* ----------------------- Start implementation -----------------------------*/
/**
  * @brief      vMBPortSerialEnable
  * @param[in]  BOOL xRxEnable, BOOL xTxEnable
  * @return     None
  * @details    This function to enable/disabele UART TX and RX interrupt.
  */
void  vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
    __disable_irq();

    if (xRxEnable)
    {
        UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk);
    }
    else
    {
        UART_DISABLE_INT(UART0, UART_INTEN_RDAIEN_Msk);
    }

    if (xTxEnable)
    {
        UART_ENABLE_INT(UART0, UART_INTEN_THREIEN_Msk);
    }
    else
    {
        UART_DISABLE_INT(UART0, UART_INTEN_THREIEN_Msk);
    }

    NVIC_EnableIRQ(UART02_IRQn);
    __enable_irq();
}
/**
  * @brief      vMBPortClose
  * @param[in]  None
  * @return     None
  * @details    This function to Close UART port.
  */
void vMBPortClose(void)
{
    /* Disable Interrupt */
    UART_DISABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk);
    NVIC_DisableIRQ(UART02_IRQn);
}

/**
  * @brief      xMBPortSerialInit
  * @param[in]  UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity
  * @return     TURE or FALSE
  * @details    This function to Open UART port and set ulBaudRate, ucDataBits, eParity.
  */
BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity)
{
    if (ucPORT != 0) return FALSE;

    /* Step 1. GPIO initial */
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Step 2. Enable and Select UART clock source*/
    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* UART0 clock source */
    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Step 3. Select Operation BaudRate */
    SYS_ResetModule(UART0_RST);
    UART_Open(UART0, ulBaudRate);

    /* Step 4. Set Configure parity*/
    switch (eParity)
    {
        case MB_PAR_NONE:
            UART0->LINE &= ~UART_PARITY_EVEN;
            break;

        case MB_PAR_EVEN:
            UART0->LINE &= ~UART_PARITY_EVEN;
            UART0->LINE |= UART_PARITY_EVEN;
            break;

        case MB_PAR_ODD:
            UART0->LINE &= ~UART_PARITY_EVEN;
            UART0->LINE |= UART_PARITY_ODD;
            break;
    }

    UART0->LINE &= ~UART_WORD_LEN_8;

    switch (ucDataBits)
    {
        case 5:
            UART0->LINE |= UART_WORD_LEN_5;      //5 bits Data Length
            break;

        case 6:
            UART0->LINE |= UART_WORD_LEN_6;      //6 bits Data Length
            break;

        case 7:
            UART0->LINE |= UART_WORD_LEN_7;      //7 bits Data Length
            break;

        case 8:
            UART0->LINE |= UART_WORD_LEN_8;      //8 bits Data Length
            break;
    }

    return TRUE;

}
/**
  * @brief      xMBPortSerialPutByte
  * @param[in]  CHAR ucByte
  * @return     TURE or FALSE
  * @details    This function to send ucByte to UART TX buffer.
  */
BOOL xMBPortSerialPutByte(CHAR ucByte)
{

    UART0->DAT = (uint8_t) ucByte;

    while (UART_GET_TX_EMPTY(UART0) != 0x00); //check Tx Empty

    return TRUE;
}
/**
  * @brief      xMBPortSerialGetByte
  * @param[in]  CHAR *pucByte
  * @return     TURE or FALSE
  * @details    This function to get pucByte from UART RX buffer.
  */
BOOL xMBPortSerialGetByte(CHAR *pucByte)
{

    while (UART_GET_RX_EMPTY(UART0) != 0x00); //check Rx Empty

    *pucByte = UART0->DAT;
    return TRUE;
}

/**
  * @brief      UART02_IRQHandler
  * @param[in]  None
  * @return     None
  * @details    This function is UART TX and RX interrupt handler function.
  */
void UART02_IRQHandler(void)
{

    if (UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk))
    {

        /* Get all the input characters */
        while (UART_IS_RX_READY(UART0))
        {
            //  prvvUARTRxISR();
            pxMBFrameCBByteReceived();
        }

    }
    else if (UART_GET_INT_FLAG(UART0, UART_INTSTS_THREINT_Msk))
    {

        //prvvUARTTxReadyISR();
        pxMBFrameCBTransmitterEmpty();
    }
}



/*
 * Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call
 * xMBPortSerialPutByte( ) to send the character.
 */
#if 0
static void prvvUARTTxReadyISR(void)
{
    pxMBFrameCBTransmitterEmpty();
}
#endif

/*
 * Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
#if 0
static void prvvUARTRxISR(void)
{
    pxMBFrameCBByteReceived();
}
#endif

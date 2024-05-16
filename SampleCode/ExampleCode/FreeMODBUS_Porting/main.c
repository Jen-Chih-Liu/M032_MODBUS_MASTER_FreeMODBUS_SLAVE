/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/06/08 7:28p $
 * @brief    M031 FreeMODBUS Porting.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 4


/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];

int32_t main()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable the Modbus RTU mode, modbus slave Address, UART port, BaudRate, Parity*/
    eMBInit(MB_RTU, 0x0A, 0, 38400, MB_PAR_NONE);

    /* Enable the Modbus Protocol Stack. */
    eMBEnable();

    for (;;)
    {
        /*eMBPoll for modbus event process*/
        (void)eMBPoll();

        /* Here we simply count the number of poll cycles. */
        usRegInputBuf[0]++;
    }
}


eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if ((usAddress >= REG_INPUT_START)
            && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
    {
        iRegIndex = (int)(usAddress - usRegInputStart);

        while (usNRegs > 0)
        {
            *pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] >> 8);
            *pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] & 0xFF);
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    (void)pucRegBuffer;
    (void)usAddress;
    (void)usNRegs;
    (void)eMode;
    return MB_ENOREG;
}


eMBErrorCode eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
    (void)pucRegBuffer;
    (void)usAddress;
    (void)usNCoils;
    (void)eMode;
    return MB_ENOREG;
}

eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
    (void)pucRegBuffer;
    (void)usAddress;
    (void)usNDiscrete;
    return MB_ENOREG;
}

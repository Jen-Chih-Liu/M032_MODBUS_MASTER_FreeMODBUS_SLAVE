/**************************************************************************//**
 * @file     portevent.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/06/08 7:28p $
 * @brief    port event function for FreeMODBUS
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Variables ----------------------------------------*/
static eMBEventType eQueuedEvent;
static BOOL     xEventInQueue;

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortEventInit(void)
{
    xEventInQueue = FALSE;
    return TRUE;
}

BOOL xMBPortEventPost(eMBEventType eEvent)
{
    xEventInQueue = TRUE;
    eQueuedEvent = eEvent;
    return TRUE;
}

BOOL xMBPortEventGet(eMBEventType *eEvent)
{
    BOOL            xEventHappened = FALSE;

    if (xEventInQueue)
    {
        *eEvent = eQueuedEvent;
        xEventInQueue = FALSE;
        xEventHappened = TRUE;
    }

    return xEventHappened;
}



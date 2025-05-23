/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for CM7_0 in the MultiCore IPC Pipes
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* IPC channel number */
#define USED_IPC_CHANNEL        7

/* Notify interrupt number. This interrupt is handled by notified core (CM0+) */
#define IPC_NOTIFY_INT_NUMBER   7

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for the CM7_0 core.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    uint32_t  resultA= 0;
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* At first force release the lock state. */
    (void)Cy_IPC_Drv_LockRelease(Cy_IPC_Drv_GetIpcBaseAddress(USED_IPC_CHANNEL), CY_IPC_NO_NOTIFICATION);

    /* Note: Wait until the CM0+ IPC server is started
     * After the CM0+ IPC server is started, the corresponding number of the INTR_MASK is set.
     * So in this case CM7 can recognize whether the server has started or not by the INTR_MASK status.
     */
    while (((1uL << (USED_IPC_CHANNEL)) != Cy_IPC_Drv_ExtractAcquireMask(Cy_IPC_Drv_GetInterruptMask(Cy_IPC_Drv_GetIntrBaseAddr(USED_IPC_CHANNEL)))))
    {
    }

	/* Initialize ADC */
	{
		/* Initialize the SAR2 module */
		Cy_SAR2_Init(ADC0_HW, &ADC0_config);
		/* Set ePASS MMIO reference buffer mode for bangap voltage */
		Cy_SAR2_SetReferenceBufferMode(PASS0_EPASS_MMIO, CY_SAR2_REF_BUF_MODE_OFF);
		/* Issue software start trigger */
		Cy_SAR2_Channel_SoftwareTrigger(ADC0_HW, 0);
	}


    for (;;)
    {
        /* Wait 0.5 [s] */
        //Cy_SysLib_Delay(100);

        /* Send the message to the M0+ through IPC */
        Cy_IPC_Drv_SendMsgWord(Cy_IPC_Drv_GetIpcBaseAddress(USED_IPC_CHANNEL),(1u << IPC_NOTIFY_INT_NUMBER),resultA);

        /* Wait until the CM0+ get the message and release the lock. */
        bool status = true;
        do
        {
            status = Cy_IPC_Drv_IsLockAcquired(Cy_IPC_Drv_GetIpcBaseAddress(USED_IPC_CHANNEL));
        } while(status);

        resultA= Cy_SAR2_Channel_GetResult(ADC0_HW, 0, NULL);

        resultA = (resultA );
    }
}

/* [] END OF FILE */

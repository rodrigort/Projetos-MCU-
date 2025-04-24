/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for Hello World Example using HAL APIs.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2022-2024, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_gpio.h"



/*******************************************************************************
 * Global Variables
 *******************************************************************************/
bool timer_interrupt_flag = false;
bool led_blink_active_flag = true;

/* Variable for storing character read from terminal */
uint8_t uart_read_value;

uint32_t  resultA= 0;


/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * This is the main function. It sets up a timer to trigger a periodic interrupt.
 * The main while loop checks for the status of a flag set by the interrupt and
 * toggles an LED at 1Hz to create an LED blinky. Will be achieving the 1Hz Blink
 * rate based on the The LED_BLINK_TIMER_CLOCK_HZ and LED_BLINK_TIMER_PERIOD
 * Macros,i.e. (LED_BLINK_TIMER_PERIOD + 1) / LED_BLINK_TIMER_CLOCK_HZ = X ,Here,
 * X denotes the desired blink rate. The while loop also checks whether the
 * 'Enter' key was pressed and stops/restarts LED blinking.
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
	cy_rslt_t result;

	/* Initialize the device and board peripherals */
	result = cybsp_init();

	/* Board init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	/* Enable global interrupts */
	__enable_irq();

	/* Initialize retarget-io to use the debug UART port */
	result = cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
			CYBSP_DEBUG_UART_CTS,CYBSP_DEBUG_UART_RTS,CY_RETARGET_IO_BAUDRATE);

	/* Initialize ADC */
	{
		/* Initialize the SAR2 module */
		Cy_SAR2_Init(ADC0_HW, &ADC0_config);
		/* Set ePASS MMIO reference buffer mode for bangap voltage */
		Cy_SAR2_SetReferenceBufferMode(PASS0_EPASS_MMIO, CY_SAR2_REF_BUF_MODE_OFF);
		/* Issue software start trigger */
		Cy_SAR2_Channel_SoftwareTrigger(ADC0_HW, 0);
	}


	/* retarget-io init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	/* GPIO init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	printf("\x1b[2J\x1b[;H");

	printf("****************** ""Exemplo ADC LED " "****************** \r\n\n");

	for (;;)
	{

		resultA = Cy_SAR2_Channel_GetResult(ADC0_HW, 0, NULL);

		if(resultA >= 2098)
		{
			Cy_GPIO_Write(GPIO_PRT5, 0U, 0);    /* Led 1 Off */
			printf("ADC Em nivel alto");
		}
		else
		{
			Cy_GPIO_Write(GPIO_PRT5, 0U, 1);    /* Led 1 Off */
		}


		if (Cy_GPIO_Read(GPIO_PRT5, 3u) == 0)
		{
			Cy_GPIO_Write(GPIO_PRT5, 2u, 0);    /* Led 1 Off */
		}
		else
		{
			Cy_GPIO_Write(GPIO_PRT5, 2u, 1);    /* Led 1 Off */
		}


	}
}

/* [] END OF FILE */

/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
* Header Files
*******************************************************************************/
/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cycfg_peripherals.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "stdio.h"

#define AHT25_ADDR 0x38       // I2C address of the AHT25 sensor
#define I2C_TIMEOUT 100       // Timeout for I2C operations (in milliseconds)
#define CMD_TO_CMD_DELAY 80   // Delay time for sensor processing (in milliseconds)

/* Definition of the I2C context */
float temperature = 0, humidity = 0;
cy_stc_scb_i2c_context_t i2c_context;  // I2C driver context

/* Definition of I2C interrupt number and priority */
#define I2C_INTR_NUM        scb_4_interrupt_IRQn
#define I2C_INTR_PRIORITY   (7UL)

/* Configuration of the I2C interrupt */
const cy_stc_sysint_t i2cIntrConfig =
{
    .intrSrc      = I2C_INTR_NUM,    // Assign interrupt source
    .intrPriority = I2C_INTR_PRIORITY,  // Set interrupt priority
};

/* I2C Interrupt Handler */
void I2C_Isr(void)
{
    Cy_SCB_I2C_MasterInterrupt(I2C_HW, &i2c_context);  // Calls only the Master mode interrupt
}

/* Initializes the I2C correctly with interrupt handling */
void init_i2c(void)
{
    // Initialize the I2C peripheral with the given configuration
    Cy_SCB_I2C_Init(I2C_HW, &I2C_config, &i2c_context);

    // Initialize and enable the I2C interrupt
    Cy_SysInt_Init(&i2cIntrConfig, I2C_Isr);
    NVIC_EnableIRQ((IRQn_Type) I2C_INTR_NUM);

    // Enable the I2C module
    Cy_SCB_I2C_Enable(I2C_HW);
    
    // Enable global interrupts
    __enable_irq();
}

/* Checks if the AHT25 sensor is connected */
bool check_aht25(void)
{
    // Send a START condition to verify sensor response
    cy_en_scb_i2c_status_t status = Cy_SCB_I2C_MasterSendStart(
        I2C_HW, AHT25_ADDR, CY_SCB_I2C_WRITE_XFER, I2C_TIMEOUT, &i2c_context
    );

    // Send a STOP condition to end communication
    Cy_SCB_I2C_MasterSendStop(I2C_HW, I2C_TIMEOUT, &i2c_context);

    // Return true if the sensor responded correctly
    return (status == CY_SCB_I2C_SUCCESS);
}

/* Reads temperature and humidity data from the AHT25 sensor using High-Level Functions */
void read_aht25(float *temperature, float *humidity)
{
    uint8_t cmd[3] = {0xAC, 0x33, 0x00}; // Command to start measurement
    uint8_t data[7] = {0};  // Buffer to store received data

    /* Configure I2C write transaction (without STOP) */
    cy_stc_scb_i2c_master_xfer_config_t writeXferConfig = {
        .slaveAddress = AHT25_ADDR,
        .buffer = cmd,
        .bufferSize = sizeof(cmd),
        .xferPending = true  // Do not generate STOP to allow reading immediately after
    };

    /* Configure I2C read transaction (with STOP) */
    cy_stc_scb_i2c_master_xfer_config_t readXferConfig = {
        .slaveAddress = AHT25_ADDR,
        .buffer = data,
        .bufferSize = sizeof(data),
        .xferPending = false  // Generate STOP at the end
    };

    /* Start the write transaction */
    (void) Cy_SCB_I2C_MasterWrite(I2C_HW, &writeXferConfig, &i2c_context);

    /* Wait for the interrupt to process the transaction */
    while (Cy_SCB_I2C_MasterGetStatus(I2C_HW, &i2c_context) & CY_SCB_I2C_MASTER_BUSY) {}

    /* Wait the necessary time for the sensor to process the measurement */
    // Cy_SysLib_DelayUs(CMD_TO_CMD_DELAY);

    /* Start the read transaction */
    (void) Cy_SCB_I2C_MasterRead(I2C_HW, &readXferConfig, &i2c_context);

    /* Wait for the interrupt to process the transaction */
    while (Cy_SCB_I2C_MasterGetStatus(I2C_HW, &i2c_context) & CY_SCB_I2C_MASTER_BUSY) {}

    /* Process humidity data */
    uint32_t raw_humidity = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4);
    *humidity = ((float)raw_humidity / 0x100000) * 100;

    /* Process temperature data */
    uint32_t raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];
    *temperature = ((float)raw_temperature / 0x100000) * 200 - 50;
}

/* Main function */
int main(void)
{
    cy_rslt_t result;

    // Initialize the hardware
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    // Initialize the UART interface for debugging
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, 115200);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    // Clear the terminal screen and display an initialization message
    printf("\x1b[2J\x1b[;H");
    printf("Initializing AHT25...\n\r");

    // Initialize the I2C communication
    init_i2c();

    // Check if the AHT25 sensor is connected
    if (!check_aht25())
    {
        printf("Error: AHT25 not detected!\n\r");
        return 1;
    }

    // Main loop
    for (;;)
    {
        // Read temperature and humidity
        read_aht25(&temperature, &humidity);

        // Display the values read
        printf("Temperature: %.2f °C, Humidity: %.2f%%\n\r", temperature, humidity);

        // Wait 1 second before the next reading
        Cy_SysLib_Delay(1000);
    }
}


/* [] END OF FILE */

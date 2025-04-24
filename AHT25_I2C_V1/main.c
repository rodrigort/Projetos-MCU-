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
#if defined (CY_USING_HAL)
#include "cyhal.h"
#endif
#include "cybsp.h"
#include "cy_retarget_io.h" // Biblioteca para redirecionamento de I/O (uso de printf via UART)
#include <stdio.h> // Biblioteca padrão para entrada e saída

/******************************************************************************
 * Macros
 *******************************************************************************/
#define AHT25_ADDR 0x38  // Endereço I2C do sensor AHT25
//#define I2C_SPEED  100000 // Frequência desejada para a comunicação I2C (100kHz)

/*******************************************************************************
 * Variáveis Globais
 *******************************************************************************/
cy_stc_scb_i2c_context_t i2c_context; // Estrutura de contexto do I2C usada pelo driver

/*******************************************************************************
 * Protótipos de Funções
 *******************************************************************************/
void init_i2c(void); // Função para inicializar o I2C
bool check_aht25(void); // Função para verificar a presença do sensor no barramento I2C
void read_aht25(float *temperature, float *humidity); // Função para ler temperatura e umidade do AHT25

/*******************************************************************************
 * Funções
 *******************************************************************************/

// 🔧 Inicializa o I2C com PDL
void init_i2c(void)
{
	Cy_SCB_I2C_Init(I2C_HW, &I2C_config, &i2c_context); // Inicializa o periférico I2C
	Cy_SCB_I2C_Enable(I2C_HW); // Habilita o periférico I2C para comunicação
}

// 🔧 Testa se o AHT25 está respondendo no barramento I2C
bool check_aht25(void)
{
	cy_en_scb_i2c_status_t status = Cy_SCB_I2C_MasterSendStart(I2C_HW, AHT25_ADDR, CY_SCB_I2C_WRITE_XFER, 100, &i2c_context); // Envia um sinal de início I2C para testar a resposta do sensor
	Cy_SCB_I2C_MasterSendStop(I2C_HW, 100, &i2c_context); // Envia um sinal de parada
	return (status == CY_SCB_I2C_SUCCESS); // Retorna true se o sensor responder corretamente
}

// 🔧 Lê temperatura e umidade do AHT25
void read_aht25(float *temperature, float *humidity)
{
	uint8_t cmd[3] = {0xAC, 0x33, 0x00}; // Comando necessário para iniciar a medição no AHT25
	uint8_t data[7] = {0}; // Buffer para armazenar os dados lidos

	// Envia comando de medição para o sensor
	Cy_SCB_I2C_MasterSendStart(I2C_HW, AHT25_ADDR, CY_SCB_I2C_WRITE_XFER, 100, &i2c_context);
	for (int i = 0; i < 3; i++)
	{
		Cy_SCB_I2C_MasterWriteByte(I2C_HW, cmd[i], 100, &i2c_context); // Envia os bytes do comando ao sensor
	}
	Cy_SCB_I2C_MasterSendStop(I2C_HW, 100, &i2c_context);

	Cy_SysLib_DelayUs(80); // Aguarda 80 microssegundos para a conversão dos dados pelo sensor

	// Lê os 7 bytes do sensor contendo temperatura e umidade
	Cy_SCB_I2C_MasterSendStart(I2C_HW, AHT25_ADDR, CY_SCB_I2C_READ_XFER, 100, &i2c_context);
	for (int i = 0; i < 6; i++)
	{
		Cy_SCB_I2C_MasterReadByte(I2C_HW, CY_SCB_I2C_ACK, &data[i], 100, &i2c_context); // Lê os primeiros 6 bytes e envia ACK
	}
	Cy_SCB_I2C_MasterReadByte(I2C_HW, CY_SCB_I2C_NAK, &data[6], 100, &i2c_context); // Lê o último byte e envia NAK
	Cy_SCB_I2C_MasterSendStop(I2C_HW, 100, &i2c_context);

	// Processa os dados de umidade recebidos do sensor
	uint32_t raw_humidity = data[1];
	raw_humidity = (raw_humidity << 8) | data[2];
	raw_humidity = (raw_humidity << 4) | (data[3] >> 4);
	if (raw_humidity > 0x100000) raw_humidity = 0x100000; // Limita a umidade a 100%
	*humidity = ((float)raw_humidity / 0x100000) * 100; // Converte a umidade para porcentagem

	// Processa os dados de temperatura recebidos do sensor
	uint32_t raw_temperature = data[3] & 0x0F;
	raw_temperature = (raw_temperature << 8) | data[4];
	raw_temperature = (raw_temperature << 8) | data[5];
	*temperature = ((float)raw_temperature / 0x100000) * 200 - 50; // Converte a temperatura para graus Celsius
}

/*******************************************************************************
 * Função principal
 *******************************************************************************/
int main(void)
{
	cy_rslt_t result;
	result = cybsp_init(); // Inicializa o hardware da placa
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0); // Se falhar, para a execução
	}

	result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, 115200); // Inicializa a comunicação UART para depuração
	CY_ASSERT(result == CY_RSLT_SUCCESS);

	/* \x1b[2J\x1b[;H - ANSI ESC sequencia para limpar a tela*/
	printf("\x1b[2J\x1b[;H");

	printf("Inicializando AHT25...\n\r");

	init_i2c(); // Inicializa o I2C
	if (!check_aht25())
	{
		printf("Erro: AHT25 não detectado!\n\r");
		return 1; // Encerra o programa se o sensor não for encontrado
	}

	float temperature = 0, humidity = 0; // Variáveis para armazenar os valores lidos
	for (;;)
	{
		read_aht25(&temperature, &humidity); // Lê temperatura e umidade do sensor
		printf("Temperatura: %.2f °C, Umidade: %.2f%%\n\r", temperature, humidity); // Exibe os valores no console
		Cy_SysLib_Delay(1000); // Aguarda 2 segundos antes da próxima leitura
	}
}

/* [] END OF FILE */

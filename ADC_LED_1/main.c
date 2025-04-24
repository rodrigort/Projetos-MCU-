/*******************************************************************************
 * File Name:   main.c
 *******************************************************************************/

#include "cybsp.h"                 // Biblioteca para inicialização do hardware
#include "cy_retarget_io.h"        // Biblioteca para comunicação serial
#include "cy_em_eeprom.h"          // Biblioteca para emulação de EEPROM
#include "cycfg_system.h"          // Configuração do sistema gerada automaticamente

/* Macros */
#define LOGICAL_EEPROM_SIZE     (2u)   // Tamanho lógico da EEPROM (15 bytes)
#define LOGICAL_EEPROM_START    (0u)   // Endereço inicial lógico na EEPROM
#define RESET_COUNT_LOCATION    (0u)   // Localização do contador de reinicialização
#define RESET_COUNT_SIZE        (2u)   // Tamanho do contador de reinicialização (2 bytes)

/* Global Variables */
uint16_t adcValue = 0;            // Variável para armazenar o valor do ADC
uint16_t savedAdcValue = 0;       // Variável para armazenar o valor lido da EEPROM

/* Configuração da EEPROM */
cy_stc_eeprom_config_t Em_EEPROM_config = 
  {
    .eepromSize = MY_EEPROM_SIZE,               // Tamanho da EEPROM
    .simpleMode = MY_EEPROM_SIMPLEMODE,         // Modo simples habilitado/desabilitado
    .blockingWrite = MY_EEPROM_BLOCKINGMODE,    // Modo de gravação bloqueante
    .redundantCopy = MY_EEPROM_REDUNDANT_COPY,  // Cópia redundante habilitada/desabilitada
    .wearLevelingFactor = MY_EEPROM_WEARLEVELING_FACTOR, // Fator de nivelamento de desgaste
    .userFlashStartAddr = CY_WFLASH_LG_SBM_BASE, // Endereço de início do flash
  };

cy_stc_eeprom_context_t Em_EEPROM_context; // Contexto necessário para a emulação da EEPROM

/* Arrays para leitura e escrita na EEPROM */
uint8_t eepromReadArray[LOGICAL_EEPROM_SIZE]= {0};   // Array para leitura da EEPROM
uint8_t eepromWriteArray[LOGICAL_EEPROM_SIZE] = {0};  // Array para escrita na EEPROM

/* Função principal */
int main(void)
  {
  cy_rslt_t result; // Variável para armazenar o resultado das operações

  /* Inicializa o hardware da placa */
  result = cybsp_init();
  if (result != CY_RSLT_SUCCESS)
    {
    printf("Erro: inicialização falhou\r\n"); // Exibe erro caso a inicialização falhe
    CY_ASSERT(0);  // Interrompe a execução para depuração
    }

  /* Inicializa o redirecionamento de IO para comunicação serial */
  cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,CY_RETARGET_IO_BAUDRATE);

  /* Inicializa a EEPROM */
  result = Cy_Em_EEPROM_Init(&Em_EEPROM_config, &Em_EEPROM_context);

  if (result != CY_EM_EEPROM_SUCCESS)
    {
    printf("Erro: inicialização da EEPROM falhou\r\n"); // Exibe erro caso a inicialização falhe
    CY_ASSERT(0);  // Interrompe a execução para depuração
    }

  /* Inicializa o ADC */
  Cy_SAR2_Init(ADC0_HW, &ADC0_config);  // Configura o ADC
  Cy_SAR2_SetReferenceBufferMode(PASS0_EPASS_MMIO, CY_SAR2_REF_BUF_MODE_OFF); // Configura referência do ADC
  Cy_SAR2_Channel_SoftwareTrigger(ADC0_HW, 0);  // Dispara a leitura do ADC

  printf("\x1b[2J\x1b[;H"); // Limpa o terminal para exibição organizada
  printf("****************** Exemplo ADC EEPROM ****************** \r\n\n");

  /* Habilita interrupções globais */
  __enable_irq();

  /* Loop principal */
  for (;;)
    {
    /* Captura valor do ADC */
    adcValue = Cy_SAR2_Channel_GetResult(ADC0_HW, 0, NULL);

    /* Botão 1 pressionado */
    if (Cy_GPIO_Read(GPIO_PRT5, 3u) == 0)
      {
      printf("Botão 1 pressionado! Salvando valor do ADC: %u\r\n", adcValue);

      /* Armazena o valor do ADC no array para escrita */
      eepromWriteArray[0] = (adcValue & 0xFF);         // Armazena o byte menos significativo
      eepromWriteArray[1] = ((adcValue >> 8) & 0xFF);  // Armazena o byte mais significativo

      /* Grava na EEPROM */
      result = Cy_Em_EEPROM_Write(LOGICAL_EEPROM_START, eepromWriteArray,
                                  LOGICAL_EEPROM_SIZE, &Em_EEPROM_context);
      if (result != CY_EM_EEPROM_SUCCESS)
        {
        printf("Erro ao salvar na EEPROM! Código: 0x%08lX\r\n",
               (unsigned long)result); // Exibe o código de erro
        CY_ASSERT(0); // Interrompe a execução para depuração
        }
      Cy_SysLib_Delay(200); // Atraso para evitar múltiplas gravações
      }

    /* Botão 2 pressionado */
    if (Cy_GPIO_Read(GPIO_PRT17, 0u) == 0)
      {
      printf("Botão 2 pressionado! Lendo valor salvo na EEPROM...\r\n");

      /* Lê da EEPROM */
      result = Cy_Em_EEPROM_Read(LOGICAL_EEPROM_START, eepromReadArray,
                                 LOGICAL_EEPROM_SIZE, &Em_EEPROM_context);
      if (result != CY_EM_EEPROM_SUCCESS)
        {
        printf("Erro ao ler da EEPROM! Código: 0x%08lX\r\n",
               (unsigned long)result); // Exibe o código de erro
        CY_ASSERT(0); // Interrompe a execução para depuração
        }
      else
        {
        /* Recupera o valor do ADC salvo */
        savedAdcValue = 
          (uint16_t)eepromReadArray[0] | ((uint16_t)eepromReadArray[1] << 8);
        printf("Valor lido da EEPROM: %u\r\n", savedAdcValue); // Exibe o valor salvo
        }
      Cy_SysLib_Delay(200); // Atraso para evitar múltiplas leituras
      }

    /* Indicador LED para nível de ADC */
    if (adcValue >= 2098)
      {
      Cy_GPIO_Write(GPIO_PRT5, 0U, 0); // Liga o LED 1
      }
    else
      {
      Cy_GPIO_Write(GPIO_PRT5, 0U, 1); // Desliga o LED 1
      }
    }
  }

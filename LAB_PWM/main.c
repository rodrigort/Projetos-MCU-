/*******************************************************************************
 * Header Files
 *******************************************************************************/
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cycfg_peripherals.h"
#include <stdint.h>

/*******************************************************************************
 * Macros
 *******************************************************************************/
#define PWM_FREQUENCY_HZ (1000u)         // Frequência do PWM em Hz
#define N_1 500                          // Número de pontos para a média móvel
#define NUM_CANAIS 1                     // Quantidade de canais de ADC
#define ADC_12_BIT 4094                  // Nome do bloco de hardware do ADC
#define OVERPLOW_PWM 49990               // Número de overflows do PWM (2^16 - 1)

/*******************************************************************************
 * Variáveis Globais
 *******************************************************************************/
uint16_t PWM_DUTY_CYCLE = 0;             // Ciclo de trabalho do PWM (0 a 65535)
uint16_t resultA = 0;                    // Resultado do ADC após processamento
uint32_t numbers[NUM_CANAIS][N_1] = {0}; // Buffers para média móvel por canal
uint64_t sums[NUM_CANAIS] = {0};         // Somatórios acumulados para cada canal
uint16_t indices[NUM_CANAIS] = {0};      // Índices circulares para cada canal

/*******************************************************************************
 * Funções
 *******************************************************************************/

/*============================================================================
Name    :    map
------------------------------------------------------------------------------
Purpose :   Mapeia um valor de uma faixa de entrada para uma faixa de saída.
Input   :   x       - Valor a ser mapeado
            in_min  - Valor mínimo da faixa de entrada
            in_max  - Valor máximo da faixa de entrada
            out_min - Valor mínimo da faixa de saída
            out_max - Valor máximo da faixa de saída
Output  :   Valor mapeado para a nova faixa (uint16_t)
============================================================================*/
uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
    // Fórmula para mapear o valor x de uma faixa [in_min, in_max] para [out_min, out_max]
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*============================================================================
Name    :    Media_Movel
------------------------------------------------------------------------------
Purpose :   Calcula a média móvel para um valor lido do ADC, suavizando leituras.
Input   :   canal - Índice do canal (0 a NUM_CANAIS-1)
            valor - Valor lido do ADC
Output  :   Média móvel do valor do canal lido (uint16_t)
============================================================================*/
uint16_t Media_Movel(uint8_t canal, uint16_t valor)
{
    uint16_t index = indices[canal];   // Pega o índice atual do canal
    uint64_t *sum = &sums[canal];      // Ponteiro para o somatório do canal
    uint32_t *buffer = numbers[canal]; // Buffer de valores do canal

    // Atualiza o somatório: subtrai o valor mais antigo e adiciona o novo
    *sum = *sum - buffer[index] + valor;

    // Atualiza o buffer com o novo valor
    buffer[index] = valor;

    // Incrementa o índice circular
    indices[canal] = (index + 1) % N_1;

    // Retorna a média móvel como valor inteiro
    return (uint16_t)(*sum / N_1);
}

/*******************************************************************************
 * Função Principal
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Inicializa o hardware do dispositivo e os periféricos da placa */
    result = cybsp_init();
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Inicializa a UART para depuração */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Inicializa o módulo SAR2 para leitura do ADC */
    Cy_SAR2_Init(ADC0_HW, &ADC0_config);
    Cy_SAR2_SetReferenceBufferMode(PASS0_EPASS_MMIO, CY_SAR2_REF_BUF_MODE_OFF);
    Cy_SAR2_Channel_SoftwareTrigger(ADC0_HW, 0); // Gera um trigger para iniciar o ADC

    /* Habilita interrupções globais */
    __enable_irq();

    /* Inicializa o PWM configurado no Device Configurator */
    printf("Inicializando o PWM...\n");
    result = Cy_TCPWM_PWM_Init(TCPWM0_PWM0_HW, TCPWM0_PWM0_NUM, &TCPWM0_PWM0_config);
    if (result != CY_TCPWM_SUCCESS)
    {
        printf("Falha ao inicializar o PWM.\n");
        CY_ASSERT(0);
    }

    /* Inicia o PWM */
    Cy_TCPWM_PWM_Enable(TCPWM0_PWM0_HW, TCPWM0_PWM0_NUM);
    Cy_TCPWM_TriggerStart_Single(TCPWM0_PWM0_HW, TCPWM0_PWM0_NUM);
    printf("PWM iniciado com %u Hz.\n", PWM_FREQUENCY_HZ);

    /* Loop Principal */
    for (;;)
    {
        /* Lê o valor do ADC (canal 0) */
        resultA = Cy_SAR2_Channel_GetResult(ADC0_HW, 0, NULL);

        /* Calcula a média móvel para o valor lido */
        resultA = Media_Movel(0, resultA);

        /* Mapeia o valor do ADC (0-4095) para o ciclo de trabalho do PWM (0-65535) */
        PWM_DUTY_CYCLE = map(resultA, 0, (ADC_12_BIT), 0, OVERPLOW_PWM);

        /* Atualiza o ciclo de trabalho do PWM */
        Cy_TCPWM_PWM_SetCompare0(TCPWM0_PWM0_HW, TCPWM0_PWM0_NUM, (PWM_DUTY_CYCLE+1));

        /* Imprime os valores para depuração */
        printf("ADC0: %u, PWM: %u\n", resultA, PWM_DUTY_CYCLE);
    }
}
#include <stm32f411xe.h>
#include <stm32_def.h>
#include "stm32f4xx_hal.h"
#include "main.h"

ADC_HandleTypeDef AdcHandle = {};
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

/**
  * Setup ADC1 for 4 channel scanned mode
  * Output via DMA
  * Triggered from TIM1 CC1
  */
static void MX_ADC1_Init(void)
{
    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
    AdcHandle.Instance = ADC1;
    AdcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
    AdcHandle.Init.ContinuousConvMode = DISABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
    AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    AdcHandle.Init.DMAContinuousRequests = ENABLE;
    AdcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    AdcHandle.Init.ScanConvMode = ENABLE;
    AdcHandle.Init.NbrOfConversion = NCHANNELS;       // number of channels configured

    AdcHandle.State = HAL_ADC_STATE_RESET;
    AdcHandle.DMA_Handle = NULL;    // gets set later
    AdcHandle.Lock = HAL_UNLOCKED;

    if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
    {
        Error_Handler();
    }

    // settings for each channel
    uint channels[NCHANNELS] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_4, ADC_CHANNEL_6};
    ADC_ChannelConfTypeDef AdcChannelConf = {};
    AdcChannelConf.SamplingTime = ADC_SAMPLETIME_56CYCLES;    // XXX investigate different sample times

    for(int i=0; i<NCHANNELS; i++) {
        AdcChannelConf.Channel = channels[i];
        AdcChannelConf.Rank = i+1;  // ranks start at 1
        if (HAL_ADC_ConfigChannel(&AdcHandle, &AdcChannelConf) != HAL_OK)
        {
            Error_Handler();
        }
    }

    // no calibration for F411
}


/**
  * Slave SPI to read out filtered adc values
  */
static void MX_SPI2_Init(void)
{
    /* SPI2 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_SLAVE;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }

    /* SPI2 DMA Init */
    hdma_spi2_tx.Instance = DMA1_Stream4;
    hdma_spi2_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi2_tx.Init.Mode = DMA_CIRCULAR;
    hdma_spi2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&hspi2,hdmatx,hdma_spi2_tx);
}

/**
  * TIM1 is the trigger for the ADC
  */
static void MX_TIM1_Init(void)
{
    // Frequency calcs
    //
    // rate = clock/ ((psc+1) * (period+1))
    // (psc+1) * (period+1) = clock/rate
    //
    // psc and period must both be in the range 0 - 65535
    //
    // with psc 0, min freq = 1525.9Hz, max freq = 100MHz
    //   period = (clk / rate) - 1
    //
    //   period   rate
    //   12499    8kHz

    #define PRESCALE 0
    #define PERIOD ((100000000 / PER_CHANNEL_SAMPLE_RATE) - 1)

    #if (PERIOD > 65535)
    #error "period out of range"
    #endif

    Serial.printf("period %x\n", PERIOD);

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    // TIM_ClockConfigTypeDef clkConfig = {};               // is something like this needed?
    // clkConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    // HAL_TIM_ConfigClockSource(&htim1, &clkConfig);

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = PRESCALE;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = PERIOD;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = PERIOD/2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }


    // HAL_TIM_MspPostInit(&htim1);
}

static void initDMA(void)
{
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&AdcHandle, DMA_Handle, hdma_adc1);
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt for ADC conversions */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/** Plumbing to connect the dma irqs to their callbacks
 */
extern "C" {

// Interrupts for ADC
void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

// DMA for SPI slave
// void DMA1_Channel3_IRQHandler(void)
// {
//   HAL_DMA_IRQHandler(&hdma_spi1_tx);
// }

} // extern "C"



static void adcInit(void)
{
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_SPI2_Init();
    MX_TIM1_Init();
    initDMA();

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /**SPI2 GPIO Configuration
    PB12     ------> SPI2_NSS
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    // kick off the conversions
    HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)adcBuffer, ADC_BUFFER_SIZE);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    // setup slave SPI to read out the filtered adc data
    HAL_SPI_Transmit_DMA(&hspi2, (uint8_t *)filteredValues, NCHANNELS*2);

}
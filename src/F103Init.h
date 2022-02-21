#include <stm32f103xb.h>

TIM_HandleTypeDef htim2;
ADC_HandleTypeDef AdcHandle = {};
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;



static void MX_ADC1_init(void)
{
    // // Settings for the ADC
    // AdcHandle.Instance                 = ADC1;
    // AdcHandle.Init.DataAlign           = ADC_DATAALIGN_RIGHT;
    // AdcHandle.Init.ScanConvMode        = ADC_SCAN_ENABLE;                       //ADC_SCAN_ENABLE;
    // AdcHandle.Init.ContinuousConvMode  = DISABLE;
    // AdcHandle.Init.NbrOfConversion     = NCHANNELS;       // number of channels configured
    // AdcHandle.Init.NbrOfDiscConversion = 0;
    // AdcHandle.Init.ExternalTrigConv    = ADC_EXTERNALTRIGCONV_T2_CC2;

    // AdcHandle.State = HAL_ADC_STATE_RESET;
    // AdcHandle.DMA_Handle = NULL;    // gets set later
    // AdcHandle.Lock = HAL_UNLOCKED;

    // HAL_StatusTypeDef result;

    // result = HAL_ADC_Init(&AdcHandle);
    // Serial.println(result);

    // // initial calibration
    // result = HAL_ADCEx_Calibration_Start(&AdcHandle);
    // Serial.println(result);

    // // settings for each channel
    // uint channels[NCHANNELS] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_4, ADC_CHANNEL_6};
    // // uint channels[NCHANNELS] = {ADC_CHANNEL_0, ADC_CHANNEL_1};
    // ADC_ChannelConfTypeDef AdcChannelConf = {};
    // // AdcChannelConf.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;    // XXX investigate different sample times
    // // AdcChannelConf.SamplingTime = ADC_SAMPLETIME_28CYCLES_5; // works ok at 10kHz * 4
    // AdcChannelConf.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;    // XXX investigate different sample times

    // for(int i=0; i<NCHANNELS; i++) {
    //     AdcChannelConf.Channel = channels[i];
    //     AdcChannelConf.Rank = i+1;  // ranks start at 1
    //     if (HAL_ADC_ConfigChannel(&AdcHandle, &AdcChannelConf) != HAL_OK)
    //     {
    //         Error_Handler();
    //     }
    // }

    // // The adc clock is setup in SystemClock_Config() to cpu clock/6 (12MHz at standard clock speed)


}

static void setupDMA(void)
{
    // // Setup the DMA controller
    // // enable clock
    // __HAL_RCC_DMA1_CLK_ENABLE();

    // hdma_adc1.Instance = DMA1_Channel1;
    // hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    // hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    // hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    // hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    // hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    // hdma_adc1.Init.Mode = DMA_CIRCULAR;
    // hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    // if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    // {
    //   Error_Handler();
    // }

    // __HAL_LINKDMA(&AdcHandle, DMA_Handle, hdma_adc1);

    // // DMA interrupt init
    // HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    // /* DMA1_Channel3_IRQn interrupt configuration */
    // HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}


static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA15     ------> SPI1_NSS
    PB3     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // __HAL_AFIO_REMAP_SPI1_ENABLE(); XXX

    /* SPI1 DMA Init */
    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = DMA1_Channel3;  // todo double check this is right
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_CIRCULAR;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmatx,hdma_spi1_tx);
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM2)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PB3     ------> TIM2_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // __HAL_AFIO_REMAP_TIM2_PARTIAL_1(); XXX

  }
}


static void MX_TIM2_Init(void)
{
    // Frequency calcs
    //
    // rate = clock/ ((psc+1) * (period+1))
    // (psc+1) * (period+1) = clock/rate
    //
    // psc and period must both be in the range 0 - 65535
    //
    // with psc 0, min freq = 1098.6Hz
    //   period = (clk / rate) - 1
    //
    //   period   rate
    //   8999     8kHz

    #define PRESCALE 0
    #define PERIOD ((72000000 / PER_CHANNEL_SAMPLE_RATE) - 1)

    #if (PERIOD > 65535)
    #error "period out of range"
    #endif

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    
    htim2.Init.Prescaler = PRESCALE;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = PERIOD;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = PERIOD/2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    // HAL_TIM_MspPostInit(&htim2);

}

/** Plumbing to connect the dma irqs to their callbacks
 */
extern "C" {

void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc1);
}

void DMA1_Channel3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

} // extern "C"


static void adcInit(void)
{
    MX_TIM2_Init();
    __HAL_RCC_TIM2_CLK_ENABLE();

    // // kick off the conversions
    // HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)adcBuffer, ADC_BUFFER_SIZE);
    // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    // // setup slave SPI to read out the filtered adc data
    // MX_SPI1_Init();
    // HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)filteredValues, 4*2);

}
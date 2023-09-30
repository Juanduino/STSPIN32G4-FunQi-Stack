
#include <Arduino.h>
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"

//#pragma once


#define PHASE_AH PE9
#define PHASE_AL PE8
#define PHASE_BH PE11
#define PHASE_BL PE10
#define PHASE_CH PE13
#define PHASE_CL PE12


#define INTERNAL_I2C3_SDA PC9
#define INTERNAL_I2C3_SCL PC8

#define INTERNAL_WAKE   PE7
#define INTERNAL_READY  PE14
#define INTERNAL_nFAULT PE15

// CORDIC SETUP 
static CORDIC_HandleTypeDef thisCordic;
static void CORDIC_Config(void)
{

LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);

CORDIC_ConfigTypeDef sConfig;
thisCordic.Instance = CORDIC;

 if (HAL_CORDIC_Init(&thisCordic) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
  }


sConfig.Function = CORDIC_FUNCTION_SINE;  
sConfig.Precision = CORDIC_PRECISION_3CYCLES;
sConfig.Scale = CORDIC_SCALE_0;
sConfig.NbWrite = CORDIC_NBWRITE_1;
sConfig.NbRead = CORDIC_NBREAD_2;
sConfig.InSize = CORDIC_INSIZE_32BITS;
sConfig.OutSize = CORDIC_OUTSIZE_32BITS;

  //HAL_CORDIC_Configure(&thisCordic, &sConfig);

if (HAL_CORDIC_Configure(&thisCordic, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }

}




static void CORDIC_Config_LL(void){

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);

  /* Configure CORDIC peripheral */
  LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_COSINE, /* cosine function */
                   LL_CORDIC_PRECISION_6CYCLES,       /* max precision for q1.31 cosine */
                   LL_CORDIC_SCALE_0,                 /* no scale */
                   LL_CORDIC_NBWRITE_1,               /* One input data: angle. Second input data (modulus) is 1 after cordic reset */
                   LL_CORDIC_NBREAD_2,                /* Two output data: cosine, then sine */
                   LL_CORDIC_INSIZE_32BITS,           /* q1.31 format for input data */
                   LL_CORDIC_OUTSIZE_32BITS);         /* q1.31 format for output data */
}


//**************************************************************************//







/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;





uint16_t adc1_buffer[4];

static void set_ADC(void){

  __HAL_RCC_ADC12_CLK_ENABLE();

   ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  GPIO_InitTypeDef GPIO_InitStruct;

 
   /* DMA controller clock enable */
  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  //__HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

  //SerialUSB.println("DMA Set priority!");


  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
      SerialUSB.println("ADC1 Init failed ");
  } else {

     SerialUSB.println("ADC1 Init OK");
  }


  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

   
   

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN1
    PA2     ------> ADC1_IN3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*

    hdma_adc1.Instance = DMA1_Channel2;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      SerialUSB.println("Error in HAL_DMA_Init");
    } else {

      SerialUSB.println("HAL_DMA_Init OK");}

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    */

   /* ADC1 interrupt Init */
    //HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

 /** Configure the ADC multi-mode
  * 
  * 
  
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_2CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    SerialUSB.println("Error in HAL_ADCEx_MultiModeConfigChannel");
  } else {
      
      SerialUSB.println("HAL_ADCEx_MultiModeConfigChannel OK");
  }

*/
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    SerialUSB.println("Error in HAL_ADC_ConfigChannel");
  } else {

    SerialUSB.println("HAL_ADC_ConfigChannel OK");
  }


  //********************************************************************//

     /**Common config */

     

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    SerialUSB.println("Error in HAL_ADC_Init");
  } else {

    SerialUSB.println("HAL_ADC_Init OK");}


  


  __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

     GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  

   /** Configure Regular Channel*/
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    SerialUSB.println("Error in HAL_ADC_ConfigChannel");
  }else {
      
      SerialUSB.println("HAL_ADC_ConfigChannel OK");
  }

      



     //ADC DMA
     if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    SerialUSB.println("Error in HAL_ADCEx_Calibration_Start");
  }else {
        
        SerialUSB.println("HAL_ADCEx_Calibration_Start OK");
  }


    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    SerialUSB.println("Error in HAL_ADCEx_Calibration_Start");
  }else {
        
        SerialUSB.println("HAL_ADCEx_Calibration_Start OK");
  }

  
  if (HAL_ADC_Start(&hadc1)!= HAL_OK){

      SerialUSB.println("ADC1 Start failed!");
  }else {
        
        SerialUSB.println("ADC1 Start OK");
  }

   if (HAL_ADC_Start(&hadc2)!= HAL_OK){

      SerialUSB.println("ADC2 Start failed!");
  }else {
        
        SerialUSB.println("ADC2 Start OK");
  }

    //SET_BIT(hadc1.Instance->CFGR, ADC_CFGR_DMAEN); //Enable DMA transfer for ADC master (ADC12_CCR.MDMA = 0b00 -> MDMA mode disabled)

    //HAL_ADC_Start_DMA(&hadc1, (uint16_t*)adc_array, 100);

    /*
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adc1_buffer, sizeof(adc1_buffer)) != HAL_OK){
        SerialUSB.println("Error in HAL_ADCEx_MultiModeStart_DMA");
    } else {
        
        SerialUSB.println("HAL_ADCEx_MultiModeStart_DMA OK");
    }
 */

   //Note: oversample eatch ADC and use injected channels.

  /* Wait for the first ADC conversion to be completed (timeout unit: ms) */
  if (HAL_ADC_PollForConversion(&hadc1, 2) != HAL_OK){
    SerialUSB.println("ADC1 Poll Failed");
  }else {  
        SerialUSB.println("ADC1 POLL OK");
  }
  
    if (HAL_ADC_PollForConversion(&hadc2, 2) != HAL_OK){
    SerialUSB.println("ADC2 Poll Failed");
  }else {  
        SerialUSB.println("ADC2 POLL OK");
  }

 
 return;

}



uint16_t return_adc (){

return (uint16_t)(READ_BIT(ADC1->DR, ADC_DR_RDATA));

}

uint16_t return_adc1 (){

return (uint16_t)(READ_BIT(ADC2->DR, ADC_DR_RDATA));

}


/*
extern "C" void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 *

  /* USER CODE END ADC1_2_IRQn 0 *
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 *

  /* USER CODE END ADC1_2_IRQn 1 *
//}
*/

TIM_HandleTypeDef htim4;


/* TIM2 init function */
static void TIM15_Init(void)
{


 
SerialUSB.println("TIM2 Init");


  __HAL_RCC_TIM15_CLK_ENABLE();

   /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
   TIM_OC_InitTypeDef sConfigOC = {0};
  
  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim4.Instance = TIM15;
  htim4.Init.Prescaler = 10;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 15000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    SerialUSB.println("TIM2 Base Init Error");
  } else {
    SerialUSB.println("TIM2 Base Init OK");
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
     SerialUSB.println("TIM2 Clock Config Error");
  } else {
    SerialUSB.println("TIM2 Clock Config OK");
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    SerialUSB.println("TIM2 Master Config Error");
  } else {
    SerialUSB.println("TIM2 Master Config OK");
  }
  
 
    if (HAL_TIM_Base_Start(&htim4)!= HAL_OK){
    SerialUSB.println("TIM BASE START ERROR");
    } else {
      SerialUSB.println("TIM BASE START OK");
    }

   
  
  
}

//***********************************************************************//

uint16_t read_TIM3() {
  return TIM3->CNT;
}



static void set_QuadEncoderTimer(void){

   __HAL_RCC_TIM3_CLK_ENABLE();
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOD_CLK_ENABLE();

   // use custom function to switch to AlternateFuction.

   GPIO_InitTypeDef GPIO_InitStruct = {0};
  
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


   TIM_HandleTypeDef       Encoder_Handle;
   TIM_Encoder_InitTypeDef sEncoderConfig;
   TIM_MasterConfigTypeDef sMasterConfig;
   //TIM_ClockConfigTypeDef sClockSourceConfig;

   Encoder_Handle.Instance = TIM3;
   Encoder_Handle.Init.Period             = 65535;
   Encoder_Handle.Init.Prescaler          = 0;
   Encoder_Handle.Init.ClockDivision      = 0;
   Encoder_Handle.Init.CounterMode        = TIM_COUNTERMODE_UP;
   Encoder_Handle.Init.RepetitionCounter  = 0;
   Encoder_Handle.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_DISABLE;

   sEncoderConfig.EncoderMode             = TIM_ENCODERMODE_TI12;

   sEncoderConfig.IC1Polarity             = TIM_ICPOLARITY_RISING;
   sEncoderConfig.IC1Selection            = TIM_ICSELECTION_DIRECTTI;
   sEncoderConfig.IC1Prescaler            = TIM_ICPSC_DIV1;
   sEncoderConfig.IC1Filter               = 0;

   sEncoderConfig.IC2Polarity             = TIM_ICPOLARITY_RISING;
   sEncoderConfig.IC2Selection            = TIM_ICSELECTION_DIRECTTI;
   sEncoderConfig.IC2Prescaler            = TIM_ICPSC_DIV1;
   sEncoderConfig.IC2Filter               = 0;

  /*
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE1;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;

  */
  
   enableTimerClock(&Encoder_Handle);
   if(HAL_TIM_Encoder_Init(&Encoder_Handle, &sEncoderConfig) != HAL_OK){

    Serial.println("Encoder init error");
   }else {
    Serial.println("Encoder init OK");
   }

     sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
    //sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENCODER_CLK;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&Encoder_Handle, &sMasterConfig) != HAL_OK)
    {
        Serial.println("TIM3 Master config error");
    } else {
        Serial.println("TIM3 Master config OK");}

  if (HAL_TIM_Encoder_Start(&Encoder_Handle, TIM_CHANNEL_ALL)!= HAL_OK) Serial.println("Init error of TIM3");
   




}


static void TIM2_init(void){

TIM_HandleTypeDef       htim2;
TIM_SlaveConfigTypeDef sSlaveConfig;
TIM_MasterConfigTypeDef sMasterConfig;
TIM_IC_InitTypeDef sConfigIC;

__HAL_RCC_TIM2_CLK_ENABLE();


    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF-1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.RepetitionCounter = 0;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Serial.println("TIM2 Base init error");
    } else {
        Serial.println("TIM2 Base init OK");
    }
 
    if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
    {
        Serial.println("TIM2 IC init error");
    }
    else
    {
        Serial.println("TIM2 IC init OK");
    }

    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    sSlaveConfig.InputTrigger = TIM_TS_ITR2;
    sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
    sSlaveConfig.TriggerFilter = 0;
    if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
    {
        Serial.println("TIM2 Slave config error");
    }
    else
    {
        Serial.println("TIM2 Slave config OK");
    }

      sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
      sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {Serial.println("TIM2 Master config error");
    } else {Serial.println("TIM2 Master config OK");}



     sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    //sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_TRC;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    {
        Serial.println("TIM2 IC config error");
    }
    else
    {
        Serial.println("TIM2 IC config OK");
    }

    if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1)  != HAL_OK)
    {
        Serial.println("TIM2 IC start error");
    }
    else
    {
        Serial.println("TIM2 IC start OK");
    }


}

//**********************************************************************************************************//




// Rodebunke

/*

long start_ticks = 0;
long stop_ticks = 0;
float elapsed_ticks = 0.0f;

uint32_t current_angle = 0;

int count = 0;

  start_ticks = micros(); 
 for (int i=0;i<700;i++)
{

  sensor.update();
  //buff_BIN_angles[i] = sensor.getRaw21bitMT6835();
  float angle = motor.electricalAngle();
  Serial.print("Angle (RAD):   ");
  Serial.println(angle);
  count++;
  delay(200);

}

 
stop_ticks = micros(); 
elapsed_ticks = stop_ticks-start_ticks;
Serial.print("Iterations:   ");
Serial.println(count);
Serial.print("elapsed_ticks:   ");
Serial.println(elapsed_ticks, 4);
Serial.print("time per iterasion:   ");
Serial.println(elapsed_ticks / count, 4);


 
  for (int i=0;i<700;i++)
{

  Serial.print(i);
  Serial.print(",");
  Serial.print(buff_BIN_angles[i], BIN);
  Serial.print(",");
  Serial.println(buff_BIN_angles[i] / (float)2097152 * _2PI, 7);

  delay(10);

}

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"
#include "stm32g4xx_ll_tim.h"


long start_ticks = 0;
long stop_ticks = 0;
float elapsed_ticks = 0.0f;

angle2 = angle;
angle2 = _normalizeAngle(angle2);
float _ca = _cos(angle2);
float _sa = _sin(angle2);


q31_value = float_to_q31(angle / (2.0f * pi));

CORDIC->WDATA = q31_value;
cordic_sine = CORDIC->RDATA;
cordic_cosine = CORDIC->RDATA;

value_f32_sine = (float)cordic_sine/(float)0x80000000;
value_f32_cosine = (float)cordic_cosine/(float)0x80000000;


if (angle < 0){
value_f32_sine = wrap_to_1(value_f32_sine);
value_f32_sine = value_f32_sine * -1;
}

if (angle > 0){
value_f32_sine = wrap_to_1(value_f32_sine);
}

value_f32_cosine = wrap_to_1(value_f32_cosine);


  Serial.print("sine:");
  Serial.print(value_f32_sine, 8);
  Serial.print(",");
  Serial.print("cosine:");
  Serial.print(value_f32_cosine, 8);  
  Serial.println();

 delay(50);


}


stop_ticks = micros(); 


*/

/*



elapsed_ticks = stop_ticks-start_ticks;
Serial.print("float angle:   ");
Serial.println(angle);
Serial.print("Iterations:   ");
Serial.println(count);


//Serial.print("input_q31_sin2:   ");
//Serial.println(input_q31_sin2);
Serial.print("elapsed_ticks:   ");
Serial.println(elapsed_ticks, 4);
Serial.print("time per iterasion:   ");
Serial.println(elapsed_ticks / count, 4);
//Serial.print("Normalized angle:   ");
//Serial.println(angle_el);
//Serial.print("Cordic_sine:   ");
//Serial.println(cordic_sine);
//Serial.print("Cordic_cosine:   ");
//Serial.println(cordic_cosine);

//Serial.print("converted_Cordic_sine:   ");
//Serial.println(value_f32_sine, 8);
//Serial.print("converted_Cordic_cosine:   ");
//Serial.println(value_f32_cosine, 8);



// sensor

  uint16_t ABZres = sensor.getZeroPosition();
  Serial.print("ABZ RES =  ");
  Serial.println(ABZres);

  delay(2000);  

     if (FD_CAN_RX_State = HIGH){
    Serial.println("FD_CAN_RX_State = HIGH");
  } else {
    Serial.println("FD_CAN_RX_State = LOW");
  }
   if (nFAULT = HIGH){
    Serial.println("nFAULT = HIGH");
  } else {
    Serial.println("nFAULT = LOW");
  }
   if (READY = HIGH){
    Serial.println("READY = HIGH");
  } else {
    Serial.println("READY = LOW");
  }

*/
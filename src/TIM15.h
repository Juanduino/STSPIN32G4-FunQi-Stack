
#include <stm32g4xx_ll_tim.h>
#include <stm32g4xx_ll_rcc.h>


void Configure_TIM_TimeBase_ADC_trigger(void)
{
  uint32_t timer_clock_frequency = 0;             /* Timer clock frequency */
  uint32_t timer_prescaler = 0;                   /* Time base prescaler to have timebase aligned on minimum frequency possible */
  uint32_t timer_reload = 0;                      /* Timer reload value in function of timer prescaler to achieve time base period */
  
  /*## Configuration of NVIC #################################################*/
  /* Note: In this example, timer interrupts are not activated.               */
  /*       If needed, timer interruption at each time base period is          */
  /*       possible.                                                          */
  /*       Refer to timer examples.                                           */
  
  /* Note: In this example, timer interrupts are not activated */
  /*       If needed, timer interruption at each time base     */
  /*       period is possible.                                 */
  /*       Refer to timer examples.                            */
  
  /* Configuration of timer as time base:                                     */ 
  /* Caution: Computation of frequency is done for a timer instance on APB1   */
  /*          (clocked by PCLK1)                                              */
  /* Timer frequency is configured from the following constants:              */
  /* - TIMER_FREQUENCY: timer frequency (unit: Hz).                           */
  /* - TIMER_FREQUENCY_RANGE_MIN: timer minimum frequency possible            */
  /*   (unit: Hz).                                                            */
  /* Note: Refer to comments at these literals definition for more details.   */
  
  /* Retrieve timer clock source frequency */
  /* If APB1 prescaler is different of 1, timers have a factor x2 on their    */
  /* clock source.                                                            */
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  
  /* Enable the timer peripheral clock */
   __HAL_RCC_TIM15_CLK_ENABLE();
  
 
  
  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 8;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 15000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM15, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM15);
  //LL_TIM_SetTriggerInput(TIM15, LL_TIM_TS_ITR0);
  LL_TIM_SetSlaveMode(TIM15, LL_TIM_SLAVEMODE_DISABLED);
  LL_TIM_DisableIT_TRIG(TIM15);
  LL_TIM_DisableDMAReq_TRIG(TIM15);
  LL_TIM_SetTriggerOutput(TIM15, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM15);
  /* USER CODE BEGIN TIM2_Init 2 */
  /* Note: In this example, timer interrupts are not activated.               */
  /*       If needed, timer interruption at each time base period is          */
  /*       possible.                                                          */
  /*       Refer to timer examples.                                           */
  
  /* Set timer the trigger output (TRGO) */
  LL_TIM_SetTriggerOutput(TIM15, LL_TIM_TRGO_UPDATE);
  
  /* Enable counter */
  LL_TIM_EnableCounter(TIM15);
  /* USER CODE END TIM2_Init 2 */

}
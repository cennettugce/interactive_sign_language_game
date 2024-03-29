/**
  ******************************************************************************
  * File Name          : FSMC.c
  * Description        : This file provides code for the configuration
  *                      of the FSMC peripheral.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "fsmc.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

//SRAM_HandleTypeDef hsram1;

/* FSMC initialization function */


//static uint32_t FSMC_Initialized = 0;

//static void HAL_FSMC_MspInit(void){
//  /* USER CODE BEGIN FSMC_MspInit 0 */
//
//  /* USER CODE END FSMC_MspInit 0 */
//  GPIO_InitTypeDef GPIO_InitStruct;
//  if (FSMC_Initialized) {
//    return;
//  }
//  FSMC_Initialized = 1;
//  /* Peripheral clock enable */
//  __HAL_RCC_FSMC_CLK_ENABLE();
//
//  /** FSMC GPIO Configuration
//  PE7   ------> FSMC_D4
//  PE8   ------> FSMC_D5
//  PE9   ------> FSMC_D6
//  PE10   ------> FSMC_D7
//  PE11   ------> FSMC_D8
//  PE12   ------> FSMC_D9
//  PE13   ------> FSMC_D10
//  PE14   ------> FSMC_D11
//  PE15   ------> FSMC_D12
//  PD8   ------> FSMC_D13
//  PD9   ------> FSMC_D14
//  PD10   ------> FSMC_D15
//  PD11   ------> FSMC_A16
//  PD14   ------> FSMC_D0
//  PD15   ------> FSMC_D1
//  PD0   ------> FSMC_D2
//  PD1   ------> FSMC_D3
//  PD4   ------> FSMC_NOE
//  PD5   ------> FSMC_NWE
//  PD7   ------> FSMC_NE1
//  */
//  /* GPIO_InitStruct */
//  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
//                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
//                          |GPIO_PIN_15;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
//
//  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//
//  /* GPIO_InitStruct */
//  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
//                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1
//                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
//
//  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN FSMC_MspInit 1 */
//
//  /* USER CODE END FSMC_MspInit 1 */
//}



//static uint32_t FSMC_DeInitialized = 0;

//static void HAL_FSMC_MspDeInit(void){
//  /* USER CODE BEGIN FSMC_MspDeInit 0 */
//
//  /* USER CODE END FSMC_MspDeInit 0 */
//  if (FSMC_DeInitialized) {
//    return;
//  }
//  FSMC_DeInitialized = 1;
//  /* Peripheral clock enable */
//  __HAL_RCC_FSMC_CLK_DISABLE();
//
//  /** FSMC GPIO Configuration
//  PE7   ------> FSMC_D4
//  PE8   ------> FSMC_D5
//  PE9   ------> FSMC_D6
//  PE10   ------> FSMC_D7
//  PE11   ------> FSMC_D8
//  PE12   ------> FSMC_D9
//  PE13   ------> FSMC_D10
//  PE14   ------> FSMC_D11
//  PE15   ------> FSMC_D12
//  PD8   ------> FSMC_D13
//  PD9   ------> FSMC_D14
//  PD10   ------> FSMC_D15
//  PD11   ------> FSMC_A16
//  PD14   ------> FSMC_D0
//  PD15   ------> FSMC_D1
//  PD0   ------> FSMC_D2
//  PD1   ------> FSMC_D3
//  PD4   ------> FSMC_NOE
//  PD5   ------> FSMC_NWE
//  PD7   ------> FSMC_NE1
//  */
//
//  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
//                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
//                          |GPIO_PIN_15);
//
//  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
//                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1
//                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7);
//
//  /* USER CODE BEGIN FSMC_MspDeInit 1 */
//
//  /* USER CODE END FSMC_MspDeInit 1 */
//}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

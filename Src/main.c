/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 30/12/2019 15:16:18
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
static CanTxMsgTypeDef        can1TxMessage;
static CanRxMsgTypeDef        can1RxMessage;

uint8_t interface_state = 0 ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void SystemClock_Config(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN2_Init(void);

/* USER CODE BEGIN PFP */
extern uint8_t CDC_periodic_callback(void);
extern uint8_t CDC_add_buf_to_transmit(uint8_t* Buf, uint16_t Len);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t halfbyte_to_hexascii(uint8_t _halfbyte)
{
 _halfbyte &= 0x0F ;
 if(_halfbyte >= 10) return('A' + _halfbyte - 10) ; 
	else               return('0' + _halfbyte) ; 
}

uint8_t hexascii_to_halfbyte(uint8_t _ascii)
{
 if((_ascii >= '0') && (_ascii <= '9')) return(_ascii - '0') ; 
 if((_ascii >= 'a') && (_ascii <= 'f')) return(_ascii - 'a') ; 
 if((_ascii >= 'A') && (_ascii <= 'F')) return(_ascii - 'A') ; 
 return(0xFF)	;
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle)
{
	uint32_t num_bytes ;
	uint8_t buf[200] ;
	static uint32_t time ;
	
	time = __HAL_TIM_GetCounter(&htim1) ; 
/*	
  num_bytes = sprintf((char*)buf,"t%3.3X%1.1X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%4.4X\r", \
		                              CanHandle->pRxMsg->StdId,\
																	CanHandle->pRxMsg->DLC,\
																	CanHandle->pRxMsg->Data[0],\
																	CanHandle->pRxMsg->Data[1],\
																	CanHandle->pRxMsg->Data[2],\
																	CanHandle->pRxMsg->Data[3],\
																	CanHandle->pRxMsg->Data[4],\
																	CanHandle->pRxMsg->Data[5],\
																	CanHandle->pRxMsg->Data[6],\
																	CanHandle->pRxMsg->Data[7],\
																	time
																	);
*/
  num_bytes = 0 ;
	buf[num_bytes++] = 't' ;
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->StdId)>>8);
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->StdId)>>4);
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->StdId));
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->DLC));
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[0])>>4);
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[0]));
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[1])>>4);
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[1]));
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[2])>>4);
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[2]));
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[3])>>4);
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[3]));
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[4])>>4);
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[4]));
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[5])>>4);
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[5]));
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[6])>>4);
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[6]));
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[7])>>4);
	buf[num_bytes++] = halfbyte_to_hexascii((CanHandle->pRxMsg->Data[7]));
	buf[num_bytes++] = halfbyte_to_hexascii((time)>>12);
	buf[num_bytes++] = halfbyte_to_hexascii((time)>>8);
	buf[num_bytes++] = halfbyte_to_hexascii((time)>>4);
	buf[num_bytes++] = halfbyte_to_hexascii((time)>>0);
	buf[num_bytes++] = '\r';
	
	if(interface_state == 1) CDC_add_buf_to_transmit(buf,num_bytes);
	
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  SystemClock_Config();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  MX_CAN2_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	
  hcan2.pTxMsg = &can1TxMessage;
  hcan2.pRxMsg = &can1RxMessage;
	
  CAN_FilterConfTypeDef canFilterConfig;
  canFilterConfig.FilterNumber = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000 << 5;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = 0;
  canFilterConfig.FilterActivation = ENABLE;
  canFilterConfig.BankNumber = 1;
	HAL_CAN_ConfigFilter(&hcan2, &canFilterConfig);
	
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) ;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
   CDC_periodic_callback();
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBPLLCLK_DIV3;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __HAL_RCC_AFIO_CLK_ENABLE();

}

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_6TQ;
  hcan1.Init.BS2 = CAN_BS2_5TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan1);

}

/* CAN2 init function */
void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_1TQ;
  hcan2.Init.BS1 = CAN_BS1_6TQ;
  hcan2.Init.BS2 = CAN_BS2_5TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = DISABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan2);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 18000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 60000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

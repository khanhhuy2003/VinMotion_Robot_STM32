/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "i2c.h"
#include "lwip.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "udpSever.h"
#include "motor_control.h"
#include "math.h"
extern int udp_count;

volatile uint8_t uart_flag = 0;
 uint8_t TxData[8];
 uint8_t RxData[8];
 FDCAN_TxHeaderTypeDef TxHeader;
 FDCAN_RxHeaderTypeDef RxHeader;
#define POLYNOMIAL 0x04C11DB7
#define INITIAL_CRC 0xFFFFFFFF
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
FDCAN_TxHeaderTypeDef TxHeader;

FDCAN_RxHeaderTypeDef RxHeader;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern struct netif gnetif;

HAL_StatusTypeDef LAN8720_ReadPHY(uint8_t phy_addr, uint8_t reg_addr,
		uint16_t *data) {
	uint32_t reg_value;
	HAL_StatusTypeDef status = HAL_ETH_ReadPHYRegister(&heth, phy_addr,
			reg_addr, &reg_value);
	*data = (uint16_t) reg_value;
	return status;
}
HAL_StatusTypeDef LAN8720_WritePHY(uint8_t phy_addr, uint8_t reg_addr,
		uint16_t data) {
	return HAL_ETH_WritePHYRegister(&heth, phy_addr, reg_addr, data);
}
// H m ki?m tra tr?ng th i k?t n?i PHY
void PHY_CheckStatus(void) {
	uint16_t status = 0;
	uint16_t control = 0;
	uint16_t phy_spec_status = 0;
	//  ?c thanh ghi Basic Control (0x00)
	LAN8720_ReadPHY(LAN8720_PHY_ADDRESS, 0x00, &control);
	printf("PHY Basic Control Register: 0x%04X\n", control);

	//  ?c thanh ghi Basic Status (0x01)
	LAN8720_ReadPHY(LAN8720_PHY_ADDRESS, 0x01, &status);
	printf("PHY Basic Status Register: 0x%04X\n", status);

	// Ki?m tra tr?ng th i li n k?t
	if (status & 0x0004) { // Bit 2: Link Status

		printf("PHY Link is UP\n");

		// Ki?m tra t?c d? v  ch? d? duplex
		LAN8720_ReadPHY(LAN8720_PHY_ADDRESS, 0x1F, &phy_spec_status);
		printf("PHY Specific Status: 0x%04X\n", phy_spec_status);

		if (phy_spec_status & 0x0400) { // Bit 10: Speed 100Mbps
			printf("Speed: 100Mbps\n");
		} else {
			printf("Speed: 10Mbps\n");
		}

		if (phy_spec_status & 0x0800) { // Bit 11: Full Duplex
			printf("Duplex: Full\n");
		} else {
			printf("Duplex: Half\n");
		}
	} else {
		NVIC_SystemReset();
		printf("PHY Link is DOWN\n");
	}
}
								//Extended ID - 29 bits



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void Left_Arm() {
	int i = 0;

	MotorReceiveCommand(MOTOR_15, 0.0, 0.0, 0.0, 20.0, 3, 1);
	MotorReceiveCommand(MOTOR_16, 0.0, 0.0, 0.0, 20.0, 3, 1);
	MotorReceiveCommand(MOTOR_17, 0.0, 0.0, 0.0, 20.0, 3, 1);
	MotorReceiveCommand(MOTOR_18, 0.0, 0.0, 0.0, 20.0, 3, 1);

	if (i <= 1.15f) {
		MotorReceiveCommand(0x0E, 0.0, i, 0.0, 20.0, 3.5, 2);
		i = i + 0.001f;
		HAL_Delay(1);
	} else {
		MotorReceiveCommand(0x0E, 0.0, i, 0.0, 20.0, 3.5, 2);
	}
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  //SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  //HAL_Delay(5000);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_LWIP_Init();
  MX_FDCAN3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK){
	  return 0;
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK) {
		/* Notification Error */
		Error_Handler();
	}
  if(HAL_FDCAN_Start(&hfdcan2) != HAL_OK){
	  return 0;
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
			0) != HAL_OK) {
		/* Notification Error */
		Error_Handler();
  }
  if(HAL_FDCAN_Start(&hfdcan3) != HAL_OK){
		  return 0;
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
				0) != HAL_OK) {
			/* Notification Error */
			Error_Handler();
  }
  ///TxHeader.Identifier = 0x09;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  PHY_CheckStatus();
  udpServer_init();
   //EnableAllMotor();
   EnableMotorX(MOTOR_14, 2);
   EnableMotorX(MOTOR_15, 1);
   EnableMotorX(MOTOR_16, 1);
   EnableMotorX(MOTOR_17, 1);
   EnableMotorX(MOTOR_18, 1);
   EnableMotorX(MOTOR_20, 3);
   EnableMotorX(MOTOR_21, 3);
   EnableMotorX(MOTOR_22, 3);
   EnableMotorX(MOTOR_23, 3);

//   EnableMotorX(MOTOR_9, 1);
//   EnableMotorX(MOTOR_1, 1);


//  EnableMotorX(0x01, 1);

  //EnableMotorX(0x01, 1);

//  HAL_Delay(2000);

  //MotorReceiveCommand(0x09, 1.0, 0.0, 3.0, 20.0, 3, 1);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float t_ff = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  ethernetif_input(&gnetif);
	  sys_check_timeouts();
	  //Left_Arm();
	   EnableMotorX(MOTOR_14, 2);
	   EnableMotorX(MOTOR_15, 1);
	   EnableMotorX(MOTOR_16, 1);
	   EnableMotorX(MOTOR_17, 1);
	   EnableMotorX(MOTOR_18, 1);
	   EnableMotorX(MOTOR_20, 3);
	   EnableMotorX(MOTOR_21, 3);
	   EnableMotorX(MOTOR_22, 3);
	   EnableMotorX(MOTOR_23, 3);

//	  t_ff = 10 * sin(t_ff);
////	      MotorReceiveCommand(MOTOR_15, t_ff, 0.0, 0.0, 0.0, 0.0, 1);
////          HAL_Delay(50);
//	      MotorReceiveCommand(MOTOR_18, t_ff, 0.0, 0.0, 0.0, 0.0, 1);
//          HAL_Delay(50);
//          t_ff = t_ff + 0.1;
//          if (uart_flag) {
//              uart_flag = 0;
//              char msg[100];
////              int len = sprintf(msg, "A: %.2f\r\n, S: %.2f\r\n, T: %.2f\r\n, Temp: %.2f\r\n",
////                                AngleCurrent[8], SpeedCurrent[8], TorqueCurrent[8], TempCurrent[8]);
//              int len = sprintf(msg, "A: %.2f\r\n",
//                                AngleCurrent[14]);
//              HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
//          }
//          HAL_Delay(100);
//	MotorDummy(0x09, 1, 0.0, -35, 0.0, 0.0);
	  //HAL_Delay(2000);
	  //MotorReceiveCommand(0x09, 1.0, 0.0, 3.0, 20.0, 3, 1);
	  //MotorReceiveCommand(0x09, 0.1, 0.5, 3.0, 2.0, 5.0, 1);






	  //DisableMotor(0x00, 0x09);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 11;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
//	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
//		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData)
//				!= HAL_OK) {
//			/* Reception Error */
//			Error_Handler();
//		}
//		if(hfdcan->Instance == FDCAN1){
//			    uint8_t canid = decode_can_id(RxHeader.Identifier);
//			    if(canid == 0x09){
//			    MotorResponse(RxData, 0x09);
//				char msg[200];
//				sprintf(msg, "A: %f, S: %f, T: %f, T: %f", AngleCurrent[8], SpeedCurrent[8], TorqueCurrent[8], TempCurrent[8]);
//				HAL_UART_Transmit(&huart1, (uint8_t*)msg, 200, HAL_MAX_DELAY);
//			    }
//
//		}
//		//HAL_Delay(1000);
//	}
//
//}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
                Error_Handler();
            }

            if (hfdcan->Instance == FDCAN1) {
                uint8_t canid = decode_can_id(RxHeader.Identifier);
                char msg[100];
                int len = 0;
                switch(canid){
                case MOTOR_15:
					MotorResponse(RxData, MOTOR_15);
					char msg[100];

                	break;
                case MOTOR_16:
                    MotorResponse(RxData, MOTOR_16);
                	break;
                case MOTOR_17:
                    MotorResponse(RxData, MOTOR_17);
                	break;
                case MOTOR_18:
                    MotorResponse(RxData, MOTOR_18);
                	break;

                }

            }
            if (hfdcan->Instance == FDCAN3) {
                uint8_t canid = decode_can_id(RxHeader.Identifier);
                char msg[100];
                int len = 0;
                switch(canid){
                case MOTOR_20:
					MotorResponse(RxData, MOTOR_20);
					char msg[100];
                	break;
                case MOTOR_21:
                    MotorResponse(RxData, MOTOR_21);
                	break;
                case MOTOR_22:
                    MotorResponse(RxData, MOTOR_22);
                	break;
                case MOTOR_23:
                    MotorResponse(RxData, MOTOR_23);
                	break;

                }

            }
            if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
          			0) != HAL_OK) {
          		/* Notification Error */
          		Error_Handler();
          	}
//

        }
    }
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
                Error_Handler();
            }

            if (hfdcan->Instance == FDCAN2) {
                uint8_t canid = decode_can_id(RxHeader.Identifier);
                char msg[100];
                int len = 0;
                switch(canid){
                case MOTOR_14:
					MotorResponse(RxData, MOTOR_15);
					char msg[100];
					len = sprintf(msg, "A14: %.2f\r\n", AngleCurrent[13]);
					HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
                	break;
//                case MOTOR_16:
//                    MotorResponse(RxData, MOTOR_16);
//                    len = sprintf(msg, "A16: %.2f\r\n", AngleCurrent[15]);
//                    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
//                	break;
//                case MOTOR_17:
//                    MotorResponse(RxData, MOTOR_17);
//                    len = sprintf(msg, "A17: %.2f\r\n", AngleCurrent[16]);
//                    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
//                	break;
//                case MOTOR_18:
//                    MotorResponse(RxData, MOTOR_18);
//                    len = sprintf(msg, "A18: %.2f\r\n", AngleCurrent[17]);
//                    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
//                	break;

                }

            }

        }
    }
}

//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
//    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
//        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
//            Error_Handler();
//        }
//        if (hfdcan->Instance == FDCAN1) {
//            uint8_t canid = decode_can_id(RxHeader.Identifier);
//            if (canid == MOTOR_15) {
//                MotorResponse(RxData, MOTOR_15);
//                char msg[100];
//  //              int len = sprintf(msg, "A: %.2f\r\n, S: %.2f\r\n, T: %.2f\r\n, Temp: %.2f\r\n",
//  //                                AngleCurrent[8], SpeedCurrent[8], TorqueCurrent[8], TempCurrent[8]);
//                int len = sprintf(msg, "A16: %.2f\r\n",
//                                  AngleCurrent[14]);
//                HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
//            }
//            if (canid == MOTOR_16) {
//                MotorResponse(RxData, MOTOR_16);
//                char msg[100];
//  //              int len = sprintf(msg, "A: %.2f\r\n, S: %.2f\r\n, T: %.2f\r\n, Temp: %.2f\r\n",
//  //                                AngleCurrent[8], SpeedCurrent[8], TorqueCurrent[8], TempCurrent[8]);
//                int len = sprintf(msg, "A17: %.2f\r\n",
//                                  AngleCurrent[15]);
//                HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
//            }
//            if (canid == MOTOR_17) {
//                MotorResponse(RxData, MOTOR_17);
//                char msg[100];
//  //              int len = sprintf(msg, "A: %.2f\r\n, S: %.2f\r\n, T: %.2f\r\n, Temp: %.2f\r\n",
//  //                                AngleCurrent[8], SpeedCurrent[8], TorqueCurrent[8], TempCurrent[8]);
//                int len = sprintf(msg, "A18: %.2f\r\n",
//                                  AngleCurrent[16]);
//                HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
//            }
//            if (canid == MOTOR_18) {
//                MotorResponse(RxData, MOTOR_18);
//                char msg[100];
//  //              int len = sprintf(msg, "A: %.2f\r\n, S: %.2f\r\n, T: %.2f\r\n, Temp: %.2f\r\n",
//  //                                AngleCurrent[8], SpeedCurrent[8], TorqueCurrent[8], TempCurrent[8]);
//                int len = sprintf(msg, "A19: %.2f\r\n",
//                                  AngleCurrent[17]);
//                HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
//            }
//
//        }
//    }
//}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x30004000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

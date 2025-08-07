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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32wlxx_nucleo_radio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef hlpuart1;

SUBGHZ_HandleTypeDef hsubghz;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SUBGHZ_Init(void);
/* USER CODE BEGIN PFP */
void Configure_2FSK_Radio(void);
void Transmit_2FSK_Data(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Redirect printf() to UART
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

static uint32_t delay = 1000;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_SUBGHZ_Init();
  /* USER CODE BEGIN 2 */
  // Configure the radio for 2FSK transmission
  Configure_2FSK_Radio();
  printf("2FSK Radio configuration complete\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Transmit 2FSK data packet
    Transmit_2FSK_Data();
    
    // Blink LED to show activity
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    
    // Wait before next transmission
    HAL_Delay(delay);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SUBGHZ Initialization Function
  * @param None
  * @retval None
  */
static void MX_SUBGHZ_Init(void)
{

  /* USER CODE BEGIN SUBGHZ_Init 0 */

  /* USER CODE END SUBGHZ_Init 0 */

  /* USER CODE BEGIN SUBGHZ_Init 1 */

  /* USER CODE END SUBGHZ_Init 1 */
  hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
  if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SUBGHZ_Init 2 */

  /* USER CODE END SUBGHZ_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FE_CTRL3_Pin|FE_CTRL2_Pin|FE_CTRL1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD1_Pin LD2_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD2_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FE_CTRL3_Pin FE_CTRL2_Pin FE_CTRL1_Pin */
  GPIO_InitStruct.Pin = FE_CTRL3_Pin|FE_CTRL2_Pin|FE_CTRL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin B2_Pin */
  GPIO_InitStruct.Pin = B1_Pin|B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : B3_Pin */
  GPIO_InitStruct.Pin = B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Configure_2FSK_Radio(void)
{
  uint8_t buffer[16];
  HAL_StatusTypeDef status;
  
  // Initialize BSP radio (required for RF switch control)
  BSP_RADIO_Init();
  printf("BSP Radio initialized\r\n");
  
  // Set standby mode
  buffer[0] = 0x01; // STANDBY_RC
  status = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, buffer, 1);
  if (status != HAL_OK) {
    printf("ERROR: Failed to set standby mode\r\n");
    return;
  }
  HAL_Delay(10);
  printf("Radio set to standby mode\r\n");
  
  // Set packet type to 2FSK
  buffer[0] = 0x00; // PACKET_TYPE_GFSK
  status = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACKETTYPE, buffer, 1);
  if (status != HAL_OK) {
    printf("ERROR: Failed to set packet type\r\n");
    return;
  }
  printf("Packet type set to 2FSK\r\n");
  
  // Set RF frequency (915 MHz)
  // Frequency calculation: Freq = (FreqInHz * 2^25) / 32MHz
  uint32_t freq = (uint32_t)((double)915000000 * 33554432 / 32000000); // Convert to register value
  buffer[0] = (freq >> 24) & 0xFF;
  buffer[1] = (freq >> 16) & 0xFF;
  buffer[2] = (freq >> 8) & 0xFF;
  buffer[3] = freq & 0xFF;
  status = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RFFREQUENCY, buffer, 4);
  if (status != HAL_OK) {
    printf("ERROR: Failed to set RF frequency\r\n");
    return;
  }
  printf("RF frequency set to 915 MHz\r\n");
  
  // Set modulation parameters for 2FSK
  buffer[0] = 0x06; // BitRate MSB (50kbps)
  buffer[1] = 0x40; // BitRate middle
  buffer[2] = 0x00; // BitRate LSB
  buffer[3] = 0x08; // ModulationShaping - Gaussian BT 0.5
  buffer[4] = 0x0A; // Bandwidth - 117.3 kHz
  buffer[5] = 0x00; // FreqDev MSB (25 kHz)
  buffer[6] = 0x66; // FreqDev middle  
  buffer[7] = 0x66; // FreqDev LSB
  status = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_MODULATIONPARAMS, buffer, 8);
  if (status != HAL_OK) {
    printf("ERROR: Failed to set modulation parameters\r\n");
    return;
  }
  printf("2FSK modulation parameters configured\r\n");
  
  // Set packet parameters
  buffer[0] = 0x00; // Preamble length MSB
  buffer[1] = 0x08; // Preamble length LSB (8 bits)
  buffer[2] = 0x04; // Preamble detector length (4 bits)
  buffer[3] = 0x07; // SyncWord length (7 bits)
  buffer[4] = 0x01; // AddrComp (Node address filtering disabled)
  buffer[5] = 0x00; // Packet type (fixed length)
  buffer[6] = 0x0A; // Payload length (10 bytes)
  buffer[7] = 0x01; // CRC type (1 byte)
  buffer[8] = 0x02; // Whitening (enabled)
  status = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACKETPARAMS, buffer, 9);
  if (status != HAL_OK) {
    printf("ERROR: Failed to set packet parameters\r\n");
    return;
  }
  printf("Packet parameters configured\r\n");
  
  // Set TX power (+14 dBm)
  buffer[0] = 0x0E; // Power level
  buffer[1] = 0x07; // Power amplifier ramp time
  status = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXPARAMS, buffer, 2);
  if (status != HAL_OK) {
    printf("ERROR: Failed to set TX parameters\r\n");
    return;
  }
  printf("TX power set to +14 dBm\r\n");
}

void Transmit_2FSK_Data(void)
{
  uint8_t buffer[10] = "Hello RF!"; // 9 chars + null terminator = 10 bytes
  HAL_StatusTypeDef status;
  
  // Configure RF switch for TX (High Power mode)
  BSP_RADIO_ConfigRFSwitch(RADIO_SWITCH_RFO_HP);
  printf("RF switch configured for TX (High Power)\r\n");
  
  // Write payload to buffer
  status = HAL_SUBGHZ_WriteBuffer(&hsubghz, 0x00, buffer, 10);
  if (status != HAL_OK) {
    printf("ERROR: Failed to write buffer\r\n");
    return;
  }
  printf("Data written to TX buffer: %s\r\n", buffer);
  
  // Set TX mode (no timeout)
  buffer[0] = 0x00; // Timeout MSB
  buffer[1] = 0x00; // Timeout middle
  buffer[2] = 0x00; // Timeout LSB
  status = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TX, buffer, 3);
  if (status != HAL_OK) {
    printf("ERROR: Failed to start TX\r\n");
    return;
  }
  printf("Transmitting 2FSK packet...\r\n");
  
  // Wait for TX done (simplified - in real app use interrupts)
  HAL_Delay(100);
  
  // Turn off RF switch after transmission
  BSP_RADIO_ConfigRFSwitch(RADIO_SWITCH_OFF);
  printf("RF switch turned off\r\n");
  printf("Transmission completed\r\n");
}
/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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

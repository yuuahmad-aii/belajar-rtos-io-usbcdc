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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usbd_cdc_if.h" // Untuk CDC_Transmit_FS
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	char command; // Karakter perintah 'A' - 'Z'
} OutputCommand_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_INPUTS 8
#define NUM_SR_OUTPUTS 13 // Jumlah output melalui Shift Register

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* Definitions for InputScanTask */
osThreadId_t InputScanTaskHandle;
const osThreadAttr_t InputScanTask_attributes = {
  .name = "InputScanTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OutputCtrlTask */
osThreadId_t OutputCtrlTaskHandle;
const osThreadAttr_t OutputCtrlTask_attributes = {
  .name = "OutputCtrlTask",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OutputCmdQueue */
osMessageQueueId_t OutputCmdQueueHandle;
const osMessageQueueAttr_t OutputCmdQueue_attributes = {
  .name = "OutputCmdQueue"
};
/* USER CODE BEGIN PV */

// Array untuk menyimpan konfigurasi pin input
// Pastikan definisi pin ini (INPUT_PIN_x_Pin dan INPUT_PIN_x_GPIO_Port) ADA di main.h atau didefinisikan di atas
GPIO_TypeDef *input_ports[NUM_INPUTS] = {
INPUT_PIN_0_GPIO_Port, INPUT_PIN_1_GPIO_Port, INPUT_PIN_2_GPIO_Port,
INPUT_PIN_3_GPIO_Port,
INPUT_PIN_4_GPIO_Port, INPUT_PIN_5_GPIO_Port, INPUT_PIN_6_GPIO_Port,
INPUT_PIN_7_GPIO_Port };
uint16_t input_pins[NUM_INPUTS] = {
INPUT_PIN_0_Pin, INPUT_PIN_1_Pin, INPUT_PIN_2_Pin, INPUT_PIN_3_Pin,
INPUT_PIN_4_Pin, INPUT_PIN_5_Pin, INPUT_PIN_6_Pin, INPUT_PIN_7_Pin };

// State untuk 16 bit output shift register (kita gunakan 13 bit)
static uint16_t current_sr_output_state = 0x0000; // Semua output mati awalnya

char usb_rx_buffer[16];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void StartInputScanTask(void *argument);
void StartOutputControlTask(void *argument);

/* USER CODE BEGIN PFP */
void InputScanTask_Entry(void *argument);
void OutputControlTask_Entry(void *argument);
static void SR_UpdateOutputs(void);
static void SR_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// int _write(int file, char *ptr, int len) {
// 	uint32_t timeout = 0;
// 	while (CDC_Transmit_FS((uint8_t*) ptr, len) == USBD_BUSY) {
// 		osDelay(1);
// 		timeout++;
// 		if (timeout > 50) {
// 			return -1;
// 		}
// 	}
// 	return len;
// }
int _write(int file, char *ptr, int len) {
	CDC_Transmit_FS((uint8_t*) ptr, len);
	return len;
}
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
	MX_USB_DEVICE_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	SR_Init(); // Inisialisasi shift register ke keadaan mati
	printf("Sistem dimulai... Kontrol 13 output via 74HC595 (SPI)\r\n");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of OutputCmdQueue */
  OutputCmdQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &OutputCmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of InputScanTask */
  InputScanTaskHandle = osThreadNew(StartInputScanTask, NULL, &InputScanTask_attributes);

  /* creation of OutputCtrlTask */
  OutputCtrlTaskHandle = osThreadNew(StartOutputControlTask, NULL, &OutputCtrlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HC595_LOAD_GPIO_Port, HC595_LOAD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INPUT_PIN_0_Pin */
  GPIO_InitStruct.Pin = INPUT_PIN_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(INPUT_PIN_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HC595_LOAD_Pin */
  GPIO_InitStruct.Pin = HC595_LOAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HC595_LOAD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_PIN_1_Pin INPUT_PIN_2_Pin INPUT_PIN_3_Pin INPUT_PIN_4_Pin
                           INPUT_PIN_5_Pin INPUT_PIN_6_Pin INPUT_PIN_7_Pin INPUT_PIN_8_Pin */
  GPIO_InitStruct.Pin = INPUT_PIN_1_Pin|INPUT_PIN_2_Pin|INPUT_PIN_3_Pin|INPUT_PIN_4_Pin
                          |INPUT_PIN_5_Pin|INPUT_PIN_6_Pin|INPUT_PIN_7_Pin|INPUT_PIN_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Inisialisasi output shift register ke keadaan mati.
 */
static void SR_Init(void) {
	current_sr_output_state = 0x0000; // Semua output mati
	SR_UpdateOutputs();
}

/**
 * @brief  Mengirim state output saat ini ke shift register 74HC595.
 */
static void SR_UpdateOutputs(void) {
	uint8_t bytes_to_send[2];

	// Data untuk 74HC595 kedua (output 8-12, sisanya tidak terpakai jika hanya 13 output)
	// Bit 8 dari current_sr_output_state -> Q0 dari SR kedua
	// Bit 12 dari current_sr_output_state -> Q4 dari SR kedua
	bytes_to_send[0] = (uint8_t) ((current_sr_output_state >> 8) & 0xFF);

	// Data untuk 74HC595 pertama (output 0-7)
	// Bit 0 dari current_sr_output_state -> Q0 dari SR pertama
	// Bit 7 dari current_sr_output_state -> Q7 dari SR pertama
	bytes_to_send[1] = (uint8_t) (current_sr_output_state & 0xFF);

	HAL_GPIO_WritePin(HC595_LOAD_GPIO_Port, HC595_LOAD_Pin, GPIO_PIN_RESET); // Latch LOW

	// Kirim data: byte untuk SR terjauh dulu, lalu byte untuk SR terdekat
	if (HAL_SPI_Transmit(&hspi1, &bytes_to_send[0], 1, 100) != HAL_OK) { // Data untuk SR2
		printf("SPI Transmit Error SR2\r\n");
	}
	if (HAL_SPI_Transmit(&hspi1, &bytes_to_send[1], 1, 100) != HAL_OK) { // Data untuk SR1
		printf("SPI Transmit Error SR1\r\n");
	}

	HAL_GPIO_WritePin(HC595_LOAD_GPIO_Port, HC595_LOAD_Pin, GPIO_PIN_SET);// Latch HIGH (data masuk ke output)
	HAL_GPIO_WritePin(HC595_LOAD_GPIO_Port, HC595_LOAD_Pin, GPIO_PIN_RESET); // Latch LOW kembali (siap untuk data berikutnya)
}

void USB_CDC_RxHandler(uint8_t *Buf, uint32_t Len) {
	if (Len > 0 && Len < sizeof(usb_rx_buffer)) {
		memcpy(usb_rx_buffer, Buf, Len);
		usb_rx_buffer[Len] = '\0';

		// Perinta	h output adalah satu karakter 'A' - 'Z'
		if (Len == 1 && usb_rx_buffer[0] >= 'A' && usb_rx_buffer[0] <= 'Z') {
			OutputCommand_t cmd_msg;
			cmd_msg.command = usb_rx_buffer[0];
			if (osMessageQueuePut(OutputCmdQueueHandle, &cmd_msg, 0U, 0U)
					!= osOK) {
				printf("Error: Gagal mengirim perintah output ke queue.\r\n");
			}
		} else {
			printf("Perintah serial tidak dikenal: %s\r\n", usb_rx_buffer);
		}
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartInputScanTask */
/**
 * @brief  Function implementing the InputScanTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartInputScanTask */
void StartInputScanTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	char status_string[NUM_INPUTS + 3];
	printf("InputScanTask dimulai.\r\n");
	/* Infinite loop */
	for (;;) {
		int char_idx = 0;
		for (int i = 0; i < NUM_INPUTS; i++) {
			// Pastikan input_ports[i] dan input_pins[i] valid dan terdefinisi
			if (input_ports[i] != NULL) {
				if (HAL_GPIO_ReadPin(input_ports[i], input_pins[i])
						== GPIO_PIN_RESET) // Active LOW
						{
					status_string[char_idx++] = 'A' + i;
				}
			}
		}

		if (char_idx > 0) {
			status_string[char_idx++] = '\r';
			status_string[char_idx++] = '\n';
			status_string[char_idx] = '\0';
			printf("%s", status_string);
		}
		osDelay(100);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartOutputControlTask */
/**
 * @brief Function implementing the OutputControlTa thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartOutputControlTask */
void StartOutputControlTask(void *argument)
{
  /* USER CODE BEGIN StartOutputControlTask */
	OutputCommand_t received_cmd;
	osStatus_t status;
	printf("OutputControlTask dimulai.\r\n");
	/* Infinite loop */
	for (;;) {
		status = osMessageQueueGet(OutputCmdQueueHandle, &received_cmd, NULL,
		osWaitForever);

		if (status == osOK) {
			int output_index = -1;
			uint8_t set_on = 0; // 0 untuk OFF, 1 untuk ON

			if (received_cmd.command >= 'A' && received_cmd.command <= 'Z') {
				int command_val = received_cmd.command - 'A'; // 0 untuk A, 1 untuk B, ..., 25 untuk Z
				output_index = command_val / 2;

				if (command_val % 2 == 0) { // Perintah genap (A, C, E, ...) -> ON
					set_on = 1;
				} else { // Perintah ganjil (B, D, F, ...) -> OFF
					set_on = 0;
				}

				if (output_index >= 0 && output_index < NUM_SR_OUTPUTS) {
					if (set_on) {
						current_sr_output_state |= (1 << output_index); // Set bit
					} else {
						current_sr_output_state &= ~(1 << output_index); // Clear bit
					}
					SR_UpdateOutputs(); // Kirim state baru ke shift registers
					printf("Output SR %d %s (State: 0x%04X)\r\n", output_index,
							set_on ? "ON" : "OFF", current_sr_output_state);
				} else {
					printf(
							"Error: Indeks Output SR tidak valid (%d) untuk perintah %c.\r\n",
							output_index, received_cmd.command);
				}
			} else {
				printf(
						"Error: Perintah output SR tidak dikenal di task: %c\r\n",
						received_cmd.command);
			}
		}
	}
  /* USER CODE END StartOutputControlTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
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
	printf("!!! ERROR HANDLER DIPANGGIL !!!\r\n");
	while (1) {
		HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
		for (volatile uint32_t i = 0; i < SystemCoreClock / 50 / 2; i++)
			;
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

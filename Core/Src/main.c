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

#define BLINKING_LED_SR_OUTPUT_INDEX 14 // Output ke-0 dari SR untuk LED berkedip

// Default blinking times
#define DEFAULT_BLINK_OFF_MS 1000 // 1 detik
#define DEFAULT_BLINK_ON_MS  500  // 0.5 detik

// Alamat Flash untuk menyimpan konfigurasi delay LED berkedip
// Pastikan alamat ini berada di area Flash yang tidak digunakan oleh program Anda
// Untuk STM32F103CBT6 (128KB Flash), page size 1KB. Pilih page dekat akhir.
#define BLINK_CONFIG_FLASH_PAGE_ADDR  ((uint32_t)0x0801F000) // Alamat awal page untuk konfigurasi blink
#define BLINK_OFF_TIME_OFFSET       0   // Offset untuk delay OFF dari awal page
#define BLINK_ON_TIME_OFFSET        4   // Offset untuk delay ON (setelah delay OFF, uint32_t = 4 bytes)

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
/* Definitions for TimerOliTask */
osThreadId_t TimerOliTaskHandle;
const osThreadAttr_t TimerOliTask_attributes = {
  .name = "TimerOliTask",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityHigh,
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

// Variabel untuk durasi kedip LED (dalam milidetik)
static uint32_t g_blinking_led_off_time_ms = DEFAULT_BLINK_OFF_MS;
static uint32_t g_blinking_led_on_time_ms = DEFAULT_BLINK_ON_MS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void StartInputScanTask(void *argument);
void StartOutputControlTask(void *argument);
void StartTimerOliTask(void *argument);

/* USER CODE BEGIN PFP */
void InputScanTask_Entry(void *argument);
void OutputControlTask_Entry(void *argument);
static void SR_UpdateOutputs(void);
static void SR_Init(void);
static uint32_t Flash_Read_Blinker_Word(uint32_t address);
static HAL_StatusTypeDef Flash_Write_Blinker_Config(uint32_t off_time_ms,
		uint32_t on_time_ms);
static void Load_Blinker_Config_From_Flash(void);
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
	Load_Blinker_Config_From_Flash(); // Muat konfigurasi blinker dari Flash
	SR_Init(); // Inisialisasi shift register ke keadaan mati
	printf("Sistem dimulai... Kontrol 13 output via 74HC595 (SPI)\r\n");
	printf(
			"LED Berkedip (dari Flash/Default): Output %d, ON: %lu ms, OFF: %lu ms\r\n",
			BLINKING_LED_SR_OUTPUT_INDEX, g_blinking_led_on_time_ms,
			g_blinking_led_off_time_ms);
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

  /* creation of TimerOliTask */
  TimerOliTaskHandle = osThreadNew(StartTimerOliTask, NULL, &TimerOliTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  if (InputScanTaskHandle == NULL) {
    printf("FATAL Error: Gagal membuat InputScanTask!\r\n");
    Error_Handler();
  }

  if (OutputCtrlTaskHandle == NULL) {
    printf("FATAL Error: Gagal membuat OutputControlTask!\r\n");
    Error_Handler();
  }

  if (TimerOliTaskHandle == NULL) {
    // Ini yang paling mungkin terjadi jika heap habis
    printf("FATAL Error: Gagal membuat BlinkingLedTask! (Cek configTOTAL_HEAP_SIZE)\r\n");
    Error_Handler();
  }

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
 * @brief  Membaca satu word (32-bit) dari alamat Flash.
 * @param  address: Alamat Flash yang akan dibaca.
 * @retval Nilai yang tersimpan di alamat tersebut.
 */
static uint32_t Flash_Read_Blinker_Word(uint32_t address) {
	return *(volatile uint32_t*) address;
}

/**
 * @brief  Menulis konfigurasi delay blinker ke Flash.
 * Fungsi ini akan menghapus satu page Flash dan menulis kedua nilai delay.
 * @param  off_time_ms: Waktu LED mati dalam milidetik.
 * @param  on_time_ms: Waktu LED menyala dalam milidetik.
 * @retval HAL_StatusTypeDef: Status operasi Flash.
 */
static HAL_StatusTypeDef Flash_Write_Blinker_Config(uint32_t off_time_ms,
		uint32_t on_time_ms) {
	HAL_StatusTypeDef status = HAL_OK;
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError = 0;

	status = HAL_FLASH_Unlock(); // Unlock Flash untuk operasi tulis/hapus
	if (status != HAL_OK) {
		printf("Error: Gagal unlock Flash! (%d)\r\n", status);
		return status;
	}

	// Konfigurasi untuk menghapus page
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = BLINK_CONFIG_FLASH_PAGE_ADDR;
	EraseInitStruct.NbPages = 1;

	// Hapus page
	status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

	if (status == HAL_OK) {
		// Tulis nilai OFF time
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
		BLINK_CONFIG_FLASH_PAGE_ADDR + BLINK_OFF_TIME_OFFSET, off_time_ms);
		if (status != HAL_OK) {
			printf("Error: Gagal menulis OFF time ke Flash! (%d)\r\n", status);
			HAL_FLASH_Lock();
			return status;
		}

		// Tulis nilai ON time
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
		BLINK_CONFIG_FLASH_PAGE_ADDR + BLINK_ON_TIME_OFFSET, on_time_ms);
		if (status != HAL_OK) {
			printf("Error: Gagal menulis ON time ke Flash! (%d)\r\n", status);
		}
	} else {
		printf(
				"Error: Gagal menghapus page Flash untuk blink config (PageError: 0x%lX, Status: %d)!\r\n",
				PageError, status);
	}

	HAL_FLASH_Lock(); // Lock Flash kembali
	return status;
}

/**
 * @brief  Memuat konfigurasi delay blinker dari Flash ke variabel global.
 * Jika Flash kosong atau data tidak valid, gunakan nilai default dan tulis ke Flash.
 */
static void Load_Blinker_Config_From_Flash(void) {
	uint32_t stored_off_time = Flash_Read_Blinker_Word(
	BLINK_CONFIG_FLASH_PAGE_ADDR + BLINK_OFF_TIME_OFFSET);
	uint32_t stored_on_time = Flash_Read_Blinker_Word(
	BLINK_CONFIG_FLASH_PAGE_ADDR + BLINK_ON_TIME_OFFSET);
	uint8_t write_defaults_to_flash = 0;

	// 0xFFFFFFFF adalah nilai umum untuk Flash yang terhapus (belum ditulis)
	// Nilai 0 juga bisa dianggap tidak valid untuk durasi.
	if (stored_off_time == 0xFFFFFFFF || stored_off_time == 0) {
		g_blinking_led_off_time_ms = DEFAULT_BLINK_OFF_MS;
		write_defaults_to_flash = 1;
		printf(
				"Info: Waktu OFF LED dari Flash tidak valid, menggunakan default: %lu ms\r\n",
				g_blinking_led_off_time_ms);
	} else {
		g_blinking_led_off_time_ms = stored_off_time;
	}

	if (stored_on_time == 0xFFFFFFFF || stored_on_time == 0) {
		g_blinking_led_on_time_ms = DEFAULT_BLINK_ON_MS;
		write_defaults_to_flash = 1;
		printf(
				"Info: Waktu ON LED dari Flash tidak valid, menggunakan default: %lu ms\r\n",
				g_blinking_led_on_time_ms);
	} else {
		g_blinking_led_on_time_ms = stored_on_time;
	}

	if (write_defaults_to_flash) {
		printf(
				"Info: Menulis konfigurasi blinker awal/default ke Flash...\r\n");
		if (Flash_Write_Blinker_Config(g_blinking_led_off_time_ms,
				g_blinking_led_on_time_ms) == HAL_OK) {
			printf(
					"Info: Konfigurasi blinker default berhasil ditulis ke Flash.\r\n");
		} else {
			printf(
					"Error: Gagal menulis konfigurasi blinker default ke Flash saat startup.\r\n");
		}
	}
}

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

	HAL_GPIO_WritePin(HC595_LOAD_GPIO_Port, HC595_LOAD_Pin, GPIO_PIN_SET); // Latch HIGH (data masuk ke output)
	HAL_GPIO_WritePin(HC595_LOAD_GPIO_Port, HC595_LOAD_Pin, GPIO_PIN_RESET); // Latch LOW kembali (siap untuk data berikutnya)
}

void USB_CDC_RxHandler(uint8_t *Buf, uint32_t Len) {
	if (Len > 0 && Len < sizeof(usb_rx_buffer)) {
		memcpy(usb_rx_buffer, Buf, Len);
		usb_rx_buffer[Len] = '\0';

		if (Len == 1 && usb_rx_buffer[0] >= 'A' && usb_rx_buffer[0] <= 'Z') {
			OutputCommand_t cmd_msg;
			cmd_msg.command = usb_rx_buffer[0];
			if (osMessageQueuePut(OutputCmdQueueHandle, &cmd_msg, 0U, 0U)
					!= osOK) {
				printf("Error: Gagal mengirim perintah output ke queue.\r\n");
			}
		} else if (strncmp(usb_rx_buffer, "$1=", 3) == 0) // Perintah untuk Waktu OFF LED Berkedip
				{
			char *value_str = usb_rx_buffer + 3;
			long val_seconds = atol(value_str);
			if (val_seconds > 0 && val_seconds <= 3600) { // Batasi hingga 1 jam (3600 detik)
				uint32_t new_off_time_ms = (uint32_t) val_seconds * 1000;
				if (new_off_time_ms != g_blinking_led_off_time_ms) {
					g_blinking_led_off_time_ms = new_off_time_ms;
					printf(
							"Info: Waktu OFF LED Berkedip diatur ke %lu ms ($1=%ld). Menyimpan ke Flash...\r\n",
							g_blinking_led_off_time_ms, val_seconds);
					if (Flash_Write_Blinker_Config(g_blinking_led_off_time_ms,
							g_blinking_led_on_time_ms) != HAL_OK) {
						printf(
								"Error: Gagal menyimpan konfigurasi blinker ke Flash setelah update $1.\r\n");
					}
				} else {
					printf(
							"Info: Waktu OFF LED Berkedip tidak berubah ($1=%ld).\r\n",
							val_seconds);
				}
			} else {
				printf(
						"Error: Nilai $1 tidak valid (%s). Harus antara 1-3600 detik.\r\n",
						value_str);
			}
		} else if (strncmp(usb_rx_buffer, "$2=", 3) == 0) // Perintah untuk Waktu ON LED Berkedip
				{
			char *value_str = usb_rx_buffer + 3;
			long val_seconds = atol(value_str);
			if (val_seconds > 0 && val_seconds <= 3600) { // Batasi hingga 1 jam
				uint32_t new_on_time_ms = (uint32_t) val_seconds * 1000;
				if (new_on_time_ms != g_blinking_led_on_time_ms) {
					g_blinking_led_on_time_ms = new_on_time_ms;
					printf(
							"Info: Waktu ON LED Berkedip diatur ke %lu ms ($2=%ld). Menyimpan ke Flash...\r\n",
							g_blinking_led_on_time_ms, val_seconds);
					if (Flash_Write_Blinker_Config(g_blinking_led_off_time_ms,
							g_blinking_led_on_time_ms) != HAL_OK) {
						printf(
								"Error: Gagal menyimpan konfigurasi blinker ke Flash setelah update $2.\r\n");
					}
				} else {
					printf(
							"Info: Waktu ON LED Berkedip tidak berubah ($2=%ld).\r\n",
							val_seconds);
				}
			} else {
				printf(
						"Error: Nilai $2 tidak valid (%s). Harus antara 1-3600 detik.\r\n",
						value_str);
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

/* USER CODE BEGIN Header_StartTimerOliTask */
/**
 * @brief Function implementing the TimerOliTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTimerOliTask */
void StartTimerOliTask(void *argument)
{
  /* USER CODE BEGIN StartTimerOliTask */
	printf("BlinkingLedTask dimulai untuk Output SR %d.\r\n",
			BLINKING_LED_SR_OUTPUT_INDEX);
	/* Infinite loop */
	for (;;) {
#if defined(USER_LED_Pin) && defined(USER_LED_GPIO_Port)
    HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin); // Tes dengan LED sederhana
    printf("DEBUG: BlinkingLedTask USER_LED Toggled.\r\n");
    #else
    printf("DEBUG: BlinkingLedTask loop berjalan - tidak ada USER_LED untuk tes.\r\n");
    #endif
    osDelay(1000); // Delay 1 detik
	}
  /* USER CODE END StartTimerOliTask */
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

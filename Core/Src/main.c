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
#include <stdbool.h>
#include <math.h>
#include "usbd_cdc_if.h" // Untuk CDC_Transmit_FS
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	char command; // Karakter perintah 'A' - 'Z'
} OutputCommand_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// config related defines
#define FIRMWARE_VERSION "1.0.0"  // Versi firmware
#define DEBUG_MODE 0			  // 0 = non-debug, 1 = debug
#define MODBUS_VERBOSE 0		  // 0 = non-verbose, 1 = verbose
#define MODBUS_SLAVE_ID 0x01	  // ID slave Modbus RTU
#define MODBUS_UART_HANDLE huart1 // uart yang digunakan untuk Modbus RTU
#define DEBOUNCE_TIME_MS 0		  // Waktu debounce dalam milidetik
#define NUM_INPUTS 9			  // Jumlah input digital biasa
#define NUM_SR_OUTPUTS 13		  // Jumlah output melalui Shift Register
#define F_CLK_TIM2 72000000UL
#define DISPLAY_BUFFER_SIZE 4095 // Buffer untuk menampilkan data di USB CDC

#define BLINKING_LED_SR_OUTPUT_INDEX 14 // Output ke-0 dari SR untuk LED berkedip
#define LINKED_INPUT_INDEX 7			// Input ke-8 (indeks 7)
#define LINKED_SR_OUTPUT_INDEX 12		// Output SR ke-13 (indeks 12)

// Pin mask untuk input digital biasa
// Gunakan bitmask untuk mengaktifkan/menonaktifkan input
#define PIN_MASK_ORIENT_OK 0
#define PIN_MASK_PROXY_TOOLS 1
#define PIN_MASK_PROXY_UMB_A 2
#define PIN_MASK_PROXY_UMB_B 3
#define PIN_MASK_PROXY_CLAMP_A 4
#define PIN_MASK_PROXY_CLAMP_B 5
#define PIN_MASK_SENSOR_OLI 6
#define PIN_MASK_INPUT_CLAMP 7
#define PIN_MASK_INPUT_UNCLAMP 8

// Shift Register Pertama (Paling Dekat dengan Mikrokontroler)
// Bit 0-7 dari total data
#define PIN_OUTPUT_SR1_Q0 0
#define PIN_OUTPUT_BUZZER 1
#define PIN_OUTPUT_LED_RED 2
#define PIN_OUTPUT_LED_GREEN 3
#define PIN_OUTPUT_CLAMP 4
#define PIN_OUTPUT_UMB 5
#define PIN_OUTPUT_OIL_PUMP 6
#define PIN_OUTPUT_ORIENT 7
// Shift Register Kedua
// Bit 8-15 dari total data
#define PIN_OUTPUT_SR2_Q0 8
#define PIN_OUTPUT_ATC_CW 9
#define PIN_OUTPUT_ATC_CCW 10
#define PIN_OUTPUT_ATC_LOCK 11
#define PIN_OUTPUT_ATC_UNLOCK 12
#define PIN_OUTPUT_SR2_Q5 13
#define PIN_OUTPUT_SR2_Q6 14
#define PIN_OUTPUT_SR2_Q7 15

// Default blinking times
#define DEFAULT_BLINK_OFF_MS 1000 // 1 detik
#define DEFAULT_BLINK_ON_MS 500	  // 0.5 detik

// Alamat Flash untuk menyimpan konfigurasi delay LED berkedip
// Pastikan alamat ini berada di area Flash yang tidak digunakan oleh program
// Untuk STM32F103CBT6 (128KB Flash), page size 1KB. Pilih page dekat akhir.
#define CONFIG_FLASH_PAGE_ADDR ((uint32_t)0x0801F000) // Alamat awal page
#define BLINK_OFF_TIME_OFFSET 0						  //$1 delay OFF
#define BLINK_ON_TIME_OFFSET 4						  //$2 delay ON (setelah delay OFF, uint32_t = 4 bytes)
#define INPUT_INVERSION_MASK_OFFSET 8				  //$3 uint8_t (disimpan sebagai uint32_t untuk kemudahan alignment Flash)
#define SSV_VARIATION_RPM_OFFSET 12					  //$4 uint32_t (stores uint16_t)
#define SSV_CYCLE_MS_OFFSET 16						  //$5 uint32_t
#define MIN_SPINDLE_RPM_OFFSET 20					  //$6 uint32_t (stores uint16_t)
#define MAX_SPINDLE_RPM_OFFSET 24					  //$7 uint32_t (stores uint16_t)

#define DEFAULT_INPUT_INVERSION_MASK 0x00 // Semua input active low (tidak ada inversi)
#define DEFAULT_SSV_VARIATION_RPM 0
#define DEFAULT_SSV_CYCLE_MS 0
#define DEFAULT_MIN_SPINDLE_RPM 0	 // RPM minimum default
#define DEFAULT_MAX_SPINDLE_RPM 6000 // RPM maksimum default

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for InputScanTask */
osThreadId_t InputScanTaskHandle;
const osThreadAttr_t InputScanTask_attributes = {
	.name = "InputScanTask",
	.stack_size = 256 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for OutputCtrlTask */
osThreadId_t OutputCtrlTaskHandle;
const osThreadAttr_t OutputCtrlTask_attributes = {
	.name = "OutputCtrlTask",
	.stack_size = 192 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for TimerOliTask */
osThreadId_t TimerOliTaskHandle;
const osThreadAttr_t TimerOliTask_attributes = {
	.name = "TimerOliTask",
	.stack_size = 192 * 4,
	.priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for ModbusSpindle */
osThreadId_t ModbusSpindleHandle;
const osThreadAttr_t ModbusSpindle_attributes = {
	.name = "ModbusSpindle",
	.stack_size = 348 * 4,
	.priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for OutputCmdQueue */
osMessageQueueId_t OutputCmdQueueHandle;
const osMessageQueueAttr_t OutputCmdQueue_attributes = {.name =
															"OutputCmdQueue"};
/* USER CODE BEGIN PV */

// Array untuk menyimpan konfigurasi pin input
// Pastikan definisi pin ini (INPUT_PIN_x_Pin dan INPUT_PIN_x_GPIO_Port) ADA di main.h atau didefinisikan di atas
GPIO_TypeDef *input_ports[NUM_INPUTS] = {
	ORIENT_OK_GPIO_Port, PROXY_TOOLS_GPIO_Port, PROXY_UMB_A_GPIO_Port,
	PROXY_UMB_B_GPIO_Port, PROXY_CLAMP_A_GPIO_Port, PROXY_CLAMP_B_GPIO_Port,
	SENSOR_OLI_GPIO_Port, INPUT_CLAMP_GPIO_Port, INPUT_UNCLAMP_GPIO_Port};
uint16_t input_pins[NUM_INPUTS] = {
	ORIENT_OK_Pin, PROXY_TOOLS_Pin, PROXY_UMB_A_Pin,
	PROXY_UMB_B_Pin, PROXY_CLAMP_A_Pin, PROXY_CLAMP_B_Pin,
	SENSOR_OLI_Pin, INPUT_CLAMP_Pin, INPUT_UNCLAMP_Pin};
// Nama pin input untuk $P (sesuaikan dengan urutan di atas)
const char *input_pin_names[NUM_INPUTS] = {"ORIENT_OK", "PROXY_TOOLS",
										   "PROXY_UMB_A", "PROXY_UMB_B", "PROXY_CLAMP_A", "PROXY_CLAMP_B",
										   "SENSOR_OLI", "INPUT_CLAMP", "INPUT_UNCLAMP"};

const char *input_port_letters[NUM_INPUTS] = {
	"PB", // Port untuk ORIENT_OK
	"PB", // Port untuk PROXY_TOOLS
	"PA", // Port untuk PROXY_UMB_A
	"PB", // Port untuk PROXY_UMB_B
	"PB", // Port untuk PROXY_CLAMP_A
	"PB", // Port untuk PROXY_CLAMP_B
	"PB", // Port untuk SENSOR_OLI
	"PB", // Port untuk INPUT_CLAMP
	"PB"  // Port untuk INPUT_UNCLAMP
};
const int input_port_number[NUM_INPUTS] = {
	0,	// ORIENT_OK
	1,	// PROXY_TOOLS
	4,	// PROXY_UMB_A
	10, // PROXY_UMB_B
	11, // PROXY_CLAMP_A
	12, // PROXY_CLAMP_B
	13, // SENSOR_OLI
	14, // INPUT_CLAMP
	15	// INPUT_UNCLAMP
};

// State untuk 16 bit output shift register (kita gunakan 13 bit)
static uint16_t current_sr_output_state = 0x0000; // Semua output mati awalnya

char usb_rx_buffer[16];

// Variabel untuk durasi kedip LED (dalam milidetik)
static uint32_t g_blinking_led_off_time_ms = DEFAULT_BLINK_OFF_MS;
static uint32_t g_blinking_led_on_time_ms = DEFAULT_BLINK_ON_MS;
static uint8_t g_input_inversion_mask = DEFAULT_INPUT_INVERSION_MASK; // Bit 0 untuk INPUT_PIN_0, dst. 1 = inverted.
static uint16_t g_ssv_variation_rpm = DEFAULT_SSV_VARIATION_RPM;
static uint32_t g_ssv_cycle_ms = DEFAULT_SSV_CYCLE_MS;
static uint16_t g_min_spindle_rpm = DEFAULT_MIN_SPINDLE_RPM;
static uint16_t g_max_spindle_rpm = DEFAULT_MAX_SPINDLE_RPM;

// Variabel global untuk status input CW dan CCW (diupdate oleh EXTI callback)
volatile bool g_user_btn_debounced_state = false;
volatile bool g_input_cw_debounced_state = false;
volatile bool g_input_ccw_debounced_state = false;
volatile uint8_t g_user_btn_event_latch = 0;
volatile uint8_t g_input_cw_event_latch = 0;
volatile uint8_t g_input_ccw_event_latch = 0;
volatile uint32_t g_last_interrupt_time_user_btn = 0;
volatile uint32_t g_last_interrupt_time_input_cw = 0;
volatile uint32_t g_last_interrupt_time_input_ccw = 0;
uint8_t current_direction = 0; // 0 = stop, 1 = CW, 2 = CCW

// Variabel baru untuk melacak status output
volatile char g_last_atc_command = '?'; // Inisialisasi ke 'tidak diketahui'
volatile char g_last_led_command = '?'; // Inisialisasi ke 'tidak diketahui'

// input pwm related variables
volatile uint32_t g_pwm_frequency = 0;
volatile float g_pwm_duty_cycle = 0.0f;
volatile uint32_t g_pwm_cycle_time_raw = 0;	 // Raw capture value for period
volatile uint32_t g_pwm_pulse_width_raw = 0; // Raw capture value for pulse
uint32_t actual_rpm_to_send = 0;

// display buffer untuk menampilkan data di USB CDC
// static char display_buffer[DISPLAY_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartInputScanTask(void *argument);
void StartOutputControlTask(void *argument);
void StartTimerOliTask(void *argument);
void StartModbusSpindleTask(void *argument);

/* USER CODE BEGIN PFP */
void InputScanTask_Entry(void *argument);
void OutputControlTask_Entry(void *argument);
static void SR_UpdateOutputs(void);
static void SR_Init(void);
static uint32_t Flash_Read_Config_Word(uint32_t address_offset);
// static HAL_StatusTypeDef Flash_Write_Config_Word(uint32_t address_offset,
//												 uint32_t data);
static void Load_All_Configs_From_Flash(void);
static HAL_StatusTypeDef Save_Input_Inversion_Mask_To_Flash(uint8_t mask);

static uint16_t calculate_crc16(const uint8_t *data, uint16_t length);
static void print_tx_data_hex(uint8_t *data, uint32_t panjang_data);
static void sendDataOverModbus(uint8_t *data, uint8_t ukuran);
static void Spindle_Modbus_CW_CCW(uint8_t arah, uint8_t verbose);
static void Spindle_Modbus_Stop_Hold(uint8_t verbose);
static void Spindle_Modbus_Stop_Disable(uint8_t verbose);
static void Spindle_Modbus_Set_Rpm(uint32_t rpm, uint8_t verbose);
static uint32_t Get_Spindle_RPM_From_PWM(void);

// Fungsi baru untuk menampilkan parameter dan pinout
// char get_port_letter(GPIO_TypeDef *port);
// uint8_t get_pin_number(uint16_t pin_mask);
static void Display_All_Parameters(void);
static void Display_Pinout_Info(void);
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
int _write(int file, char *ptr, int len)
{
	CDC_Transmit_FS((uint8_t *)ptr, len);
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
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	// untuk timer input capture (PWM)
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // Signal Input Channel (Main)
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);	// Secondary Channel

	Load_All_Configs_From_Flash(); // Muat konfigurasi blinker dari Flash
	SR_Init();					   // Inisialisasi shift register ke keadaan mati
	g_user_btn_debounced_state = (HAL_GPIO_ReadPin(USER_BTN_GPIO_Port,
												   USER_BTN_Pin) == GPIO_PIN_SET); // both rising_falling
	g_input_cw_debounced_state = (HAL_GPIO_ReadPin(INPUT_CW_GPIO_Port,
												   INPUT_CW_Pin) == GPIO_PIN_SET); // both rising_falling
	g_input_ccw_debounced_state = (HAL_GPIO_ReadPin(INPUT_CCW_GPIO_Port,
													INPUT_CCW_Pin) == GPIO_PIN_SET); // both rising_falling
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
	OutputCmdQueueHandle = osMessageQueueNew(16, sizeof(uint16_t),
											 &OutputCmdQueue_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of InputScanTask */
	InputScanTaskHandle = osThreadNew(StartInputScanTask, NULL,
									  &InputScanTask_attributes);

	/* creation of OutputCtrlTask */
	OutputCtrlTaskHandle = osThreadNew(StartOutputControlTask, NULL,
									   &OutputCtrlTask_attributes);

	/* creation of TimerOliTask */
	TimerOliTaskHandle = osThreadNew(StartTimerOliTask, NULL,
									 &TimerOliTask_attributes);

	/* creation of ModbusSpindle */
	ModbusSpindleHandle = osThreadNew(StartModbusSpindleTask, NULL,
									  &ModbusSpindle_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	if (InputScanTaskHandle == NULL)
	{
		DEBUG_PRINTF("FATAL Error: Gagal membuat InputScanTask!\r\n");
		Error_Handler();
	}

	if (OutputCtrlTaskHandle == NULL)
	{
		DEBUG_PRINTF("FATAL Error: Gagal membuat OutputControlTask!\r\n");
		Error_Handler();
	}

	if (TimerOliTaskHandle == NULL)
	{
		// Ini yang paling mungkin terjadi jika heap habis
		DEBUG_PRINTF(
			"FATAL Error: Gagal membuat BlinkingLedTask! (Cek configTOTAL_HEAP_SIZE)\r\n");
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
	while (1)
	{
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
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */
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
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 38400;
	huart1.Init.WordLength = UART_WORDLENGTH_9B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_EVEN;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */
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
	HAL_GPIO_WritePin(GPIOA, HC595_LOAD_Pin | MODBUS_SEL_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_BTN_Pin */
	GPIO_InitStruct.Pin = USER_BTN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PROXY_UMB_A_Pin */
	GPIO_InitStruct.Pin = PROXY_UMB_A_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(PROXY_UMB_A_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : HC595_LOAD_Pin MODBUS_SEL_Pin */
	GPIO_InitStruct.Pin = HC595_LOAD_Pin | MODBUS_SEL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : ORIENT_OK_Pin PROXY_TOOLS_Pin PROXY_UMB_B_Pin PROXY_CLAMP_A_Pin
	 PROXY_CLAMP_B_Pin SENSOR_OLI_Pin INPUT_CLAMP_Pin INPUT_UNCLAMP_Pin */
	GPIO_InitStruct.Pin = ORIENT_OK_Pin | PROXY_TOOLS_Pin | PROXY_UMB_B_Pin | PROXY_CLAMP_A_Pin | PROXY_CLAMP_B_Pin | SENSOR_OLI_Pin | INPUT_CLAMP_Pin | INPUT_UNCLAMP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : USER_LED_Pin */
	GPIO_InitStruct.Pin = USER_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : INPUT_CCW_Pin INPUT_CW_Pin */
	GPIO_InitStruct.Pin = INPUT_CCW_Pin | INPUT_CW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Callback untuk interrupt GPIO.
 * @param  GPIO_Pin: Pin yang menghasilkan interrupt.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//	uint32_t current_time = HAL_GetTick();

	if (GPIO_Pin == USER_BTN_Pin) // Ganti dengan nama makro pin tombol 1 Anda
	{
#if DEBOUNCE_TIME_MS != 0
		if ((current_time - g_last_interrupt_time_user_btn) > DEBOUNCE_TIME_MS)
		{
			// g_user_btn_debounced_state = !g_user_btn_debounced_state; // Contoh toggle
			g_user_btn_debounced_state = (HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_SET); // both rising_falling
			g_user_btn_event_latch = 1;																		   // Set flag event untuk Tombol 1
			g_last_interrupt_time_user_btn = current_time;
			// DEBUG_PRINTF("Tombol 1 event!\r\n"); // Debug
		}
#else
		g_user_btn_debounced_state = (HAL_GPIO_ReadPin(USER_BTN_GPIO_Port,
													   USER_BTN_Pin) == GPIO_PIN_SET); // both rising_falling
		g_user_btn_event_latch = 1;
#endif
	}
	else if (GPIO_Pin == INPUT_CW_Pin) // Ganti dengan nama makro pin tombol 2 Anda
	{
#if DEBOUNCE_TIME_MS != 0
		if ((current_time - g_last_interrupt_time_input_cw) > DEBOUNCE_TIME_MS)
		{
			g_input_cw_debounced_state = (HAL_GPIO_ReadPin(INPUT_CW_GPIO_Port, INPUT_CW_Pin) == GPIO_PIN_SET); // Contoh toggle
			g_input_cw_event_latch = 1;
			g_last_interrupt_time_input_cw = current_time;
		}
#else
		g_input_cw_debounced_state = (HAL_GPIO_ReadPin(INPUT_CW_GPIO_Port,
													   INPUT_CW_Pin) == GPIO_PIN_SET); // Contoh toggle
		g_input_cw_event_latch = 1;
#endif
	}
	else if (GPIO_Pin == INPUT_CCW_Pin) // Ganti dengan nama makro pin tombol 3 Anda
	{
#if DEBOUNCE_TIME_MS != 0
		if ((current_time - g_last_interrupt_time_input_ccw) > DEBOUNCE_TIME_MS)
		{
			g_input_ccw_debounced_state = (HAL_GPIO_ReadPin(INPUT_CCW_GPIO_Port, INPUT_CCW_Pin) == GPIO_PIN_SET); // Contoh toggle
			g_input_ccw_event_latch = 1;
			g_last_interrupt_time_input_ccw = current_time;
		}
#else
		g_input_ccw_debounced_state = (HAL_GPIO_ReadPin(INPUT_CCW_GPIO_Port,
														INPUT_CCW_Pin) == GPIO_PIN_SET); // Contoh toggle
		g_input_ccw_event_latch = 1;
#endif
	}
	// Anda bisa menambahkan else if untuk pin interrupt lainnya jika ada
}

/**
 * @brief  Membaca satu word (32-bit) dari alamat Flash.
 * @param  address: Alamat Flash yang akan dibaca.
 * @retval Nilai yang tersimpan di alamat tersebut.
 */
static uint32_t Flash_Read_Config_Word(uint32_t address_offset)
{
	return *(volatile uint32_t *)(CONFIG_FLASH_PAGE_ADDR + address_offset);
}

// static HAL_StatusTypeDef Flash_Write_Config_Word(uint32_t address_offset,
//												 uint32_t data)
//{
//	HAL_StatusTypeDef status;
//	// Tidak perlu erase page setiap kali menulis satu word jika page sudah di-erase sebelumnya
//	// atau jika kita tahu kita hanya mengubah bit dari 1 ke 0.
//	// Namun, untuk kesederhanaan dan keandalan, erase page jika ini adalah bagian dari update besar.
//	// Untuk kasus ini, kita asumsikan page sudah di-erase oleh fungsi yang lebih tinggi jika perlu.
//	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
//							   CONFIG_FLASH_PAGE_ADDR + address_offset, data);
//	return status;
// }

/**
 * @brief  Menulis konfigurasi delay blinker ke Flash.
 * Fungsi ini akan menghapus satu page Flash dan menulis kedua nilai delay.
 * @param  off_time_ms: Waktu LED mati dalam milidetik.
 * @param  on_time_ms: Waktu LED menyala dalam milidetik.
 * @retval HAL_StatusTypeDef: Status operasi Flash.
 */

// Fungsi ini akan menghapus page dan menulis semua konfigurasi yang relevan
static HAL_StatusTypeDef Flash_Write_All_Known_Configs(void)
{
	HAL_StatusTypeDef status;
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError = 0;

	status = HAL_FLASH_Unlock();
	if (status != HAL_OK)
	{
		DEBUG_PRINTF(
			"Error: Flash Unlock gagal saat menyimpan semua config! (%d)\r\n",
			status);
		return status;
	}

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = CONFIG_FLASH_PAGE_ADDR;
	EraseInitStruct.NbPages = 1;
	status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

	if (status == HAL_OK)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
							  CONFIG_FLASH_PAGE_ADDR + BLINK_OFF_TIME_OFFSET,
							  g_blinking_led_off_time_ms) != HAL_OK)
			goto flash_write_error;
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
							  CONFIG_FLASH_PAGE_ADDR + BLINK_ON_TIME_OFFSET,
							  g_blinking_led_on_time_ms) != HAL_OK)
			goto flash_write_error;
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
							  CONFIG_FLASH_PAGE_ADDR + INPUT_INVERSION_MASK_OFFSET,
							  (uint32_t)g_input_inversion_mask) != HAL_OK)
			goto flash_write_error;
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
							  CONFIG_FLASH_PAGE_ADDR + SSV_VARIATION_RPM_OFFSET,
							  (uint32_t)g_ssv_variation_rpm) != HAL_OK)
			goto flash_write_error;
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
							  CONFIG_FLASH_PAGE_ADDR + SSV_CYCLE_MS_OFFSET, g_ssv_cycle_ms) != HAL_OK)
			goto flash_write_error;
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
							  CONFIG_FLASH_PAGE_ADDR + MIN_SPINDLE_RPM_OFFSET,
							  (uint32_t)g_min_spindle_rpm) != HAL_OK)
			goto flash_write_error;
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
							  CONFIG_FLASH_PAGE_ADDR + MAX_SPINDLE_RPM_OFFSET,
							  (uint32_t)g_max_spindle_rpm) != HAL_OK)
			goto flash_write_error;
	}
	else
	{
		DEBUG_PRINTF(
			"Error: Gagal erase page untuk semua config (PageError: 0x%lX, Status: %d)!\r\n",
			PageError, status);
	}
	goto flash_write_end;

flash_write_error:
	status = HAL_ERROR;
flash_write_end:
	if (status != HAL_OK && PageError == 0xFFFFFFFF)
	{
		DEBUG_PRINTF("Error saat menulis salah satu config ke Flash! (%d)\r\n",
					 status);
	}
	HAL_FLASH_Lock();
	return status;
}

/**
 * @brief  Memuat konfigurasi delay blinker dari Flash ke variabel global.
 * Jika Flash kosong atau data tidak valid, gunakan nilai default dan tulis ke Flash.
 */

static void Load_All_Configs_From_Flash(void)
{
	uint32_t stored_val;
	uint8_t write_defaults_to_flash = 0;

	stored_val = Flash_Read_Config_Word(BLINK_OFF_TIME_OFFSET);
	if (stored_val == 0xFFFFFFFF || stored_val == 0)
	{
		g_blinking_led_off_time_ms = DEFAULT_BLINK_OFF_MS;
		write_defaults_to_flash = 1;
	}
	else
	{
		g_blinking_led_off_time_ms = stored_val;
	}

	stored_val = Flash_Read_Config_Word(BLINK_ON_TIME_OFFSET);
	if (stored_val == 0xFFFFFFFF || stored_val == 0)
	{
		g_blinking_led_on_time_ms = DEFAULT_BLINK_ON_MS;
		write_defaults_to_flash = 1;
	}
	else
	{
		g_blinking_led_on_time_ms = stored_val;
	}

	stored_val = Flash_Read_Config_Word(INPUT_INVERSION_MASK_OFFSET);
	if (stored_val == 0xFFFFFFFF)
	{
		g_input_inversion_mask = DEFAULT_INPUT_INVERSION_MASK;
		write_defaults_to_flash = 1;
	}
	else
	{
		g_input_inversion_mask = (uint8_t)(stored_val & 0xFF);
	}

	stored_val = Flash_Read_Config_Word(SSV_VARIATION_RPM_OFFSET);
	if (stored_val == 0xFFFFFFFF)
	{
		g_ssv_variation_rpm = DEFAULT_SSV_VARIATION_RPM;
		write_defaults_to_flash = 1;
	}
	else
	{
		g_ssv_variation_rpm = (uint16_t)(stored_val & 0xFFFF);
	}

	stored_val = Flash_Read_Config_Word(SSV_CYCLE_MS_OFFSET);
	if (stored_val == 0xFFFFFFFF || stored_val == 0)
	{
		g_ssv_cycle_ms = DEFAULT_SSV_CYCLE_MS;
		write_defaults_to_flash = 1;
	}
	else
	{
		g_ssv_cycle_ms = stored_val;
	}

	stored_val = Flash_Read_Config_Word(MIN_SPINDLE_RPM_OFFSET);
	if (stored_val == 0xFFFFFFFF)
	{
		g_min_spindle_rpm = DEFAULT_MIN_SPINDLE_RPM;
		write_defaults_to_flash = 1;
	}
	else
	{
		g_min_spindle_rpm = (uint16_t)(stored_val & 0xFFFF);
	}

	stored_val = Flash_Read_Config_Word(MAX_SPINDLE_RPM_OFFSET);
	if (stored_val == 0xFFFFFFFF || stored_val < g_min_spindle_rpm)
	{ // Max RPM harus > Min RPM
		g_max_spindle_rpm = DEFAULT_MAX_SPINDLE_RPM;
		if (g_max_spindle_rpm < g_min_spindle_rpm)
			g_max_spindle_rpm = g_min_spindle_rpm + 1000; // Pastikan Max > Min
		write_defaults_to_flash = 1;
	}
	else
	{
		g_max_spindle_rpm = (uint16_t)(stored_val & 0xFFFF);
	}

	if (write_defaults_to_flash)
	{
		DEBUG_PRINTF(
			"Info: Satu atau lebih konfigurasi tidak valid/default, menulis ulang semua ke Flash...\r\n");
		if (Flash_Write_All_Known_Configs() == HAL_OK)
		{
			DEBUG_PRINTF("Info: Konfigurasi default berhasil ditulis ke Flash.\r\n");
		}
		else
		{
			DEBUG_PRINTF(
				"Error: Gagal menulis konfigurasi default ke Flash saat startup.\r\n");
		}
	}
}

static HAL_StatusTypeDef Save_Input_Inversion_Mask_To_Flash(uint8_t mask)
{
	g_input_inversion_mask = mask; // Update RAM copy
	// Untuk menyimpan hanya satu item, kita perlu membaca yang lain, erase, lalu tulis semua
	// Atau, jika FEE (Flash Emulated EEPROM) digunakan, itu akan lebih mudah.
	// Untuk sekarang, kita akan menulis ulang semua config yang diketahui.
	printf("Info: Menyimpan Input Inversion Mask (0x%02X) ke Flash...\r\n",
				 g_input_inversion_mask);
	return Flash_Write_All_Known_Configs();
}

static HAL_StatusTypeDef Save_Blinker_Config_To_Flash(void)
{
	printf("Info: Menyimpan Blinker Config (ON:%lu, OFF:%lu) ke Flash...\r\n",
				 g_blinking_led_on_time_ms, g_blinking_led_off_time_ms);
	return Flash_Write_All_Known_Configs();
}

static HAL_StatusTypeDef Save_SSV_Config_To_Flash(void)
{
	printf(
		"Info: Menyimpan SSV Config (Var:%u RPM, Cycle:%lu ms) ke Flash...\r\n",
		g_ssv_variation_rpm, g_ssv_cycle_ms);
	return Flash_Write_All_Known_Configs();
}
static HAL_StatusTypeDef Save_RPM_Range_Config_To_Flash(void)
{
	printf("Info: Menyimpan RPM Range (Min:%u, Max:%u) ke Flash...\r\n",
				 g_min_spindle_rpm, g_max_spindle_rpm);
	return Flash_Write_All_Known_Configs();
}

/**
 * @brief  Inisialisasi output shift register ke keadaan mati.
 */
static void SR_Init(void)
{
	current_sr_output_state = 0x0000; // Semua output mati
	SR_UpdateOutputs();
}

/**
 * @brief  Mengirim state output saat ini ke shift register 74HC595.
 */
static void SR_UpdateOutputs(void)
{
	uint8_t bytes_to_send[2];

	// Data untuk 74HC595 kedua (output 8-12, sisanya tidak terpakai jika hanya 13 output)
	// Bit 8 dari current_sr_output_state -> Q0 dari SR kedua
	// Bit 12 dari current_sr_output_state -> Q4 dari SR kedua
	bytes_to_send[0] = (uint8_t)((current_sr_output_state >> 8) & 0xFF);

	// Data untuk 74HC595 pertama (output 0-7)
	// Bit 0 dari current_sr_output_state -> Q0 dari SR pertama
	// Bit 7 dari current_sr_output_state -> Q7 dari SR pertama
	bytes_to_send[1] = (uint8_t)(current_sr_output_state & 0xFF);

	HAL_GPIO_WritePin(HC595_LOAD_GPIO_Port, HC595_LOAD_Pin, GPIO_PIN_RESET); // Latch LOW

	// Kirim data: byte untuk SR terjauh dulu, lalu byte untuk SR terdekat
	if (HAL_SPI_Transmit(&hspi1, &bytes_to_send[0], 1, 100) != HAL_OK)
	{ // Data untuk SR2
		DEBUG_PRINTF("SPI Transmit Error SR2\r\n");
	}
	if (HAL_SPI_Transmit(&hspi1, &bytes_to_send[1], 1, 100) != HAL_OK)
	{ // Data untuk SR1
		DEBUG_PRINTF("SPI Transmit Error SR1\r\n");
	}

	HAL_GPIO_WritePin(HC595_LOAD_GPIO_Port, HC595_LOAD_Pin, GPIO_PIN_SET);	 // Latch HIGH (data masuk ke output)
	HAL_GPIO_WritePin(HC595_LOAD_GPIO_Port, HC595_LOAD_Pin, GPIO_PIN_RESET); // Latch LOW kembali (siap untuk data berikutnya)
}

static void Display_All_Parameters(void)
{
	//	baca_nilai_pada_flash();
	char buffer[512];
	int len =
		snprintf(buffer, sizeof(buffer),
				 "firm_version = %s\n$1 = %lu g_blinking_led_off_time_ms\n$2 = %lu g_blinking_led_on_time_ms\n$3 = %u g_input_inversion_mask\n$4 = %u g_ssv_variation_rpm\n$5 = %lu g_ssv_cycle_ms\n$6 = %u g_min_spindle_rpm\n$7 = %u g_max_spindle_rpm\n",
				 FIRMWARE_VERSION, g_blinking_led_off_time_ms, // 1
				 g_blinking_led_on_time_ms,					   // 2
				 g_input_inversion_mask,					   // 3
				 g_ssv_variation_rpm,						   // 4
				 g_ssv_cycle_ms,							   // 5
				 g_min_spindle_rpm,							   // 6
				 g_max_spindle_rpm							   // 7
		);
	CDC_Transmit_FS((uint8_t *)buffer, len);
	// tampilkan_data_flash = 0;
}

static void Display_Pinout_Info(void)
{
	char buffer[512];
	int len =
		snprintf(buffer, sizeof(buffer),
				 "%s : %s%d\r\n%s : %s%d\r\n%s : %s%d\r\n%s : %s%d\r\n%s : %s%d\r\n%s : %s%d\r\n%s : %s%d\r\n%s : %s%d\r\n%s : %s%d\r\n",
				 input_pin_names[0], input_port_letters[0],
				 input_port_number[0], input_pin_names[1],
				 input_port_letters[1], input_port_number[1],
				 input_pin_names[2], input_port_letters[2],
				 input_port_number[2], input_pin_names[3],
				 input_port_letters[3], input_port_number[3],
				 input_pin_names[4], input_port_letters[4],
				 input_port_number[4], input_pin_names[5],
				 input_port_letters[5], input_port_number[5],
				 input_pin_names[6], input_port_letters[6],
				 input_port_number[6], input_pin_names[7],
				 input_port_letters[7], input_port_number[7],
				 input_pin_names[8], input_port_letters[8],
				 input_port_number[8]);
	CDC_Transmit_FS((uint8_t *)buffer, len);
}

void USB_CDC_RxHandler(uint8_t *Buf, uint32_t Len)
{
	if (Len > 0 && Len < sizeof(usb_rx_buffer))
	{
		memcpy(usb_rx_buffer, Buf, Len);
		usb_rx_buffer[Len] = '\0';

		if (usb_rx_buffer[0] == 'E' && Len == 1)
		{
			Spindle_Modbus_Stop_Disable(MODBUS_VERBOSE);
			printf("Spindle STOP\r\n");
		}
		else if (Len == 1 && usb_rx_buffer[0] >= 'A' && usb_rx_buffer[0] <= 'Z')
		{
			OutputCommand_t cmd_msg;
			cmd_msg.command = usb_rx_buffer[0];
			if (osMessageQueuePut(OutputCmdQueueHandle, &cmd_msg, 0U, 0U) != osOK)
			{
				printf("Error: Gagal mengirim perintah output ke queue.\r\n");
			}
		}
		else if (strncmp(usb_rx_buffer, "$$", 2) == 0 && Len == 2) // Perintah $$
		{
			Display_All_Parameters();
		}
		else if (strncmp(usb_rx_buffer, "$P", 2) == 0 && Len == 2) // Perintah $P
		{
			Display_Pinout_Info();
		}
		else if (strncmp(usb_rx_buffer, "$1=", 3) == 0) // Waktu OFF LED Berkedip (detik)
		{
			char *value_str = usb_rx_buffer + 3;
			long val_seconds = atol(value_str);
			if (val_seconds > 0 && val_seconds <= 3600)
			{
				uint32_t new_off_time_ms = (uint32_t)val_seconds * 1000;
				if (new_off_time_ms != g_blinking_led_off_time_ms)
				{
					g_blinking_led_off_time_ms = new_off_time_ms;
					Save_Blinker_Config_To_Flash();
				}
				else
				{
					printf("Info: Waktu OFF LED tidak berubah ($1=%ld).\r\n",
						   val_seconds);
				}
			}
			else
			{
				printf(
					"Error: Nilai $1 tidak valid (%s). Harus 1-3600 detik.\r\n",
					value_str);
			}
		}
		else if (strncmp(usb_rx_buffer, "$2=", 3) == 0) // Waktu ON LED Berkedip (detik)
		{
			char *value_str = usb_rx_buffer + 3;
			long val_seconds = atol(value_str);
			if (val_seconds > 0 && val_seconds <= 3600)
			{
				uint32_t new_on_time_ms = (uint32_t)val_seconds * 1000;
				if (new_on_time_ms != g_blinking_led_on_time_ms)
				{
					g_blinking_led_on_time_ms = new_on_time_ms;
					Save_Blinker_Config_To_Flash();
				}
				else
				{
					printf("Info: Waktu ON LED tidak berubah ($2=%ld).\r\n",
						   val_seconds);
				}
			}
			else
			{
				printf(
					"Error: Nilai $2 tidak valid (%s). Harus 1-3600 detik.\r\n",
					value_str);
			}
		}
		else if (strncmp(usb_rx_buffer, "$3=", 3) == 0) // Input Inversion Mask
		{
			char *value_str = usb_rx_buffer + 3;
			long val_mask = atol(value_str); // Bisa juga strtol dengan basis 10 atau 16
			if (val_mask >= 0 && val_mask <= 255)
			{ // 8 bit mask
				uint8_t new_mask = (uint8_t)val_mask;
				if (new_mask != g_input_inversion_mask)
				{
					Save_Input_Inversion_Mask_To_Flash(new_mask);
				}
				else
				{
					printf(
						"Info: Input Inversion Mask tidak berubah ($3=%ld).\r\n",
						val_mask);
				}
			}
			else
			{
				printf("Error: Nilai $3 tidak valid (%s). Harus 0-255.\r\n",
					   value_str);
			}
		}
		else if (strncmp(usb_rx_buffer, "$4=", 3) == 0) // SSV Variation RPM
		{
			char *value_str = usb_rx_buffer + 3;
			long val_rpm = atol(value_str);
			if (val_rpm >= 0 && val_rpm <= 5000)
			{ // Batasi variasi RPM
				uint16_t new_var_rpm = (uint16_t)val_rpm;
				if (new_var_rpm != g_ssv_variation_rpm)
				{
					g_ssv_variation_rpm = new_var_rpm;
					Save_SSV_Config_To_Flash();
				}
			}
			else
			{
				printf("Error: Nilai $4 (SSV Var) tidak valid (0-5000).\r\n");
			}
		}
		else if (strncmp(usb_rx_buffer, "$5=", 3) == 0) // SSV Cycle (ms)
		{
			char *value_str = usb_rx_buffer + 3;
			long val_ms = atol(value_str);
			// Cycle 0 berarti SSV nonaktif, atau minimal 100ms
			if (val_ms == 0 || (val_ms >= 100 && val_ms <= 60000))
			{
				uint32_t new_cycle_ms = (uint32_t)val_ms;
				if (new_cycle_ms != g_ssv_cycle_ms)
				{
					g_ssv_cycle_ms = new_cycle_ms;
					Save_SSV_Config_To_Flash();
				}
			}
			else
			{
				printf(
					"Error: Nilai $5 (SSV Cycle) tidak valid (0 atau 100-60000 ms).\r\n");
			}
		}
		else if (strncmp(usb_rx_buffer, "$6=", 3) == 0) // Min Spindle RPM
		{
			char *value_str = usb_rx_buffer + 3;
			long val_rpm = atol(value_str);
			if (val_rpm >= 0 && val_rpm < g_max_spindle_rpm && val_rpm <= 30000)
			{ // Min RPM tidak boleh > Max RPM
				uint16_t new_min_rpm = (uint16_t)val_rpm;
				if (new_min_rpm != g_min_spindle_rpm)
				{
					g_min_spindle_rpm = new_min_rpm;
					Save_RPM_Range_Config_To_Flash();
				}
			}
			else
			{
				printf(
					"Error: Nilai $6 (Min RPM) tidak valid atau > Max RPM.\r\n");
			}
		}
		else if (strncmp(usb_rx_buffer, "$7=", 3) == 0) // Max Spindle RPM
		{
			char *value_str = usb_rx_buffer + 3;
			long val_rpm = atol(value_str);
			if (val_rpm > g_min_spindle_rpm && val_rpm <= 30000)
			{ // Max RPM harus > Min RPM
				uint16_t new_max_rpm = (uint16_t)val_rpm;
				if (new_max_rpm != g_max_spindle_rpm)
				{
					g_max_spindle_rpm = new_max_rpm;
					Save_RPM_Range_Config_To_Flash();
				}
			}
			else
			{
				printf(
					"Error: Nilai $7 (Max RPM) tidak valid atau < Min RPM.\r\n");
			}
		}
		// Perintah manual spindle via USB CDC
		else if ((usb_rx_buffer[0] == 'M' || usb_rx_buffer[0] == 'N') && Len >= 2)
		{
			// Format: M200 atau N200
			char dir = usb_rx_buffer[0];
			int rpm = atoi(&usb_rx_buffer[1]);
			if (rpm > 0 && rpm <= g_max_spindle_rpm)
			{
				if (dir == 'M')
				{
					Spindle_Modbus_CW_CCW(1, MODBUS_VERBOSE); // CW
					Spindle_Modbus_Set_Rpm(rpm, MODBUS_VERBOSE);
					printf("Spindle CW %d rpm\r\n", rpm);
				}
				else if (dir == 'N')
				{
					Spindle_Modbus_CW_CCW(0, MODBUS_VERBOSE); // CCW
					Spindle_Modbus_Set_Rpm(rpm, MODBUS_VERBOSE);
					printf("Spindle CCW %d rpm\r\n", rpm);
				}
			}
			else
			{
				printf("Error: RPM tidak valid (1-5000)\r\n");
			}
		}
		else
		{
			printf("Perintah serial tidak dikenal: %s\r\n", usb_rx_buffer);
		}
	}
}

// --- Implementasi Fungsi Modbus ---
static uint16_t calculate_crc16(const uint8_t *data, uint16_t length)
{
	uint16_t crc = 0xFFFF;
	for (uint16_t i = 0; i < length; i++)
	{
		crc ^= data[i];
		for (uint8_t j = 0; j < 8; j++)
		{
			if (crc & 0x0001)
			{
				crc = (crc >> 1) ^ 0xA001;
			}
			else
			{
				crc = crc >> 1;
			}
		}
	}
	return crc;
}

static void print_tx_data_hex(uint8_t *data, uint32_t panjang_data)
{
	if (!MODBUS_VERBOSE)
		return;
	DEBUG_PRINTF("Modbus TX: ");
	for (uint32_t i = 0; i < panjang_data; i++)
	{
		DEBUG_PRINTF("%02X ", data[i]);
	}
	DEBUG_PRINTF("\r\n");
}

static void sendDataOverModbus(uint8_t *data, uint8_t ukuran)
{
	HAL_GPIO_WritePin(MODBUS_SEL_GPIO_Port, MODBUS_SEL_Pin, GPIO_PIN_SET); // Enable Transmit
	osDelay(1);															   // Sedikit delay untuk memastikan DE aktif sebelum data dikirim
	HAL_UART_Transmit(&MODBUS_UART_HANDLE, data, ukuran, 100);			   // Timeout 100ms
	// Tunggu sampai transmisi selesai (penting untuk RS485)
	// Cara paling sederhana adalah dengan delay yang cukup, atau cek flag UART TX Complete
	// Untuk HAL_UART_Transmit blocking, delay setelahnya mungkin cukup.
	osDelay(5);																 // Beri waktu untuk byte terakhir terkirim sepenuhnya
	HAL_GPIO_WritePin(MODBUS_SEL_GPIO_Port, MODBUS_SEL_Pin, GPIO_PIN_RESET); // Disable Transmit (Enable Receive)
}

static void Spindle_Modbus_CW_CCW(uint8_t arah, uint8_t verbose)
{
	uint8_t TxData[5];
	TxData[0] = MODBUS_SLAVE_ID;
	TxData[1] = 0x44; // Fungsi untuk kontrol arah/stop
	TxData[2] = (arah == 1) ? 0x01 /* CW */ : 0x02 /* CCW */;
	uint16_t crc = calculate_crc16(TxData, 3);
	TxData[3] = crc & 0xFF;
	TxData[4] = (crc >> 8) & 0xFF;
	if (verbose)
		print_tx_data_hex(TxData, 5);
	sendDataOverModbus(TxData, 5);
	osDelay(20); // Delay antar perintah Modbus
}

static void Spindle_Modbus_Stop_Hold(uint8_t verbose)
{
	uint8_t TxData[5];
	TxData[0] = MODBUS_SLAVE_ID;
	TxData[1] = 0x44;
	TxData[2] = 0x03; // Perintah Stop/Hold
	uint16_t crc = calculate_crc16(TxData, 3);
	TxData[3] = crc & 0xFF;
	TxData[4] = (crc >> 8) & 0xFF;
	if (verbose)
		print_tx_data_hex(TxData, 5);
	sendDataOverModbus(TxData, 5);
	osDelay(20);
}

static void Spindle_Modbus_Stop_Disable(uint8_t verbose)
{					   // Menggantikan SON_Motor_Off
	uint8_t TxData[5]; // Disesuaikan dengan fungsi 0x44 jika ini adalah "disable"
	TxData[0] = MODBUS_SLAVE_ID;
	TxData[1] = 0x44; // Fungsi untuk kontrol arah/stop
	TxData[2] = 0x00; // Perintah Disable (atau Free Stop)
	uint16_t crc = calculate_crc16(TxData, 3);
	TxData[3] = crc & 0xFF;
	TxData[4] = (crc >> 8) & 0xFF;
	if (verbose)
		print_tx_data_hex(TxData, 5);
	sendDataOverModbus(TxData, 5);
	osDelay(20);
}

static void Spindle_Modbus_Set_Rpm(uint32_t rpm, uint8_t verbose)
{
	uint8_t TxData[8]; // Ukuran buffer untuk Set RPM
	TxData[0] = MODBUS_SLAVE_ID;
	TxData[1] = 0x06;			   // Write Single Register
	TxData[2] = 0x00;			   // Alamat Register RPM (High Byte) - SESUAIKAN DENGAN MANUAL SPINDLE
	TxData[3] = 0x4C;			   // Alamat Register RPM (Low Byte) - SESUAIKAN (misal, P-076)
	TxData[4] = (rpm >> 8) & 0xFF; // Nilai RPM (High Byte)
	TxData[5] = rpm & 0xFF;		   // Nilai RPM (Low Byte)
	uint16_t crc = calculate_crc16(TxData, 6);
	TxData[6] = crc & 0xFF;
	TxData[7] = (crc >> 8) & 0xFF;
	if (verbose)
		print_tx_data_hex(TxData, 8);
	sendDataOverModbus(TxData, 8);
	osDelay(20);
}

// Placeholder untuk fungsi pembacaan PWM
static uint32_t Get_Spindle_RPM_From_PWM(void)
{
	float duty = g_pwm_duty_cycle; // 0.0f - 100.0f

	// Pastikan duty cycle berada dalam rentang 0-100
	if (duty < 0.0f)
		duty = 0.0f;
	if (duty > 100.0f)
		duty = 100.0f;

	// Linear scaling dari duty cycle (0-100%) ke RPM (min_rpm - max_rpm)
	if (g_max_spindle_rpm <= g_min_spindle_rpm)
	{
		return g_min_spindle_rpm; // Hindari pembagian dengan nol atau rentang negatif
	}

	uint32_t rpm_range = g_max_spindle_rpm - g_min_spindle_rpm;
	uint32_t calculated_rpm = g_min_spindle_rpm + (uint32_t)((duty / 100.0f) * rpm_range);

	return calculated_rpm;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // Interrupt dari Channel 1 (Period)
		{
			g_pwm_cycle_time_raw = HAL_TIM_ReadCapturedValue(htim,
															 TIM_CHANNEL_1);
			g_pwm_pulse_width_raw = HAL_TIM_ReadCapturedValue(htim,
															  TIM_CHANNEL_2); // Baca pulse width

			if (g_pwm_cycle_time_raw != 0)
			{
				g_pwm_duty_cycle = (g_pwm_pulse_width_raw * 100.0f) / g_pwm_cycle_time_raw;
				g_pwm_frequency = F_CLK_TIM2 / g_pwm_cycle_time_raw; // F_CLK_TIM2 harus benar
			}
			else
			{
				g_pwm_duty_cycle = 0.0f;
				g_pwm_frequency = 0;
			}
			// Tidak perlu re-start IT di sini, HAL akan menangani re-arming interrupt
			// selama TIM_CHANNEL_1 terus menerima edge yang valid.
		}
		// Tidak ada interrupt yang diaktifkan untuk TIM_CHANNEL_2 secara langsung,
		// nilainya dibaca ketika CH1 interrupt terjadi.
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
	//  MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 5 */
	// Buffer untuk membangun string output yang akan dikirim
	char final_output_buffer[128];
	// Array untuk menyimpan snapshot dari status input yang aktif
	uint8_t active_input_states[NUM_INPUTS] = {0};

	uint8_t last_linked_input_active_state = 0xFF; // Inisialisasi ke nilai yang tidak mungkin

	// dapatkan data dari queue
	// OutputCommand_t received_cmd;
	// osStatus_t status;

	DEBUG_PRINTF("InputScanTask dimulai dengan format baru.\r\n");

	/* Infinite loop */
	for (;;)
	{
		// baca queeue untuk mendapatkan perintah dari USB CDC
		// status = osMessageQueueGet(OutputCmdQueueHandle, &received_cmd, NULL, osWaitForever);

		// ======================================================================
		// Langkah 1: Baca dan simpan status semua 9 input terlebih dahulu
		// ======================================================================
		uint8_t current_linked_input_active_state = 0;
		for (int i = 0; i < NUM_INPUTS; i++)
		{
			if (input_ports[i] != NULL)
			{
				GPIO_PinState pin_raw_state = HAL_GPIO_ReadPin(input_ports[i],
															   input_pins[i]);
				uint8_t is_inverted = (g_input_inversion_mask >> i) & 0x01;
				uint8_t input_active = 0;

				if (is_inverted)
				{ // Logika terbalik: aktif jika HIGH
					input_active = (pin_raw_state == GPIO_PIN_SET);
				}
				else
				{ // Logika normal: aktif jika LOW (default)
					input_active = (pin_raw_state == GPIO_PIN_RESET);
				}

				// Simpan status aktif (1) atau tidak aktif (0)
				active_input_states[i] = input_active;

				// Cek status untuk input yang terhubung ke output (linked)
				if (i == PIN_MASK_SENSOR_OLI)
				{
					current_linked_input_active_state = input_active;
				}
			}
		}

		// ======================================================================
		// Langkah 2: Update output yang terhubung
		// sensor oli dengan pin buzzer
		// ======================================================================
		if (current_linked_input_active_state != last_linked_input_active_state)
		{
			// Hanya update output jika ada perubahan status input yang terhubung
			if (current_linked_input_active_state)
				current_sr_output_state |= (1 << PIN_OUTPUT_BUZZER);
			else
				current_sr_output_state &= ~(1 << PIN_OUTPUT_BUZZER);

			SR_UpdateOutputs();
			last_linked_input_active_state = current_linked_input_active_state;
		}

		// ======================================================================
		// Langkah 3: Bangun string output sesuai format "O:...|T:...|P:..."
		// ======================================================================
		int offset = 0;

		// Grup 'O' (Input 1, 5, 6 -> Indeks 0, 4, 5)
		offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "O:");
		if (current_direction != 0) // timer oli aktif (m3 atau m4)
		{
			offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "M");
		}
		if (current_sr_output_state & (1 << PIN_OUTPUT_OIL_PUMP)) // pompa oli aktif
		{
			offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "H");
		}
		if (active_input_states[PIN_MASK_SENSOR_OLI])
		{
			offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "E");
		}

		// Grup 'T' (Input pin proxy atc related, kecuali tools)
		offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "|T:");
		if (active_input_states[PIN_MASK_ORIENT_OK])
		{
			offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "O");
		}
		if (active_input_states[PIN_MASK_PROXY_UMB_A])
		{
			offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "F");
		}
		if (active_input_states[PIN_MASK_INPUT_UNCLAMP])
		{
			offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "U");
		}
		if (active_input_states[PIN_MASK_INPUT_CLAMP])
		{
			offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "L");
		}
		if (active_input_states[PIN_MASK_PROXY_UMB_B])
		{
			offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "B");
		}

		// Grup 'P' (Input proxy tools)
		offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "|P:");
		if (active_input_states[PIN_MASK_PROXY_TOOLS])
		{
			offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "1");
		}
		else
		{
			offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "0");
		}

		// Grup 'M' (ATC movement status)
		offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "|M:");
		offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "%c", g_last_atc_command);

		// Grup 'L' (LED color status)
		offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "|L:");
		offset += snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "%c", g_last_led_command);

		// Finalisasi string dengan baris baru
		snprintf(final_output_buffer + offset, sizeof(final_output_buffer) - offset, "\r\n");

		// ======================================================================
		// Langkah 4: Kirim seluruh string yang sudah diformat dalam satu kali panggilan
		// ======================================================================
		printf("%s", final_output_buffer);

		// Tunggu sebelum memulai siklus pemindaian berikutnya
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
	DEBUG_PRINTF("OutputControlTask dimulai.\r\n");

	/* Infinite loop */
	for (;;)
	{
		status = osMessageQueueGet(OutputCmdQueueHandle, &received_cmd, NULL,
								   osWaitForever);

		if (status == osOK)
		{
			if (strchr("ABC", received_cmd.command))
			{
				g_last_atc_command = received_cmd.command;
			}
			// Cek jika perintah untuk LED dan update status
			else if (strchr("RDXZ", received_cmd.command))
			{
				g_last_led_command = received_cmd.command;
			}

			switch (received_cmd.command)
			{
			case 'H': // pin orientasi aktif
				current_sr_output_state |= (1 << PIN_OUTPUT_ORIENT);
				SR_UpdateOutputs();
				break;
			case 'J': // pin orientasi nonaktif
				current_sr_output_state &= ~(1 << PIN_OUTPUT_ORIENT);
				SR_UpdateOutputs();
				break;
			case 'P': // pin output umb aktif
				current_sr_output_state |= (1 << PIN_OUTPUT_UMB);
				SR_UpdateOutputs();
				break;
			case 'K': // pin output umb nonaktif
				current_sr_output_state &= ~(1 << PIN_OUTPUT_UMB);
				SR_UpdateOutputs();
				break;
			case 'U': // pin output clamp aktif
				current_sr_output_state |= (1 << PIN_OUTPUT_CLAMP);
				SR_UpdateOutputs();
				break;
			case 'L': // pin output clamp nonaktif
				current_sr_output_state &= ~(1 << PIN_OUTPUT_CLAMP);
				SR_UpdateOutputs();
				break;
			case 'C': // pergerakan atc nonaktif
				current_sr_output_state &= ~(1 << PIN_OUTPUT_ATC_CW);
				current_sr_output_state &= ~(1 << PIN_OUTPUT_ATC_CCW);
				SR_UpdateOutputs();
				break;
			case 'A': // pergerakan atc CW
				current_sr_output_state |= (1 << PIN_OUTPUT_ATC_CW);
				current_sr_output_state &= ~(1 << PIN_OUTPUT_ATC_CCW);
				SR_UpdateOutputs();
				break;
			case 'B': // pergerakan atc CCW
				current_sr_output_state &= ~(1 << PIN_OUTPUT_ATC_CW);
				current_sr_output_state |= (1 << PIN_OUTPUT_ATC_CCW);
				SR_UpdateOutputs();
				break;
			case 'R': // led warna hijau aktif
				current_sr_output_state |= (1 << PIN_OUTPUT_LED_RED);
				current_sr_output_state &= ~(1 << PIN_OUTPUT_LED_GREEN);
				SR_UpdateOutputs();
				break;
			case 'D': // led warna kuning aktif
				current_sr_output_state |= (1 << PIN_OUTPUT_LED_RED);
				current_sr_output_state |= (1 << PIN_OUTPUT_LED_GREEN);
				SR_UpdateOutputs();
				break;
			case 'X': // led warna merah aktif
				current_sr_output_state &= ~(1 << PIN_OUTPUT_LED_RED);
				current_sr_output_state |= (1 << PIN_OUTPUT_LED_GREEN);
				SR_UpdateOutputs();
				break;
			case 'Z': // led warna mati
				current_sr_output_state &= ~(1 << PIN_OUTPUT_LED_RED);
				current_sr_output_state &= ~(1 << PIN_OUTPUT_LED_GREEN);
				SR_UpdateOutputs();
				break;
			default:
				printf("Perintah output tidak dikenal: %c\r\n",
							 received_cmd.command);
			}
		}
		else if (status == osErrorTimeout)
		{
			// Tidak ada perintah baru, lanjutkan loop
		}
		else
		{
			printf("Error: Gagal menerima perintah output dari queue.\r\n");
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
	DEBUG_PRINTF("BlinkingLedTask dimulai untuk Output SR %d.\r\n",
				 BLINKING_LED_SR_OUTPUT_INDEX);
	/* Infinite loop */
	for (;;)
	{
		current_sr_output_state |= (1 << BLINKING_LED_SR_OUTPUT_INDEX);
		SR_UpdateOutputs();
		osDelay(g_blinking_led_on_time_ms > 0 ? g_blinking_led_on_time_ms : 1); // Min delay 1ms

		current_sr_output_state &= ~(1 << BLINKING_LED_SR_OUTPUT_INDEX);
		SR_UpdateOutputs();
		osDelay(
			g_blinking_led_off_time_ms > 0 ? g_blinking_led_off_time_ms : 1); // Min delay 1ms
	}
	/* USER CODE END StartTimerOliTask */
}

/* USER CODE BEGIN Header_StartModbusSpindleTask */
/**
 * @brief Function implementing the ModbusSpindle thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartModbusSpindleTask */
void StartModbusSpindleTask(void *argument)
{
	/* USER CODE BEGIN StartModbusSpindleTask */
	DEBUG_PRINTF("ModbusSpindleControlTask dimulai.\r\n");
	uint32_t base_target_rpm = 0;
	// uint8_t current_direction = 0; // 0 = stop, 1 = CW, 2 = CCW
	uint8_t last_direction = 0xFF;
	uint32_t last_sent_actual_rpm = 0xFFFFFFFF;
	uint32_t ssv_cycle_start_time_ms = 0;

	Spindle_Modbus_Stop_Disable(MODBUS_VERBOSE);
	osDelay(100);
	/* Infinite loop */
	for (;;)
	{
		uint8_t cw_active = g_input_cw_debounced_state;
		uint8_t ccw_active = g_input_ccw_debounced_state;

		if (cw_active && !ccw_active)
			current_direction = 1;
		else if (!cw_active && ccw_active)
			current_direction = 2;
		else
			current_direction = 0;

		base_target_rpm = Get_Spindle_RPM_From_PWM();
		actual_rpm_to_send = base_target_rpm; // Default ke RPM dasar

		// Logika SSV
		if (current_direction != 0 && g_ssv_variation_rpm > 0 && g_ssv_cycle_ms > 0)
		{
			uint32_t current_time_ms = osKernelGetTickCount();
			if (ssv_cycle_start_time_ms == 0)
			{ // Inisialisasi start time untuk siklus SSV
				ssv_cycle_start_time_ms = current_time_ms;
			}

			uint32_t time_in_full_cycle_ms = (current_time_ms - ssv_cycle_start_time_ms) % g_ssv_cycle_ms;
			float progress_ratio;
			int32_t ssv_offset;

			uint32_t half_cycle_ms = g_ssv_cycle_ms / 2;

			if (time_in_full_cycle_ms < half_cycle_ms)
			{ // Fase naik (Base - Var -> Base + Var)
				progress_ratio = (float)time_in_full_cycle_ms / half_cycle_ms;
				ssv_offset = (int32_t)(-g_ssv_variation_rpm + progress_ratio * (2.0f * g_ssv_variation_rpm));
			}
			else
			{ // Fase turun (Base + Var -> Base - Var)
				uint32_t time_in_second_half = time_in_full_cycle_ms - half_cycle_ms;
				progress_ratio = (float)time_in_second_half / half_cycle_ms;
				ssv_offset = (int32_t)(g_ssv_variation_rpm - progress_ratio * (2.0f * g_ssv_variation_rpm));
			}

			actual_rpm_to_send = base_target_rpm + ssv_offset;
			if ((int32_t)actual_rpm_to_send < 0)
				actual_rpm_to_send = 0; // Pastikan RPM tidak negatif
		}
		else
		{
			ssv_cycle_start_time_ms = 0; // Reset SSV start time jika spindle berhenti atau SSV dinonaktifkan
		}

		// Kirim perintah arah jika berubah
		if (current_direction != last_direction)
		{
			if (current_direction == 1)
			{
				Spindle_Modbus_CW_CCW(1, MODBUS_VERBOSE);
			}
			else if (current_direction == 2)
			{
				Spindle_Modbus_CW_CCW(0, MODBUS_VERBOSE);
			}
			else
			{
				Spindle_Modbus_Stop_Hold(MODBUS_VERBOSE);
			}
			last_direction = current_direction;
			// Setelah arah berubah, selalu kirim RPM (meskipun mungkin sama dengan sebelumnya,
			// karena spindle mungkin perlu perintah RPM setelah start/arah baru)
			// Kita akan mengandalkan pengecekan di bawah untuk mengirim RPM jika berbeda atau arah baru saja berubah.
			// last_sent_actual_rpm = 0xFFFFFFFF; // Reset agar RPM dikirim lagi
		}

		// Kirim RPM jika arah tidak STOP dan RPM aktual (termasuk SSV) berubah
		if (current_direction != 0)
		{
			if (actual_rpm_to_send != last_sent_actual_rpm)
			{
				Spindle_Modbus_Set_Rpm(actual_rpm_to_send, MODBUS_VERBOSE);
				last_sent_actual_rpm = actual_rpm_to_send;
			}
		}
		else
		{ // current_direction == 0 (STOP)
			if (last_sent_actual_rpm != 0 && last_direction != 0)
			{
				// Jika spindle baru saja dihentikan dan RPM terakhir bukan 0,
				// Stop_Hold seharusnya sudah cukup. Set RPM ke 0 bisa redundan.
				// Spindle_Modbus_Set_Rpm(0, MODBUS_VERBOSE);
				last_sent_actual_rpm = 0; // Anggap RPM menjadi 0 setelah stop
			}
		}
		osDelay(50); // Kurangi delay untuk respons SSV yang lebih baik, tapi jangan terlalu cepat
	}
	/* USER CODE END StartModbusSpindleTask */
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
	DEBUG_PRINTF("!!! ERROR HANDLER DIPANGGIL !!!\r\n");
	while (1)
	{
		HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
		for (volatile uint32_t i = 0; i < SystemCoreClock / 50 / 2; i++)
			;
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
	   ex: DEBUG_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

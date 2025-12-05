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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include "usbd_cdc_if.h"

#include "common.h"
#include "ov7670.h"

#include "core_cm7.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  MODE_CAPTURE = 0,
  MODE_SIMULATE = 1,
  MODE_RGB = 2
} operation_mode_t;

typedef struct {
  uint32_t MagicNumber;
  uint32_t Mode;           // operation_mode_t persisted as uint32
  uint32_t MaxFramerate;   // frames per second (ms precision gate)
  uint32_t DisplacedPixelThreshold;
  uint32_t ErrorThreshold;
  uint32_t Reserved[3];
} __attribute__((aligned(32))) AppConfig_t;

typedef struct {
  float dcmi_ms;
  float depth_ms;
  float usb_ms;
  uint32_t displaced_pixels;
  uint8_t has_dcmi;
  uint8_t valid;
} FrameMetricEntry;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMG_WIDTH 320
#define IMG_HEIGHT 240

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define PHASE_TO_U16_SCALE (10430.378f)	// Used convert floats to normalized uint16_t scale

// --- SIMULATION PARAMETERS ---
// Amplitude (reflectivity). Max is 4095. Must be < OFFSET.
#define SIM_AMPLITUDE 2000.0f
// Offset (ambient light).
#define SIM_OFFSET 2048

// "Depths" as phase angles (in radians)
#define PHASE_WALL_FAR   (M_PI / 1.5f)  //
#define PHASE_WALL_NEAR  (M_PI / 4.0f)  // 45 degrees
#define PHASE_CUBE_OBJ   (M_PI / 2.0f)  // 90 degrees

// Legacy checkerboard/cube parameters kept for reference
#define CHECKER_SIZE 20
#define CUBE_SIZE 40

#define WALL_PHASE_VARIATION 0.12f
#define DOOR_WIDTH 110
#define DOOR_HEIGHT 140
#define DOOR_FRAME_THICKNESS 6
#define DOOR_BOTTOM (IMG_HEIGHT - 45)
#define DOORWAY_DEPTH_BOOST 0.65f
#define DOOR_FRAME_PHASE (PHASE_WALL_NEAR - 0.04f)
#define FLOOR_NEAR_PHASE (0.08f)
#define INTRUDER_WIDTH 10
#define INTRUDER_HEIGHT (INTRUDER_WIDTH * 2)
#define ROOM2_FLOOR_INTERSECT_RATIO 0.25f
#define INTRUDER_FLOOR_CONTACT_RATIO 0.55f
#define INTRUDER_PHASE (PHASE_WALL_NEAR + 0.405f)
#define INTRUDER_SPEED 6
#define SIM_NOISE_PHASE 0.060f

#define CONFIG_FLASH_ADDR       0x081E0000U
#define CONFIG_MAGIC            0xDEADBEEFU
#define MIN_ALLOWED_FRAMERATE   1U
#define MAX_ALLOWED_FRAMERATE   120U
#define DEFAULT_MAX_FRAMERATE   30U
#define METRIC_WINDOW_SIZE      10U
#define MIN_DISPLACED_THRESHOLD  1U
#define MAX_DISPLACED_THRESHOLD  (IMG_WIDTH * IMG_HEIGHT)
#define DEFAULT_DISPLACED_THRESHOLD 1200U
#define MIN_ERROR_THRESHOLD      0U
#define MAX_ERROR_THRESHOLD      65535U
#define DEFAULT_ERROR_THRESHOLD  1820U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

#define SINGLE_FRAME_SIZE (IMG_WIDTH * IMG_HEIGHT) // 320*240 = 76800
#define NUM_FRAMES 4

// One giant buffer to hold all 4 frames contiguously
uint16_t quad_frame_buffer[SINGLE_FRAME_SIZE * NUM_FRAMES];

// One buffer to reference frame angles
uint16_t reference_angle_buffer[SINGLE_FRAME_SIZE];

// One buffer for current frame angles
uint16_t current_angle_buffer[SINGLE_FRAME_SIZE];

// Pointers to the start of each frame
uint16_t* p_frame1 = &quad_frame_buffer[SINGLE_FRAME_SIZE * 0];
uint16_t* p_frame2 = &quad_frame_buffer[SINGLE_FRAME_SIZE * 1];
uint16_t* p_frame3 = &quad_frame_buffer[SINGLE_FRAME_SIZE * 2];
uint16_t* p_frame4 = &quad_frame_buffer[SINGLE_FRAME_SIZE * 3];

static const uint32_t kDcmiDmaLength = (SINGLE_FRAME_SIZE * NUM_FRAMES) / 2; // words per HAL call

uint8_t rx_byte = 0;
uint8_t rx_buffer[64];
uint8_t rx_index = 0;

volatile AppConfig_t CurrentConfig;
static operation_mode_t active_mode = (operation_mode_t)0xFF;

static uint32_t frame_interval_ms = 0;
static uint32_t last_frame_processed_tick = 0;

// Flag to tell the main loop the 4-frame set is ready
volatile uint8_t all_frames_ready = 0;
static volatile uint32_t frame_capture_count = 0;

// Button press flag
volatile uint8_t calibration_requested = 0;

// Some timer variables to be used with DWT
volatile uint32_t tDCMI_start, tDCMI_end,
					tUSB_start, tUSB_end,
					tDepthCalc_start, tDepthCalc_end;

extern USBD_HandleTypeDef hUsbDeviceHS;
//uint8_t stream_started = 0; // Not sure why this was here

static volatile uint8_t simulate_cube_visible = 0;

static FrameMetricEntry metrics_window[METRIC_WINDOW_SIZE];
static uint8_t metrics_index = 0;
static uint8_t metrics_filled = 0;
static uint32_t displaced_pixel_threshold = DEFAULT_DISPLACED_THRESHOLD;
static uint16_t error_threshold_counts = DEFAULT_ERROR_THRESHOLD;
static uint8_t intrusion_led_state = 0U;
static uint8_t capture_frames_needed = NUM_FRAMES;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_DCMI_Init(void);
/* USER CODE BEGIN PFP */
void vprint(const char *fmt, va_list argp);
void my_printf(const char *fmt, ...);
void usb_printf(char *msg);
static void USB_Transmit_Blocking(uint8_t* Buf, uint16_t Len);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint32_t get_sector(uint32_t Address);
void save_config(void);
void load_config(void);
void apply_mode(operation_mode_t new_mode);
void update_frame_interval(uint32_t fps);
uint8_t frame_rate_gate_allows(uint32_t now_ms);
void ParseCommand(uint8_t* Buf, uint32_t Len);

void DWT_Init(void);
void calculate_depth_simple(void);
uint32_t detect_displacement(void); // returns number of pixels displaced
static void record_frame_metrics(float dcmi_ms, uint8_t dcmi_valid, float depth_ms, float usb_ms, uint32_t displaced_pixels);
static void print_data_log(void);
static uint32_t clamp_displaced_threshold(uint32_t value);
static uint32_t clamp_error_threshold(uint32_t value);
static void refresh_threshold_cache(void);
static void update_intrusion_led(uint8_t active);

void simulate_tof_data(uint8_t show_cube);
static float compute_floor_phase_for_row(int y, float ratio_cap, float floor_denominator, float floor_phase_span);

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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  MX_DCMI_Init();
  /* USER CODE BEGIN 2 */
  // Don't exactly know why but these caches need to be manually enabled
  SCB_EnableICache();
  SCB_EnableDCache();

  // us timer initialize
  DWT_Init();

  // Camera initialize TODO: significant changes needed when moving to epc660
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  ov7670_init(&hdcmi, &hdma_dcmi, &hi2c2);
  ov7670_config(OV7670_MODE_QVGA_RGB565);
  ov7670_stopCap();

  load_config();
  update_frame_interval(CurrentConfig.MaxFramerate);
  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

  operation_mode_t requested_mode = (CurrentConfig.Mode <= MODE_RGB) ? (operation_mode_t)CurrentConfig.Mode : MODE_CAPTURE;
  if (requested_mode != (operation_mode_t)CurrentConfig.Mode)
  {
    CurrentConfig.Mode = requested_mode;
    save_config();
  }
  apply_mode(requested_mode);

  // I don't know if the frame header is useful - can be removed later
  const uint8_t FRAME_HEADER[4] = {0xAA, 0x55, 0xAA, 0x55};

  const uint32_t frame_timeout_ms = 1000U;
  uint32_t last_frame_tick = HAL_GetTick();

  float cpu_freq_mhz = (float)HAL_RCC_GetHCLKFreq() / 1000000.0f;

  // Clear flags (These are triggered in callbacks)
  all_frames_ready = 0;
  frame_capture_count = 0;
  update_intrusion_led(0U);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      uint32_t now_ms = HAL_GetTick();

      uint8_t frame_source = 0; // 1=capture, 2=simulate

      if (((active_mode == MODE_CAPTURE) || (active_mode == MODE_RGB)) && (all_frames_ready == 1))
      {
        if (frame_rate_gate_allows(now_ms))
        {
          frame_source = 1;
        }
      }
      else if (active_mode == MODE_SIMULATE)
      {
        if (frame_rate_gate_allows(now_ms))
        {
          frame_source = 2;
        }
      }

      if (frame_source != 0)
      {
        last_frame_processed_tick = now_ms;
        float dcmi_capture_ms = 0.0f;
        uint8_t dcmi_sample_valid = 0U;
        float depth_calc_ms = 0.0f;
        uint32_t displaced_pixels = 0U;
        uint8_t is_rgb_mode = (active_mode == MODE_RGB) ? 1U : 0U;

        if (frame_source == 1)
        {
          // End timer for frame capture (happens inside DCMI frame event callback)
          dcmi_capture_ms = (float)(tDCMI_end - tDCMI_start) / cpu_freq_mhz;
          dcmi_sample_valid = 1U;
          all_frames_ready = 0;
          SCB_InvalidateDCache_by_Addr((uint32_t*)quad_frame_buffer, SINGLE_FRAME_SIZE * NUM_FRAMES * 2);
          if (!is_rgb_mode)
          {
            tDepthCalc_start = DWT->CYCCNT;
          }
        }
        else
        {
          tDepthCalc_start = DWT->CYCCNT;
          simulate_tof_data(simulate_cube_visible);
        }

        uint8_t* payload_ptr = (uint8_t*)current_angle_buffer;
        uint32_t payload_bytes = SINGLE_FRAME_SIZE * 2U;

        if (!is_rgb_mode)
        {
          calculate_depth_simple();
          SCB_CleanDCache_by_Addr((uint32_t*)quad_frame_buffer, SINGLE_FRAME_SIZE * NUM_FRAMES * 2);

          if (calibration_requested == 1)
          {
            memcpy(reference_angle_buffer, current_angle_buffer, SINGLE_FRAME_SIZE * 2);
            SCB_CleanDCache_by_Addr((uint32_t*)reference_angle_buffer, SINGLE_FRAME_SIZE * 2);
            calibration_requested = 0;
            my_printf("Reference frame CALIBRATED.\r\n");
          }

          displaced_pixels = detect_displacement();
          uint8_t intrusion_active = (displaced_pixels >= displaced_pixel_threshold) ? 1U : 0U;
          update_intrusion_led(intrusion_active);

          tDepthCalc_end = DWT->CYCCNT;
          depth_calc_ms = (float)(tDepthCalc_end - tDepthCalc_start) / cpu_freq_mhz;
        }
        else
        {
          // RGB passthrough mode: no depth math
          SCB_CleanDCache_by_Addr((uint32_t*)quad_frame_buffer, SINGLE_FRAME_SIZE * NUM_FRAMES * 2);
          if (calibration_requested == 1)
          {
            my_printf("Calibration ignored in MODE_RGB.\r\n");
            calibration_requested = 0;
          }
          update_intrusion_led(0U);
          payload_ptr = (uint8_t*)p_frame1;
        }

        if (!is_rgb_mode)
        {
          payload_ptr = (uint8_t*)current_angle_buffer;
        }

        if (frame_source == 1)
        {
          tDCMI_start = DWT->CYCCNT;
          HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)quad_frame_buffer, kDcmiDmaLength);
        }

        tUSB_start = DWT->CYCCNT;
        while(hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED);

        USB_Transmit_Blocking((uint8_t*)FRAME_HEADER, 4);

        uint8_t* p_buffer = payload_ptr;
        uint32_t bytes_remaining = payload_bytes;
        uint16_t chunk_size;

        while (bytes_remaining > 0)
        {
          if (bytes_remaining > 65535)
          {
            chunk_size = 65535;
          }
          else
          {
            chunk_size = (uint16_t)bytes_remaining;
          }

          USB_Transmit_Blocking(p_buffer, chunk_size);
          p_buffer += chunk_size;
          bytes_remaining -= chunk_size;
        }

        tUSB_end = DWT->CYCCNT;
        float usb_tx_ms = (float)(tUSB_end - tUSB_start) / cpu_freq_mhz;

        record_frame_metrics(dcmi_capture_ms, dcmi_sample_valid, depth_calc_ms, usb_tx_ms, displaced_pixels);

        last_frame_tick = now_ms;
      }

      if (((active_mode == MODE_CAPTURE) || (active_mode == MODE_RGB)) && (now_ms - last_frame_tick > frame_timeout_ms))
      {
        HAL_DCMI_Stop(&hdcmi);
        HAL_DCMI_DeInit(&hdcmi);
        MX_DCMI_Init();
        HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)quad_frame_buffer, kDcmiDmaLength);
        last_frame_tick = HAL_GetTick();
      }


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

  /*AXI clock gating */
  RCC->CKGAENR = 0xE003FFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL1_DIVQ);
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLL1QCLK, RCC_MCODIV_4);
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CAMERA_RESET_Pin|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CAMERA_RESET_Pin PD12 */
  GPIO_InitStruct.Pin = CAMERA_RESET_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*************************************************** USER DEFINED FUNCTIONS ********************************************************/
/*************************************************** USER DEFINED FUNCTIONS ********************************************************/
/*************************************************** USER DEFINED FUNCTIONS ********************************************************/

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
  frame_capture_count++;

  if (frame_capture_count >= capture_frames_needed)
  {
	  HAL_DCMI_Stop(hdcmi);
	  all_frames_ready = 1;
	  frame_capture_count = 0;

	  // Stop frame capture timer
	  tDCMI_end = DWT->CYCCNT;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // Check if the interrupt came from our button (PC13)
  if (GPIO_Pin == GPIO_PIN_13)
  {
    simulate_cube_visible = !(simulate_cube_visible);
    my_printf("Simulation intruder %s\r\n", simulate_cube_visible ? "ENABLED" : "DISABLED");
  }
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi)
{
  if (HAL_DCMI_GetError(hdcmi) == HAL_DCMI_ERROR_OVF)
  {
    my_printf("!!! DCMI OVERRUN ERROR !!!\r\n");
  }
  // You may need to reset the DCMI here
}

void vprint(const char *fmt, va_list argp) {
	char string[200];
	if (0 < vsprintf(string, fmt, argp)) // build string
	{
		HAL_UART_Transmit(&huart3, (uint8_t*) string, strlen(string), 0xffffff); // send message via UART
	}
}

void my_printf(const char *fmt, ...) // custom printf() function
{
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}

void usb_printf(char *msg)
{
	if (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED)
	{
	  // 3. Send the data
	  CDC_Transmit_HS((uint8_t*)msg, strlen(msg));
	}
}

static void USB_Transmit_Blocking(uint8_t* Buf, uint16_t Len)
{
    USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;

    // Wait until previous transfer completes
    while (hcdc->TxState != 0)
    {
        // Optionally add a timeout here to avoid infinite lockup
    }

    // Queue new transfer
    CDC_Transmit_HS(Buf, Len);

    // Wait until this one completes
    while (hcdc->TxState != 0)
    {
        // Optionally add small delay for CPU efficiency
    }
}

void DWT_Init(void)
{
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }
  if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk))
  {
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
  DWT->CYCCNT = 0;
}

void calculate_depth_simple(void)
{
    for (int i = 0; i < (320 * 240); i++)
    {
        // 1. Read, cast, and subtract
        float y = (float)p_frame1[i] - (float)p_frame3[i];
        float x = (float)p_frame2[i] - (float)p_frame4[i];

        // 2. Calculate and store
        float phase = atan2f(y, x);

        uint16_t scaled_angle = (uint16_t)((phase + M_PI) * PHASE_TO_U16_SCALE);

        current_angle_buffer[i] = scaled_angle;
    }


}

uint32_t detect_displacement(void)
{
  uint32_t displaced_pixel_count = 0;
  const uint16_t threshold = error_threshold_counts;

  for (int i = 0; i < SINGLE_FRAME_SIZE; i++)
  {
    int32_t diff = abs((int32_t)current_angle_buffer[i] - (int32_t)reference_angle_buffer[i]);
    if (diff > 32768)
    {
      diff = 65536 - diff;
    }

    if (diff > threshold)
    {
      displaced_pixel_count++;
    }
  }
  return displaced_pixel_count;
}

static uint32_t clamp_displaced_threshold(uint32_t value)
{
  if (value < MIN_DISPLACED_THRESHOLD)
  {
    return MIN_DISPLACED_THRESHOLD;
  }
  if (value > MAX_DISPLACED_THRESHOLD)
  {
    return MAX_DISPLACED_THRESHOLD;
  }
  return value;
}

static uint32_t clamp_error_threshold(uint32_t value)
{
  if (value < MIN_ERROR_THRESHOLD)
  {
    return MIN_ERROR_THRESHOLD;
  }
  if (value > MAX_ERROR_THRESHOLD)
  {
    return MAX_ERROR_THRESHOLD;
  }
  return value;
}

static void refresh_threshold_cache(void)
{
  displaced_pixel_threshold = clamp_displaced_threshold(CurrentConfig.DisplacedPixelThreshold);
  error_threshold_counts = (uint16_t)clamp_error_threshold(CurrentConfig.ErrorThreshold);
}

static void update_intrusion_led(uint8_t active)
{
  if (intrusion_led_state == active)
  {
    return;
  }
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, active ? GPIO_PIN_SET : GPIO_PIN_RESET);
  intrusion_led_state = active;
}

static void record_frame_metrics(float dcmi_ms, uint8_t dcmi_valid, float depth_ms, float usb_ms, uint32_t displaced_pixels)
{
  FrameMetricEntry *slot = &metrics_window[metrics_index];
  slot->dcmi_ms = dcmi_ms;
  slot->depth_ms = depth_ms;
  slot->usb_ms = usb_ms;
  slot->displaced_pixels = displaced_pixels;
  slot->has_dcmi = dcmi_valid;
  slot->valid = 1U;

  metrics_index = (metrics_index + 1U) % METRIC_WINDOW_SIZE;
  if (metrics_filled < METRIC_WINDOW_SIZE)
  {
    metrics_filled++;
  }
}

static void print_data_log(void)
{
  if (metrics_filled == 0U)
  {
    my_printf("DATA_LOG: No frame data captured yet.\r\n");
    return;
  }

  float sum_depth = 0.0f;
  float sum_usb = 0.0f;
  float sum_dcmi = 0.0f;
  uint64_t sum_displaced = 0ULL;
  uint8_t dcmi_samples = 0U;

  for (uint8_t i = 0U; i < METRIC_WINDOW_SIZE; i++)
  {
    if (metrics_window[i].valid == 0U)
    {
      continue;
    }

    sum_depth += metrics_window[i].depth_ms;
    sum_usb += metrics_window[i].usb_ms;
    sum_displaced += metrics_window[i].displaced_pixels;

    if (metrics_window[i].has_dcmi)
    {
      sum_dcmi += metrics_window[i].dcmi_ms;
      dcmi_samples++;
    }
  }

  float depth_avg = sum_depth / (float)metrics_filled;
  float usb_avg = sum_usb / (float)metrics_filled;
  float displaced_avg = (float)sum_displaced / (float)metrics_filled;

  if (dcmi_samples > 0U)
  {
    my_printf("Avg DCMI capture (%u frames): %.3f ms\r\n", dcmi_samples, sum_dcmi / (float)dcmi_samples);
  }
  else
  {
    my_printf("Avg DCMI capture: N/A (no capture frames)\r\n");
  }

  my_printf("Avg depth calc (%u frames): %.3f ms\r\n", metrics_filled, depth_avg);
  my_printf("Avg USB TX (%u frames): %.3f ms\r\n", metrics_filled, usb_avg);
  my_printf("Avg displaced pixels (%u frames): %.1f\r\n", metrics_filled, displaced_avg);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    if (rx_byte == '\n' || rx_byte == '\r')
    {
      if (rx_index > 0)
      {
        rx_buffer[rx_index] = '\0';
        ParseCommand(rx_buffer, rx_index);
      }
      rx_index = 0;
      memset(rx_buffer, 0, sizeof(rx_buffer));
    }
    else if ((rx_byte < 0x20U) && (rx_byte != '\t'))
    {
    // Ignore leading control characters (e.g. stray 0x00 from VCP connect)
    }
    else if (rx_index < sizeof(rx_buffer) - 1)
    {
      rx_buffer[rx_index++] = rx_byte;
    }

    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  }
}

uint32_t get_sector(uint32_t Address)
{
  uint32_t offset = Address - 0x08000000U;
  if (offset >= 0x100000U)
  {
    return (offset - 0x100000U) / 0x2000U;
  }
  return offset / 0x2000U;
}

void save_config(void)
{
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef EraseInitStruct = {0};
  uint32_t SectorError = 0;

  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.Banks = FLASH_BANK_2;
  EraseInitStruct.Sector = get_sector(CONFIG_FLASH_ADDR);
  EraseInitStruct.NbSectors = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
    my_printf("Erase Failed: %lu\r\n", SectorError);
    HAL_FLASH_Lock();
    return;
  }

  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, CONFIG_FLASH_ADDR, (uint32_t)&CurrentConfig) != HAL_OK)
  {
    my_printf("Program Failed\r\n");
  }

  HAL_FLASH_Lock();
}

void load_config(void)
{
  AppConfig_t *flash_cfg = (AppConfig_t *)CONFIG_FLASH_ADDR;
  uint8_t needs_save = 0;

  if (flash_cfg->MagicNumber == CONFIG_MAGIC)
  {
    memcpy((void*)&CurrentConfig, flash_cfg, sizeof(AppConfig_t));
  }
  else
  {
    memset((void*)&CurrentConfig, 0, sizeof(AppConfig_t));
    CurrentConfig.MagicNumber = CONFIG_MAGIC;
    CurrentConfig.Mode = MODE_CAPTURE;
    CurrentConfig.MaxFramerate = DEFAULT_MAX_FRAMERATE;
    CurrentConfig.DisplacedPixelThreshold = DEFAULT_DISPLACED_THRESHOLD;
    CurrentConfig.ErrorThreshold = DEFAULT_ERROR_THRESHOLD;
    needs_save = 1;
  }

  if (CurrentConfig.Mode > MODE_RGB)
  {
    CurrentConfig.Mode = MODE_CAPTURE;
    needs_save = 1;
  }

  if ((CurrentConfig.MaxFramerate > MAX_ALLOWED_FRAMERATE) && (CurrentConfig.MaxFramerate != 0))
  {
    CurrentConfig.MaxFramerate = MAX_ALLOWED_FRAMERATE;
    needs_save = 1;
  }

  if (CurrentConfig.MaxFramerate == 0)
  {
    // 0 is treated as unlimited; nothing to clamp
  }
  else if (CurrentConfig.MaxFramerate < MIN_ALLOWED_FRAMERATE)
  {
    CurrentConfig.MaxFramerate = MIN_ALLOWED_FRAMERATE;
    needs_save = 1;
  }

  if ((CurrentConfig.DisplacedPixelThreshold < MIN_DISPLACED_THRESHOLD) ||
      (CurrentConfig.DisplacedPixelThreshold > MAX_DISPLACED_THRESHOLD))
  {
    CurrentConfig.DisplacedPixelThreshold = DEFAULT_DISPLACED_THRESHOLD;
    needs_save = 1;
  }

  if ((CurrentConfig.ErrorThreshold < MIN_ERROR_THRESHOLD) ||
      (CurrentConfig.ErrorThreshold > MAX_ERROR_THRESHOLD))
  {
    CurrentConfig.ErrorThreshold = DEFAULT_ERROR_THRESHOLD;
    needs_save = 1;
  }

  if (needs_save)
  {
    save_config();
  }

  refresh_threshold_cache();
}

void apply_mode(operation_mode_t new_mode)
{
  if (new_mode > MODE_RGB)
  {
    new_mode = MODE_CAPTURE;
  }

  HAL_DCMI_Stop(&hdcmi);
  frame_capture_count = 0;
  all_frames_ready = 0;

  capture_frames_needed = (new_mode == MODE_RGB) ? 1U : NUM_FRAMES;

  if (new_mode == MODE_CAPTURE || new_mode == MODE_RGB)
  {
    HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)quad_frame_buffer, kDcmiDmaLength);
  }

  if (active_mode != new_mode)
  {
    const char *mode_name = (new_mode == MODE_CAPTURE) ? "MODE_CAPTURE" :
                           (new_mode == MODE_SIMULATE ? "MODE_SIMULATE" : "MODE_RGB");
    my_printf("Switched to %s\r\n", mode_name);
  }

  active_mode = new_mode;
}

void update_frame_interval(uint32_t fps)
{
  frame_interval_ms = (fps == 0U) ? 0U : (1000U / fps);
  if (frame_interval_ms == 0U && fps != 0U)
  {
    frame_interval_ms = 1U;
  }

  if (frame_interval_ms == 0U)
  {
    last_frame_processed_tick = 0U;
  }
  else
  {
    uint32_t now = HAL_GetTick();
    last_frame_processed_tick = (now > frame_interval_ms) ? (now - frame_interval_ms) : 0U;
  }
}

uint8_t frame_rate_gate_allows(uint32_t now_ms)
{
  if (frame_interval_ms == 0)
  {
    return 1;
  }
  return (uint8_t)((now_ms - last_frame_processed_tick) >= frame_interval_ms);
}

void ParseCommand(uint8_t* Buf, uint32_t Len)
{
  if (Len == 0)
  {
    return;
  }

  my_printf("Command: %s\r\n", Buf);

  if (strncmp((char*)Buf, "MODE_CAPTURE", 12) == 0)
  {
    if (CurrentConfig.Mode != MODE_CAPTURE)
    {
      CurrentConfig.Mode = MODE_CAPTURE;
      save_config();
    }
    apply_mode(MODE_CAPTURE);
  }
  else if (strncmp((char*)Buf, "MODE_SIMULATE", 13) == 0)
  {
    if (CurrentConfig.Mode != MODE_SIMULATE)
    {
      CurrentConfig.Mode = MODE_SIMULATE;
      save_config();
    }
    apply_mode(MODE_SIMULATE);
  }
  else if (strncmp((char*)Buf, "MODE_RGB", 8) == 0)
  {
    if (CurrentConfig.Mode != MODE_RGB)
    {
      CurrentConfig.Mode = MODE_RGB;
      save_config();
    }
    apply_mode(MODE_RGB);
  }
  else if (strncmp((char*)Buf, "MAX_FRAMERATE=", 14) == 0)
  {
    uint32_t fps = (uint32_t)strtoul((char*)Buf + 14, NULL, 10);
    if (fps == 0)
    {
      CurrentConfig.MaxFramerate = 0;
    }
    else
    {
      if (fps > MAX_ALLOWED_FRAMERATE)
      {
        fps = MAX_ALLOWED_FRAMERATE;
      }
      if (fps < MIN_ALLOWED_FRAMERATE)
      {
        fps = MIN_ALLOWED_FRAMERATE;
      }
      CurrentConfig.MaxFramerate = fps;
    }
    save_config();
    update_frame_interval(CurrentConfig.MaxFramerate);
    my_printf("MAX_FRAMERATE set to %lu\r\n", (unsigned long)CurrentConfig.MaxFramerate);
  }
  else if (strncmp((char*)Buf, "CALIBRATE", 9) == 0)
  {
    calibration_requested = 1;
    my_printf("Calibration request queued\r\n");
  }
  else if (strncmp((char*)Buf, "DISPLACED_THRESHOLD=", 20) == 0)
  {
    uint32_t threshold = (uint32_t)strtoul((char*)Buf + 20, NULL, 10);
    threshold = clamp_displaced_threshold(threshold);
    CurrentConfig.DisplacedPixelThreshold = threshold;
    displaced_pixel_threshold = threshold;
    save_config();
    my_printf("DISPLACED_THRESHOLD set to %lu\r\n", (unsigned long)threshold);
  }
  else if (strncmp((char*)Buf, "ERROR_THRESHOLD=", 16) == 0)
  {
    uint32_t threshold = (uint32_t)strtoul((char*)Buf + 16, NULL, 10);
    threshold = clamp_error_threshold(threshold);
    CurrentConfig.ErrorThreshold = threshold;
    error_threshold_counts = (uint16_t)threshold;
    save_config();
    my_printf("ERROR_THRESHOLD set to %lu\r\n", (unsigned long)threshold);
  }
  else if (strncmp((char*)Buf, "DATA_LOG", 8) == 0)
  {
    print_data_log();
  }
  else
  {
    my_printf("Unknown command: %s\r\n", Buf);
  }
}

void simulate_tof_data(uint8_t show_cube)
{
  float phase_to_use;
  uint16_t amp = (uint16_t)SIM_AMPLITUDE;
  uint16_t offset = (uint16_t)SIM_OFFSET;

  const int door_center_x = IMG_WIDTH / 2;
  const int door_half_w = DOOR_WIDTH / 2;
  const int door_bottom = DOOR_BOTTOM;
  const int door_top = door_bottom - DOOR_HEIGHT;
  const int door_center_y = (door_top + door_bottom) / 2;
  int door_inner_left = door_center_x - door_half_w + DOOR_FRAME_THICKNESS;
  int door_inner_right = door_center_x + door_half_w - DOOR_FRAME_THICKNESS;
  int door_inner_top = door_top + DOOR_FRAME_THICKNESS;
  int door_inner_bottom = door_bottom;
  if (door_inner_left < 0)
  {
    door_inner_left = 0;
  }
  if (door_inner_right >= IMG_WIDTH)
  {
    door_inner_right = IMG_WIDTH - 1;
  }
  if (door_inner_right <= door_inner_left)
  {
    door_inner_right = door_inner_left + 1;
  }
  if (door_inner_top < 0)
  {
    door_inner_top = 0;
  }
  if (door_inner_top >= door_inner_bottom)
  {
    door_inner_top = door_inner_bottom - 1;
  }
  const int floor_start = door_bottom;
  const float wall_phase = PHASE_WALL_NEAR;
  const float floor_phase_span = wall_phase - FLOOR_NEAR_PHASE;
  const float floor_denominator = (float)((IMG_HEIGHT - 1) - floor_start);

  int floor_meet_y = door_bottom - (int)((float)DOOR_HEIGHT * ROOM2_FLOOR_INTERSECT_RATIO);
  if (floor_meet_y < door_top)
  {
    floor_meet_y = door_top;
  }
  if (floor_meet_y > door_bottom)
  {
    floor_meet_y = door_bottom;
  }

  float room2_floor_ratio_target = 1.0f;
  if (floor_denominator > 0.0f)
  {
    room2_floor_ratio_target = (float)(IMG_HEIGHT - 1 - floor_meet_y) / floor_denominator;
    room2_floor_ratio_target = fmaxf(room2_floor_ratio_target, 1.0f);
  }

  const float room2_wall_phase = compute_floor_phase_for_row(floor_meet_y,
                                                             room2_floor_ratio_target,
                                                             floor_denominator,
                                                             floor_phase_span);

  int intruder_bottom_y = floor_meet_y + (int)((door_bottom - floor_meet_y) * INTRUDER_FLOOR_CONTACT_RATIO);
  if (intruder_bottom_y < floor_meet_y)
  {
    intruder_bottom_y = floor_meet_y;
  }
  if (intruder_bottom_y > door_bottom)
  {
    intruder_bottom_y = door_bottom;
  }

  int intruder_top_y = intruder_bottom_y - INTRUDER_HEIGHT;
  if (intruder_top_y < door_inner_top)
  {
    intruder_top_y = door_inner_top;
  }
  if (intruder_top_y >= intruder_bottom_y)
  {
    intruder_top_y = intruder_bottom_y - 1;
    if (intruder_top_y < door_inner_top)
    {
      intruder_top_y = door_inner_top;
      intruder_bottom_y = intruder_top_y + 1;
    }
  }

  const float intruder_bottom_phase = compute_floor_phase_for_row(intruder_bottom_y,
                                                                  room2_floor_ratio_target,
                                                                  floor_denominator,
                                                                  floor_phase_span);

  static int intruder_x = -INTRUDER_WIDTH;
  static uint32_t noise_state = 0x13579BDFu;

  if (!show_cube)
  {
    intruder_x = door_inner_left - INTRUDER_WIDTH;
  }
  else
  {
    intruder_x += INTRUDER_SPEED;
    int reset_x = door_inner_right + INTRUDER_WIDTH;
    int start_x = door_inner_left - INTRUDER_WIDTH;

    if (intruder_x > reset_x)
    {
      intruder_x = start_x;
    }
  }

  // Loop over every pixel
  for (int y = 0; y < IMG_HEIGHT; y++)
  {
    for (int x = 0; x < IMG_WIDTH; x++)
    {
      // Start with the near wall depth as the default
      phase_to_use = wall_phase;

      // Ramp the floor depth from camera (bottom) up to the wall seam
      if (y >= floor_start)
      {
        phase_to_use = compute_floor_phase_for_row(y, 1.0f, floor_denominator, floor_phase_span);
      }

      int dx = x - door_center_x;
      int inside_door = (abs(dx) <= door_half_w) && (y >= door_top) && (y <= door_bottom);
      int on_frame = (abs(abs(dx) - door_half_w) < DOOR_FRAME_THICKNESS && y >= door_top && y <= door_bottom) ||
               (y >= door_top && y < door_top + DOOR_FRAME_THICKNESS && abs(dx) <= door_half_w);

      if (on_frame)
      {
        phase_to_use = DOOR_FRAME_PHASE;
      }
      else if (inside_door)
      {
        if (y >= floor_meet_y)
        {
          phase_to_use = compute_floor_phase_for_row(y, room2_floor_ratio_target, floor_denominator, floor_phase_span);
        }
        else
        {
          phase_to_use = room2_wall_phase;
        }
      }

      int intruder_in_row = (y >= intruder_top_y) && (y <= intruder_bottom_y);
      int intruder_in_col = (x >= intruder_x) && (x < (intruder_x + INTRUDER_WIDTH));
      int inside_door_opening = (x >= door_inner_left) && (x <= door_inner_right) &&
                                (y >= door_inner_top) && (y <= door_inner_bottom);
      int intruder_in_aperture = intruder_in_row && intruder_in_col && inside_door_opening;
      if (show_cube && intruder_in_aperture)
      {
        float intruder_phase = INTRUDER_PHASE;
        if (intruder_bottom_y > intruder_top_y)
        {
          float span = (float)(intruder_bottom_y - intruder_top_y);
          float mix = (float)(y - intruder_top_y) / span;
          mix = fminf(fmaxf(mix, 0.0f), 1.0f);
          intruder_phase = INTRUDER_PHASE + mix * (intruder_bottom_phase - INTRUDER_PHASE);
        }
        phase_to_use = intruder_phase;
      }

      // Inject lightweight pseudo-random noise (after object logic so every region shares the same variance)
      noise_state = noise_state * 1664525u + 1013904223u;
      int16_t noise_raw = (int16_t)(noise_state >> 16);
      float noise = ((float)noise_raw / 32768.0f) * SIM_NOISE_PHASE;
      phase_to_use += noise;

      // --- 2. Calculate the four frame values for this pixel ---
      float sinP = sinf(phase_to_use);
      float cosP = cosf(phase_to_use);
      int idx = (y * IMG_WIDTH) + x;

      p_frame1[idx] = offset + (uint16_t)(amp * sinP);
      p_frame2[idx] = offset + (uint16_t)(amp * cosP);
      p_frame3[idx] = offset - (uint16_t)(amp * sinP);
      p_frame4[idx] = offset - (uint16_t)(amp * cosP);
    }
  }
}

static float compute_floor_phase_for_row(int y, float ratio_cap, float floor_denominator, float floor_phase_span)
{
  if (floor_denominator <= 0.0f || floor_phase_span <= 0.0f)
  {
    return FLOOR_NEAR_PHASE;
  }

  float ratio = (float)(IMG_HEIGHT - 1 - y) / floor_denominator;
  ratio = fminf(fmaxf(ratio, 0.0f), ratio_cap);

  return FLOOR_NEAR_PHASE + ratio * floor_phase_span;
}

#define CHUNK_SIZE 256

/*************************************************** USER DEFINED FUNCTIONS ********************************************************/
/*************************************************** USER DEFINED FUNCTIONS ********************************************************/
/*************************************************** USER DEFINED FUNCTIONS ********************************************************/
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
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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

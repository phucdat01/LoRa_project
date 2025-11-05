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
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LoRa.h"
#include "sht3x.h"
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

/* USER CODE BEGIN PV */
sht3x_handle_t sht3x;
LoRa myLoRa;							// Cấu trúc cấu hình LoRa
uint16_t LoRa_status;					// Trạng thái khởi tạo LoRa
char TxBuffer[256];
char DebugBuffer[256];
extern volatile uint8_t TxDoneFlag;	    // Cờ báo ngắt TX Done từ LoRa (set trong callback EXTI)
volatile uint8_t TimerFlag = 0;			// Cờ báo tick định kỳ từ TIM1 (set trong callback Timer)
float temp, hum;
uint16_t soil_raw = 0;
float soil_vol = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float SoilMoisture_Convert(uint16_t adc_value);   // Hàm chuyển đổi giá trị ADC sang phần trăm độ ẩm đất
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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  sht3x.i2c_handle = &hi2c1;
  sht3x.device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW; // 0x44
  if (!sht3x_init(&sht3x)) {
	  char msg[] = "SHT3x init failed!\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
      while (1);
  }
  char msg[] = "SHT3x init OK\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);


  myLoRa = newLoRa();
  myLoRa.CS_port         = NSS_GPIO_Port;
  myLoRa.CS_pin          = NSS_Pin;
  myLoRa.reset_port      = RST_GPIO_Port;
  myLoRa.reset_pin       = RST_Pin;
  myLoRa.DIO0_port       = DIO0_GPIO_Port;
  myLoRa.DIO0_pin        = DIO0_Pin;
  myLoRa.hSPIx           = &hspi1;

  myLoRa.frequency             = 433;
  myLoRa.spredingFactor        = SF_7;
  myLoRa.bandWidth             = BW_125KHz;
  myLoRa.crcRate               = CR_4_5;
  myLoRa.power                 = POWER_17db;
  myLoRa.overCurrentProtection = 100;
  myLoRa.preamble              = 8;

  LoRa_reset(&myLoRa);
  LoRa_status = LoRa_init(&myLoRa);
  snprintf(DebugBuffer, sizeof(DebugBuffer), "LoRa init result = %d\r\n", LoRa_status);
  HAL_UART_Transmit(&huart1, (uint8_t*)DebugBuffer, strlen(DebugBuffer), 200);
  if (LoRa_status != LORA_OK) {
      snprintf(DebugBuffer, sizeof(DebugBuffer), "Khởi tạo LoRa thất bại: %d\r\n", LoRa_status);
      HAL_UART_Transmit(&huart1, (uint8_t*)DebugBuffer, strlen(DebugBuffer), 200);
      while (1);
  }

  LoRa_setSyncWord(&myLoRa, 0x34);		// Cấu hình Sync Word (mặc định mạng công cộng 0x34)
  LoRa_setTOMsb_setCRCon(&myLoRa);		// Bật CRC cho payload, cấu hình phần đầu gói (TOMsb/CRCon tuỳ thư viện)
  HAL_TIM_Base_Start_IT(&htim1);		// Bật Timer 1 ở chế độ ngắt định kỳ (chu kỳ cấu hình trong MX_TIM1_Init)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	  if (TimerFlag) {
          TimerFlag = 0;			// Xoá cờ để xử lý một lần cho mỗi chu kỳ timer

          // Đọc SHT3x
          if (sht3x_read_temperature_and_humidity(&sht3x, &temp, &hum)) {

        	  // Đọc Soil Moisture
        	  HAL_ADC_Start(&hadc1);
        	  if (HAL_ADC_PollForConversion(&hadc1, 50) == HAL_OK) {
                  soil_raw = HAL_ADC_GetValue(&hadc1);
                  soil_vol = SoilMoisture_Convert(soil_raw);
              }
              HAL_ADC_Stop(&hadc1);

              // Chuẩn bị gói tin LoRa
        	  snprintf(TxBuffer, sizeof(TxBuffer), "T=%.1fC,RH=%.1f%%,Soil=%.1f%%Vol", temp, hum, soil_vol);
        	  LoRa_gotoMode(&myLoRa, STNBY_MODE);
        	  HAL_Delay(2);
        	  LoRa_transmit(&myLoRa, (uint8_t*)TxBuffer, strlen(TxBuffer));
      	  } else {
      		  char err[] = "SHT3x read fail\r\n";
      		  HAL_UART_Transmit(&huart1, (uint8_t*)err, sizeof(err)-1, 50);
      	  }
	  }

	  if (TxDoneFlag) {
          TxDoneFlag = 0;
          snprintf(DebugBuffer, sizeof(DebugBuffer), "TxDone IRQ at %lu ms\r\n", HAL_GetTick());
          HAL_UART_Transmit(&huart1, (uint8_t*)DebugBuffer, strlen(DebugBuffer), 200);
          LoRa_gotoMode(&myLoRa, SLEEP_MODE);
          HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == myLoRa.DIO0_pin)
    {
        uint8_t irqFlags = LoRa_read(&myLoRa, RegIrqFlags); // Đọc thanh ghi cờ ngắt của LoRa để kiểm tra sự kiện
        if (irqFlags & 0x08) {
            TxDoneFlag = 1;	// Set cờ để xử lý trong vòng lặp chính

        }
        LoRa_write(&myLoRa, RegIrqFlags, irqFlags); // Xoá cờ ngắt trong LoRa
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        TimerFlag = 1;	// Báo vòng lặp chính thực hiện đo + gửi

    }
}

/**
 * @brief Chuyển đổi giá trị ADC sang phần trăm độ ẩm đất (%Vol)
 * @param adc_value: giá trị ADC thô (0..4095 tuỳ cấu hình)
 * @note  Hiệu chuẩn giả định:
 *        - 2500 -> 0% (khô)
 *        - 830  -> 60% (ẩm)
 *        Tuyến tính giữa hai điểm. Ngoài khoảng thì kẹp về biên.
 */

float SoilMoisture_Convert(uint16_t adc_value)
{
    // Giới hạn giá trị trong khoảng hiệu chuẩn
    if (adc_value > 2500) adc_value = 2500;
    if (adc_value < 830)  adc_value = 830;

    // Nội suy tuyến tính: 2500 -> 0%, 830 -> 60%
    float percent = (float)(2500 - adc_value) * 60.0f / (2500 - 830);

    return percent;
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

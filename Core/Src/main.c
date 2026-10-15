/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR      (0x68 << 1)
#define MPU6050_PWR_MGMT1 0x6B
#define MPU6050_CONFIG    0x1A
#define MPU6050_GYRO_CFG  0x1B
#define MPU6050_ACCEL_CFG 0x1C
#define MPU6050_ACCEL_OUT 0x3B

#define RAD_TO_DEG   57.29578f
#define Z_BROKEN     1

#define ALPHA        0.98f   // gyro payı
#define LOOP_MS      10      // her turun süresi
#define PRINT_EVERY  10      // 10 turda bir yazdır (yaklaşık 100 ms)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char msg[128];
float gx_off = 0.0f, gy_off = 0.0f;   // gyro sıfır hatası
float roll = 0.0f, pitch = 0.0f;      // filtrelenmiş açılar
uint32_t last_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void format_angle(char *out, float angle)
{
    int32_t a10 = (int32_t)(angle * 10.0f);
    char sign = (a10 < 0) ? '-' : '+';
    if (a10 < 0) a10 = -a10;
    sprintf(out, "%c%ld.%ld", sign, (long)(a10 / 10), (long)(a10 % 10));
}

// Sensörden accel (g) ve gyro (derece/saniye) oku
static uint8_t mpu_read(float *ax, float *ay, float *az, float *gx, float *gy)
{
    uint8_t buf[14];
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_OUT, 1, buf, 14, 100) != HAL_OK)
        return 0;

    int16_t axr = (int16_t)(buf[0]  << 8 | buf[1]);
    int16_t ayr = (int16_t)(buf[2]  << 8 | buf[3]);
    int16_t azr = (int16_t)(buf[4]  << 8 | buf[5]);
    int16_t gxr = (int16_t)(buf[8]  << 8 | buf[9]);
    int16_t gyr = (int16_t)(buf[10] << 8 | buf[11]);

    *ax = axr / 16384.0f;
    *ay = ayr / 16384.0f;
    *az = azr / 16384.0f;
    *gx = gxr / 131.0f;
    *gy = gyr / 131.0f;

#if Z_BROKEN
    // Z'yi X ve Y'den tahmin et (kart yukarı bakıyor varsayımı)
    float rest = 1.0f - (*ax) * (*ax) - (*ay) * (*ay);
    if (rest < 0.0f) rest = 0.0f;
    *az = sqrtf(rest);
#endif
    return 1;
}

// Sadece accelerometer'dan açı hesapla
static void accel_angles(float ax, float ay, float az, float *r, float *p)
{
    *r = atan2f(ay, az) * RAD_TO_DEG;
    *p = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t data = 0;
  float ax, ay, az, gx, gy;
  float sum_gx = 0.0f, sum_gy = 0.0f;
  uint16_t n = 0;
  uint8_t print_cnt = 0;

  HAL_Delay(100);

  // Sensörü sıfırla ve uyandır
  data = 0x80;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT1, 1, &data, 1, 100);
  HAL_Delay(200);
  data = 0x01;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT1, 1, &data, 1, 100);
  HAL_Delay(100);

  // Dahili alçak geçiren filtre (yaklaşık 44 Hz), gürültüyü azaltır
  data = 0x03;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_CONFIG, 1, &data, 1, 100);

  // Gyro ±250 derece/saniye, Accel ±2g
  data = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_CFG, 1, &data, 1, 100);
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_CFG, 1, &data, 1, 100);
  HAL_Delay(50);

  // Gyro kalibrasyonu: kart hareketsizken ortalama sıfır hatasını bul
  snprintf(msg, sizeof(msg), "Kalibrasyon basladi, karti kipirdatma...\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

  for (int i = 0; i < 200; i++)
  {
      if (mpu_read(&ax, &ay, &az, &gx, &gy))
      {
          sum_gx += gx;
          sum_gy += gy;
          n++;
      }
      HAL_Delay(5);
  }
  if (n > 0)
  {
      gx_off = sum_gx / n;
      gy_off = sum_gy / n;
  }

  // Başlangıç açısını accelerometer'dan al
  if (mpu_read(&ax, &ay, &az, &gx, &gy))
  {
      accel_angles(ax, ay, az, &roll, &pitch);
  }

  snprintf(msg, sizeof(msg), "Hazir.\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

  last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (mpu_read(&ax, &ay, &az, &gx, &gy))
	      {
	          // Geçen süre (saniye)
	          uint32_t now = HAL_GetTick();
	          float dt = (now - last_tick) / 1000.0f;
	          last_tick = now;
	          if (dt <= 0.0f || dt > 0.5f) dt = LOOP_MS / 1000.0f;

	          // Gyro sıfır hatasını çıkar
	          gx -= gx_off;
	          gy -= gy_off;

	          // Sadece accelerometer açısı (karşılaştırma için)
	          float roll_acc, pitch_acc;
	          accel_angles(ax, ay, az, &roll_acc, &pitch_acc);

	          // Complementary filter
	          roll  = ALPHA * (roll  + gx * dt) + (1.0f - ALPHA) * roll_acc;
	          pitch = ALPHA * (pitch + gy * dt) + (1.0f - ALPHA) * pitch_acc;

	          print_cnt++;
	          if (print_cnt >= PRINT_EVERY)
	          {
	              print_cnt = 0;
	              char ra[12], rf[12], pf[12];
	              format_angle(ra, roll_acc);
	              format_angle(rf, roll);
	              format_angle(pf, pitch);
	              snprintf(msg, sizeof(msg),
	                       "Roll(acc): %s | Roll(filtre): %s | Pitch(filtre): %s\r\n",
	                       ra, rf, pf);
	              HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	          }
	      }

	      HAL_Delay(LOOP_MS);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

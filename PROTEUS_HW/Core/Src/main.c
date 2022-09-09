/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "dwt_delay.h" //Mikrosaniyelik delay yapabilmemizi sağlayan DWT kütüphanesini ekle
#include "LCD.h" //LCD ekranı kontrol edebilmemizi sağlayan LCD kütüphanesini ekle
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
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
int count = 0; //Geçen mikrosaniyeyi saymak için bir değişken tanımladım
float distance = 0; //Uzaklığı cm biriminden bu değişkene yazdım
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//HC-SR04 modülünü kullanmak için yazdığım fonksiyon
void measure_distance(void){
	  count = 0; //Geçen mikrosaniyeyi 0 yap
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); //HC-SR04'ün Trigger pinine yüksek voltaj ver
	  DWT_Delay(10); //10 mikrosaniye delay yap
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); //HC-SR04'ün Trigger pinine giden yüksek voltajı kapat

	  while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)); //HC-SR04 cevap verene kadar döngüde kal

	  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)){ // Cevap geldikten sonra HC-SR04 yüksek voltaj verdikçe geçen süreyi say
	 	 count++; //Geçen süreyi 1 arttırarak say
	 	 DWT_Delay(1); //Geçen süreyi saymak için 1 mikrosaniye delay
	  }

	  distance = (float) count / 29.1; //HC-SR04'ten yüksek voltaj alınan zamanı 29.1'e bölerek uzaklığı cm cinsinden hesapla
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
  DWT_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  char str[20]; //İleride float değeri stringe dönüştürmek için kullanılacak string
  //PWM sinyali üretmek için kullanılan TIM3'ün 1. ve 2. kanallarını başlat
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  //Sırasıyla GPIO pinlerinin hangi portlardan LCD ekrana bağlanacağını gir
  Lcd_PortType ports[] = { GPIOA, GPIOA, GPIOA, GPIOA };
  //LCD ekranın D4, D5, D6 ve D7 pinlerinin hangi GPIO çıkışlarını kullanacağını gir
  Lcd_PinType pins[] = {GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11};
  Lcd_HandleTypeDef lcd; //lcd'yi kontrol edebilmek için bir struct oluştur
  //LCD ekranı yaratıp önceden ayarladığımız lcd structına bilgileri ver
  lcd = Lcd_create(ports, pins, GPIOA, GPIO_PIN_7, GPIOA, GPIO_PIN_6, LCD_4_BIT_MODE);
  Lcd_cursor(&lcd, 0,0); //LCD ekrana yazılmaya başlanacak satır ve sütunu en başa getir
  Lcd_string(&lcd, "mesafe: "); //LCD üzerine "mesafe: " stringini gir
  Lcd_cursor(&lcd, 1, 0); //LCD ekranın 2. satırının başına cursor'ı getir
  Lcd_string(&lcd, "mod: "); //LCD ekranın üzerine "mod: " stringini yaz
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  measure_distance(); //Uzaklığı hesapla (değer global distance değişkeninde güncellenir)
	  Lcd_cursor(&lcd, 0, 8); //LCD ekranın ilk satırının 9. sütununa imleci getir
	  sprintf(str, "%f", distance); //float cinsinden olan distance'ı stringe çevirip str'ye yaz
	  Lcd_string(&lcd, str); //LCD ekran üzerine uzaklığı yaz

	  //HC-SR04'ü 250 ve 350 cm arasında duracak şekilde revize ettim çünkü kullandığım sensör 400 cm üstünde düzgün çalışmıyor
	  if (distance < 250) { //Uzaklık 250'nin altındaysa bu koşulu çalıştır
		  TIM3->CCR2 = 0; //Motora giden PWM sinyalinin 2. kanalının duty cycle'ını 0% yap
		  TIM3->CCR1 = 127; //Motora giden PWM sinyalinin 1. kanalının duty cycle'ını 50% yap
		  //Sadece kırmızı led çalışacak şekilde ledleri ayarla
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		  //Cursoru 2. satırın 6. Sütununa getirip "kirmizi" yaz
		  Lcd_cursor(&lcd, 1, 5);
		  Lcd_string(&lcd, "kirmizi");
	  }
	  else if (distance < 350) { //Uzaklık 250 ve 350 arasıysa bu koşulu çalıştır
		  TIM3->CCR2 = 0; //Motora giden PWM sinyallerinin duty cycle'larını 0% yap
		  TIM3->CCR1 = 0;
		  //Sadece sarı led çalışacak şekilde ledleri ayarla
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_8, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		  //Cursoru 2. satırın 6. Sütununa getirip "sari   " yaz (boşlukların sebebi önceden kirmizi yazıldıysa onun izi harflerini silmek)
		  Lcd_cursor(&lcd, 1, 5);
		  Lcd_string(&lcd, "sari   ");
	  }
	  else { //Uzaklık 450 üstüyse bu koşulu çalıştır
		  TIM3->CCR1 = 0; //Motora giden PWM sinyalinin 2. kanalının duty cycle'ını 0% yap
		  TIM3->CCR2 = 127; //Motora giden PWM sinyalinin 1. kanalının duty cycle'ını 50% yap
		  //Sadece yeşil led çalışacak şekilde ledleri ayarla
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7 | GPIO_PIN_6, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		  //Cursoru 2. satırın 6. Sütununa getirip "yesil  " yaz (boşlukların sebebi önceden kirmizi yazıldıysa onun izi harflerini silmek)
		  Lcd_cursor(&lcd, 1, 5);
		  Lcd_string(&lcd, "yesil  ");
	  }
	  HAL_Delay(100); //0.1 saniye sistemi beklet ki HC-SR04 sürekli çalışmasın
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LCD_EN_Pin|LCD_RS_Pin|LCD_D4_Pin
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 LCD_EN_Pin LCD_RS_Pin LCD_D4_Pin
                           LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LCD_EN_Pin|LCD_RS_Pin|LCD_D4_Pin
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

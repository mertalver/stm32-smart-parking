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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"

uint8_t gelen_buf[64];
int32_t dogru_cevap;
char dogru_cevap_buffer[64];
char soru[64];
volatile uint8_t timer_done = 0; // Timer tamamlanma bayrağı
volatile uint8_t countdown = 20;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
/* Kullanıcı performansını tutan Struct */
typedef struct {
    int dogru_sayisi;
    int yanlis_sayisi;
    int zaman_yetmedi;
} KullaniciPerformansi;

static KullaniciPerformansi performans = {0, 0, 0};

/* Zorluk Seviyesi Enum */
typedef enum {
    ZORLUK_KOLAY = 0,
    ZORLUK_ORTA,
    ZORLUK_ZOR
} ZorlukSeviyesi;

/* --- Prototipler --- */
uint32_t SimpleHash(uint32_t input);
void     InitializeRandomSeed(void);
void     CheckUserAnswer(uint8_t *data);

uint32_t ReadPotentiometer(void);
ZorlukSeviyesi GetDifficultyLevel(void);

void     SetCountdownByDifficulty(ZorlukSeviyesi zorluk);
void     GenerateRandomQuestion(ZorlukSeviyesi zorluk);
const char* GetDifficultyString(ZorlukSeviyesi zorluk);

/* --- Fonksiyonlar --- */

/**
  * @brief  Basit bir hashing fonksiyonu ile seed değerini karıştırır.
  * @param  input: Girdi değer
  * @retval Karma değer
  */
uint32_t GenerateAnalogRandomSeed(void) {
    uint32_t adcValue = 0;

    // ADC'yi başlat
    HAL_ADC_Start(&hadc1);

    // ADC ölçümünü bekle
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        adcValue = HAL_ADC_GetValue(&hadc1);  // ADC ölçüm sonucu
    }

    // ADC'yi durdur
    HAL_ADC_Stop(&hadc1);

    return adcValue;  // Rastgelelik kaynağı olarak ADC değeri
}

void InitializeRandomSeed(void) {
    uint32_t seed = GenerateAnalogRandomSeed();  // ADC'den rastgelelik
    seed ^= HAL_GetTick();  // Seed'i zamanla karıştır
    srand(seed);  // Rastgele sayı üreteci başlat
}

/**
  * @brief  Rastgele dört işlem sorusu üretir ve global soru değişkenine yazdırır.
  */
void GenerateRandomQuestion(ZorlukSeviyesi zorluk) {
    int sayi1, sayi2;
    char islem;

    switch (zorluk) {
        case ZORLUK_KOLAY:
            sayi1 = rand() % 50 + 1;   // 1–50 arası
            sayi2 = rand() % 50 + 1;   // 1–50 arası
            islem = (rand() % 2 == 0) ? '+' : '-'; // Toplama veya çıkarma
            break;

        case ZORLUK_ORTA:
            sayi1 = rand() % 100 + 1;  // 1–100 arası
            sayi2 = rand() % 100 + 1;  // 1–100 arası
            switch (rand() % 3) {      // Toplama, çıkarma, çarpma
                case 0: islem = '+'; break;
                case 1: islem = '-'; break;
                case 2: islem = '*'; break;
            }
            break;

        case ZORLUK_ZOR:
            sayi1 = rand() % 100 + 1;  // 1–100 arası
            sayi2 = rand() % 100 + 1;  // 1–100 arası
            switch (rand() % 4) {      // Toplama, çıkarma, çarpma, bölme
                case 0: islem = '+'; break;
                case 1: islem = '-'; break;
                case 2: islem = '*'; break;
                case 3:
                    islem = '/';
                    if (sayi2 == 0) sayi2 = 1; // Sıfıra bölünmeyi önle
                    break;
            }
            break;
    }

    // Doğru cevabı hesapla
    switch (islem) {
        case '+': dogru_cevap = sayi1 + sayi2; break;
        case '-': dogru_cevap = sayi1 - sayi2; break;
        case '*': dogru_cevap = sayi1 * sayi2; break;
        case '/': dogru_cevap = sayi1 / sayi2; break;
    }

    // Soruyu oluştur
    sprintf(soru, "Sorunuz: %d %c %d = ?\n", sayi1, islem, sayi2);
}

/**
  * @brief  Kullanıcının verdiği cevabı kontrol eder ve performans verisini günceller.
  * @param  data: Kullanıcıdan gelen cevap (string olarak)
  */
void CheckUserAnswer(uint8_t *data) {
    int kullanici_cevap = atoi((char *)data); // Kullanıcı cevabını int'e çevir

    if (kullanici_cevap == dogru_cevap) {
        performans.dogru_sayisi++;
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // Yeşil LED yak
        ssd1306_SetCursor(0, 12);
        ssd1306_WriteString("DOGRU", Font_7x10, White);
        ssd1306_UpdateScreen();
        HAL_Delay(2000);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    } else {
        performans.yanlis_sayisi++;
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // Kırmızı LED yak

        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("YANLIS", Font_7x10, White);

        ssd1306_SetCursor(0, 12);
        sprintf(dogru_cevap_buffer, "Dogru Cevap: %d", dogru_cevap);
        ssd1306_WriteString(dogru_cevap_buffer, Font_6x8, White);

        ssd1306_UpdateScreen();
        HAL_Delay(2000);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    }
}

/**
  * @brief  Potansiyometre değerini ADC üzerinden okur ve değeri döndürür.
  * @retval ADC değeri
  */
uint32_t ReadPotentiometer(void) {
    uint32_t adcValue = 0;

    // ADC'yi başlat
    HAL_ADC_Start(&hadc1);

    // Ölçüm tamamlanana kadar bekle
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        // Okunan değeri al
        adcValue = HAL_ADC_GetValue(&hadc1);
    }

    // ADC'yi durdur
    HAL_ADC_Stop(&hadc1);

    return adcValue;
}

/**
  * @brief  Potansiyometreden okunan değere göre zorluk seviyesini belirler.
  *         0-1365   : KOLAY
  *         1365-2730: ORTA
  *         2730-4095: ZOR
  * @retval ZorlukSeviyesi enum değeri
  */
ZorlukSeviyesi GetDifficultyLevel(void) {
    uint32_t adcValue = ReadPotentiometer();

    if (adcValue < 1365) {
        return ZORLUK_KOLAY;
    } else if (adcValue < 2730) {
        return ZORLUK_ORTA;
    } else {
        return ZORLUK_ZOR;
    }
}

/**
  * @brief  Verilen zorluk seviyesine göre geri sayım süresini ayarlar.
  * @param  zorluk: ZorlukSeviyesi
  */
void SetCountdownByDifficulty(ZorlukSeviyesi zorluk) {
    switch (zorluk) {
        case ZORLUK_KOLAY:
            countdown = 30;
            break;
        case ZORLUK_ORTA:
            countdown = 20;
            break;
        case ZORLUK_ZOR:
            countdown = 10;
            break;
        default:
            countdown = 20; // Varsayılan
            break;
    }
}

/**
  * @brief  Zorluk enum değerini string olarak döndürür.
  * @param  zorluk: ZorlukSeviyesi
  * @retval Zorluk seviyesini temsil eden string
  */
const char* GetDifficultyString(ZorlukSeviyesi zorluk) {
    switch (zorluk) {
        case ZORLUK_KOLAY:
            return "KOLAY";
        case ZORLUK_ORTA:
            return "ORTA";
        case ZORLUK_ZOR:
            return "ZOR";
        default:
            return "BILINMIYOR";
    }
}

/**
  * @brief  Timer kesmesi. Her kesmede geri sayım yapılır.
  * @param  htim: Timer handle
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
        if (countdown > 0) {
            countdown--;
        } else {
            timer_done = 1;
        }
    }
}

/* --- main--- */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USB_DEVICE_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
    MX_ADC1_Init();

    /* Timer'ı başlat (interrupt) */
    HAL_TIM_Base_Start_IT(&htim2);

    /* Rastgele sayı üretimi için seed oluştur */
    InitializeRandomSeed();

    char countdown_message[20];
    ssd1306_Init();

    /* Uygulama ilk açıldığında zorluk seviyesini göster */
    ZorlukSeviyesi zorluk = GetDifficultyLevel();
    SetCountdownByDifficulty(zorluk);

    const char* zorluk_seviyesi_str = GetDifficultyString(zorluk);
    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString("Zorluk: ", Font_7x10, White);
    ssd1306_WriteString(zorluk_seviyesi_str, Font_7x10, White);
    ssd1306_UpdateScreen();

    while (1) {
        /* Yeni Soru komutu geldiğinde */
        if (strcmp(gelen_buf, "Yeni Soru") == 0) {
            ssd1306_Init();

            /* Her yeni soruda zorluk tekrar okunup countdown yeniden ayarlanabilir */
            ZorlukSeviyesi zorluk = GetDifficultyLevel();
            SetCountdownByDifficulty(zorluk);

            /* Soru üret ve gönder */
            GenerateRandomQuestion(zorluk);
            CDC_Transmit_FS((uint8_t *)soru, strlen(soru));

            /* Kalan süre mesajı */
            sprintf(countdown_message, "Kalan sure: %lu\n", (unsigned long)countdown);
            CDC_Transmit_FS((uint8_t *)countdown_message, strlen(countdown_message));

            /* Gelen buffer temizle */
            memset(gelen_buf, 0, sizeof(gelen_buf));

            timer_done = 0;
            uint32_t old_countdown = (uint32_t)(-1);

            /* Timer bitene veya cevap gelene kadar bekle */
            while (!timer_done) {
                if (gelen_buf[0] != '\0') {
                    /* Kullanıcı cevabı gelmiş, kontrol et */
                    CheckUserAnswer((uint8_t*)gelen_buf);
                    break;
                }
                /* Her 1 saniyede bir countdown değiştiğinde süreyi gönder */
                if (old_countdown != countdown) {
                    old_countdown = countdown;
                    sprintf(countdown_message, "Kalan sure: %lu\n", (unsigned long)countdown);
                    CDC_Transmit_FS((uint8_t *)countdown_message, strlen(countdown_message));
                }
            }

            /* Süre dolmuş ama cevap gelmemiş ise */
            if (timer_done && gelen_buf[0] == '\0') {
                performans.zaman_yetmedi++;
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);

                ssd1306_SetCursor(0, 0);
                ssd1306_WriteString("SURENIZ BITTI", Font_7x10, White);

                ssd1306_SetCursor(0, 12);
                sprintf(dogru_cevap_buffer, "Dogru Cevap: %d", dogru_cevap);
                ssd1306_WriteString(dogru_cevap_buffer, Font_6x8, White);

                ssd1306_UpdateScreen();
                HAL_Delay(2000);
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
            }
        }

        /* Sonucu Gör komutu geldiğinde */
        if (strcmp(gelen_buf, "Sonucu Gor") == 0) {
            memset(gelen_buf, 0, sizeof(gelen_buf));
            ssd1306_Init();

            char buffer[30];
            ssd1306_SetCursor(0, 0);
            ssd1306_WriteString("Sonuclar:", Font_6x8, White);

            ssd1306_SetCursor(0, 12);
            sprintf(buffer, "Dogru: %d Yanlis: %d", performans.dogru_sayisi, performans.yanlis_sayisi);
            ssd1306_WriteString(buffer, Font_6x8, White);

            ssd1306_SetCursor(0, 24);
            sprintf(buffer, "Zaman Yetmedi: %d", performans.zaman_yetmedi);
            ssd1306_WriteString(buffer, Font_6x8, White);

            ssd1306_UpdateScreen();
        }
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

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
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4800-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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


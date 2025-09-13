/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include <string.h>
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "usbd_cdc_if.h"
#include "queue.h"
#include "semphr.h"

SemaphoreHandle_t oledMutex;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define TOTAL_SPOTS 15

typedef struct {
  char id[4];        // A1, B2 vs.
  uint8_t status;    // 0 boş, 1 rezerve
  uint16_t timeLeft; // dakika
  float price;       // fiyat (TL)
} ParkingSpot;

ParkingSpot spots[TOTAL_SPOTS];

typedef struct {
    char line1[22]; // OLED satır 1 (en fazla 21 karakter)
    char line2[22]; // OLED satır 2
    uint16_t duration_ms; // mesajı kaç ms boyunca gösterelim
} OledMessage;

QueueHandle_t oledQueue;
QueueHandle_t commandQueue;

// İstatistik değişkenleri
uint32_t totalReservations = 0;
uint32_t totalMinutesParked = 0;
uint32_t completedReservations = 0;
uint8_t occupiedSpots = 0;

char rxBuffer[64];
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Gösterilecek sayı (örneğin boş yer sayısı)
uint8_t displayValue = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

/* Definitions for UsbCommTask */
osThreadId_t UsbCommTaskHandle;
const osThreadAttr_t UsbCommTask_attributes = {
  .name = "UsbCommTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ParkingManagerT */
osThreadId_t ParkingManagerTHandle;
const osThreadAttr_t ParkingManagerT_attributes = {
  .name = "ParkingManagerT",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
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
void StartUsbCommTask(void *argument);
void StartParkingManagerTask(void *argument);
void StartDisplayTask(void *argument);

/* USER CODE BEGIN PFP */

extern uint8_t segmentCodes[10];
extern GPIO_TypeDef* segmentPorts[7];
extern uint16_t segmentPins[7];
extern GPIO_TypeDef* digitPorts[4];
extern uint16_t digitPins[4];

void segment_init(void);
void setSegment(uint8_t index, GPIO_PinState state);
void setDigit(uint8_t index, GPIO_PinState state);
void update7Segment(uint8_t val);
void segment_refresh_loop(uint8_t val);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void initParkingSpots() {
  int index = 0;
  for (char block = 'A'; block <= 'C'; block++) {
    for (int num = 1; num <= 5; num++) {
      sprintf(spots[index].id, "%c%d", block, num);
      spots[index].status = 0;
      spots[index].timeLeft = 0;
      spots[index].price = 0.0;
      index++;
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  oledMutex = xSemaphoreCreateMutex();
  if (oledMutex == NULL) {
      Error_Handler();
  }

  commandQueue = xQueueCreate(20, sizeof(char[64]));
  if (commandQueue == NULL) {
	  Error_Handler();
  }

  oledQueue = xQueueCreate(20, sizeof(OledMessage));
  if (oledQueue == NULL) {
      Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  initParkingSpots();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UsbCommTask */
  UsbCommTaskHandle = osThreadNew(StartUsbCommTask, NULL, &UsbCommTask_attributes);

  /* creation of ParkingManagerT */
  ParkingManagerTHandle = osThreadNew(StartParkingManagerTask, NULL, &ParkingManagerT_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  OledMessage testMsg = { "Test", "Calisiyor", 3000 };
  xQueueSend(oledQueue, &testMsg, 0);

  /* Start scheduler */
  osKernelStart();
  HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET); // KIRMIZI LED

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

/* USER CODE BEGIN Header_StartUsbCommTask */
/**
  * @brief  Function implementing the UsbCommTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUsbCommTask */
uint8_t commandReady = 0;

void StartUsbCommTask(void *argument) {
  MX_USB_DEVICE_Init();
  while (1) {
	  if (commandReady) {
	      commandReady = 0;

	      OledMessage msg;
	      snprintf(msg.line1, sizeof(msg.line1), "USB OK");
	      snprintf(msg.line2, sizeof(msg.line2), "Komut Alindi");
	      msg.duration_ms = 2000;
	      xQueueSend(oledQueue, &msg, 0);

	      char temp[64];
	      strcpy(temp, rxBuffer);

	      // Queue'ya gönder
	      xQueueSend(commandQueue, &temp, pdMS_TO_TICKS(100));
	  }
    osDelay(10);
  }
}

/* USER CODE BEGIN Header_StartParkingManagerTask */
/**
* @brief Function implementing the ParkingManagerT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartParkingManagerTask */
void StartParkingManagerTask(void *argument) {
    char cmd[64];
    TickType_t lastCheck = xTaskGetTickCount();
    TickType_t lastStatsUpdate = xTaskGetTickCount();
    uint8_t reminderSent[TOTAL_SPOTS] = {0};

    while (1) {
        // Queue'dan komut bekle
        if (xQueueReceive(commandQueue, &cmd, pdMS_TO_TICKS(100)) == pdPASS) {

            if (strncmp(cmd, "RESERVE", 7) == 0) {
                char spot[4] = {0};
                int duration = 0;
                float price = 0.0;

                int parsed = sscanf(cmd, "RESERVE;%3[^;];%d;%f", spot, &duration, &price);
                if (parsed >= 2) {  // En az spot ve duration bilgisi varsa
                    spot[strcspn(spot, "\r\n")] = 0;
                    spot[3] = '\0';

                    for (int i = 0; i < TOTAL_SPOTS; i++) {
                        if (strcmp(spots[i].id, spot) == 0) {
                            spots[i].status = 1;
                            spots[i].timeLeft = duration;
                            
                            // Fiyat bilgisi varsa kaydet
                            if (parsed == 3) {
                                spots[i].price = price;
                            }
                            
                            // İstatistikleri güncelle
                            totalReservations++;
                            occupiedSpots++;

                            OledMessage reserveMsg;
                            snprintf(reserveMsg.line1, sizeof(reserveMsg.line1), "Rezerve: %s", spots[i].id);
                            
                            if (parsed == 3) {
                                snprintf(reserveMsg.line2, sizeof(reserveMsg.line2), "%d dk, %.1f TL", duration, price);
                            } else {
                                snprintf(reserveMsg.line2, sizeof(reserveMsg.line2), "Sure: %d dk", duration);
                            }
                            
                            reserveMsg.duration_ms = 3000;
                            xQueueSend(oledQueue, &reserveMsg, pdMS_TO_TICKS(100));
                            
                            reminderSent[i] = 0; // Hatırlatma sıfırla
                            break;
                        }
                    }
                }
                else {
                    OledMessage err;
                    snprintf(err.line1, sizeof(err.line1), "Komut hatali");
                    snprintf(err.line2, sizeof(err.line2), "%.20s", cmd);
                    err.duration_ms = 3000;
                    xQueueSend(oledQueue, &err, pdMS_TO_TICKS(100));
                }
            }
            else if (strncmp(cmd, "CANCEL", 6) == 0) {
                char spot[4] = {0};
                sscanf(cmd, "CANCEL;%3s", spot);
                spot[strcspn(spot, "\r\n")] = 0;

                for (int i = 0; i < TOTAL_SPOTS; i++) {
                    if (strcmp(spots[i].id, spot) == 0 && spots[i].status == 1) {
                        uint16_t kalanSure = spots[i].timeLeft;
                        float spotPrice = spots[i].price;

                        // İstatistikleri güncelle
                        occupiedSpots--;
                        totalMinutesParked += (duration - kalanSure);
                        completedReservations++;

                        spots[i].status = 0;
                        spots[i].timeLeft = 0;
                        spots[i].price = 0.0;

                        OledMessage cancelMsg;
                        snprintf(cancelMsg.line1, sizeof(cancelMsg.line1), "Iptal: %s", spots[i].id);
                        snprintf(cancelMsg.line2, sizeof(cancelMsg.line2), "Kalan: %d dk", kalanSure);
                        cancelMsg.duration_ms = 3000;
                        xQueueSend(oledQueue, &cancelMsg, pdMS_TO_TICKS(100));

                        char usbMsg[64];
                        snprintf(usbMsg, sizeof(usbMsg), "EXPIRED;%s\r\n", spots[i].id);
                        CDC_Transmit_FS((uint8_t*)usbMsg, strlen(usbMsg));
                        break;
                    }
                }
            }
            else if (strncmp(cmd, "GET_STATS", 9) == 0) {
                // İstatistikleri gönder
                char statsMsg[128];
                float avgDuration = 0;
                
                if (completedReservations > 0) {
                    avgDuration = (float)totalMinutesParked / completedReservations;
                }
                
                snprintf(statsMsg, sizeof(statsMsg), 
                         "STATS;TOTAL_RESERVATIONS:%lu;COMPLETED:%lu;AVG_DURATION:%.1f;OCCUPIED:%d\r\n", 
                         totalReservations, completedReservations, avgDuration, occupiedSpots);
                         
                CDC_Transmit_FS((uint8_t*)statsMsg, strlen(statsMsg));
                
                // OLED'e de göster
                OledMessage statsOledMsg;
                snprintf(statsOledMsg.line1, sizeof(statsOledMsg.line1), "Stats sent");
                snprintf(statsOledMsg.line2, sizeof(statsOledMsg.line2), "Total Res: %lu", totalReservations);
                statsOledMsg.duration_ms = 2000;
                xQueueSend(oledQueue, &statsOledMsg, pdMS_TO_TICKS(100));
            }
        }

        TickType_t now = xTaskGetTickCount();
        
        // Dakikalık kontrol - süre azaltma
        if ((now - lastCheck) >= pdMS_TO_TICKS(60000)) {
            lastCheck = now;

            HAL_GPIO_TogglePin(GPIOD, LD3_Pin);

            for (int i = 0; i < TOTAL_SPOTS; i++) {
                if (spots[i].status == 1 && spots[i].timeLeft > 0) {
                    spots[i].timeLeft--;
                    
                    // Sürenin %80'i geçmiş ama hatırlatma gönderilmemişse
                    uint16_t reminderThreshold = spots[i].timeLeft / 5; // Kalan sürenin 1/5'i (yani %20)
                    if (reminderThreshold <= 2 && !reminderSent[i]) {
                        reminderSent[i] = 1;
                        
                        // Reminder bilgisini gönder
                        char reminderMsg[64];
                        snprintf(reminderMsg, sizeof(reminderMsg), "REMINDER;%s;%d\r\n", 
                                 spots[i].id, spots[i].timeLeft);
                        CDC_Transmit_FS((uint8_t*)reminderMsg, strlen(reminderMsg));
                        
                        // OLED'e de göster
                        OledMessage reminderOledMsg;
                        snprintf(reminderOledMsg.line1, sizeof(reminderOledMsg.line1), "%s hatirlatma", spots[i].id);
                        snprintf(reminderOledMsg.line2, sizeof(reminderOledMsg.line2), "Kalan: %d dk", spots[i].timeLeft);
                        reminderOledMsg.duration_ms = 3000;
                        xQueueSend(oledQueue, &reminderOledMsg, pdMS_TO_TICKS(100));
                    }

                    if (spots[i].timeLeft == 0) {
                        // İstatistikleri güncelle
                        totalMinutesParked += spots[i].timeLeft;
                        completedReservations++;
                        occupiedSpots--;
                        
                        spots[i].status = 0;

                        OledMessage expiredMsg;
                        snprintf(expiredMsg.line1, sizeof(expiredMsg.line1), "Sure bitti!");
                        snprintf(expiredMsg.line2, sizeof(expiredMsg.line2), "%s bos", spots[i].id);
                        expiredMsg.duration_ms = 3000;
                        xQueueSend(oledQueue, &expiredMsg, pdMS_TO_TICKS(100));

                        char usbMsg[64];
                        snprintf(usbMsg, sizeof(usbMsg), "EXPIRED;%s\r\n", spots[i].id);
                        CDC_Transmit_FS((uint8_t*)usbMsg, strlen(usbMsg));
                    }
                }
            }
        }
        
        // 30 Saniye aralıklarla istatistik güncelleme
        if ((now - lastStatsUpdate) >= pdMS_TO_TICKS(30000)) {
            lastStatsUpdate = now;
            
            // Boş park yeri sayısını say
            uint8_t emptyCount = 0;
            for (int i = 0; i < TOTAL_SPOTS; i++) {
                if (spots[i].status == 0) {
                    emptyCount++;
                }
            }
            
            // 7-segment display için
            displayValue = emptyCount;
            
            // İstatistik bilgilerini OLED'e göster
            OledMessage statsMsg;
            snprintf(statsMsg.line1, sizeof(statsMsg.line1), "Bos: %d / Dolu: %d", 
                     emptyCount, TOTAL_SPOTS - emptyCount);
            snprintf(statsMsg.line2, sizeof(statsMsg.line2), "Toplam Rez: %lu", totalReservations);
            statsMsg.duration_ms = 3000;
            xQueueSend(oledQueue, &statsMsg, pdMS_TO_TICKS(100));
        }
        
        vTaskDelay(1);
    }
}



/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument) {
	ssd1306_Init();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);

    OledMessage msg;
    uint32_t msgEndTime = 0;

    while (1) {
        // Yeni mesaj varsa al
        if (xQueueReceive(oledQueue, &msg, 0) == pdPASS) {
            if (xSemaphoreTake(oledMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                ssd1306_Fill(Black);
                ssd1306_SetCursor(0, 0);
                ssd1306_WriteString(msg.line1, Font_7x10, White);
                ssd1306_SetCursor(0, 16);
                ssd1306_WriteString(msg.line2, Font_7x10, White);
                ssd1306_UpdateScreen();
                xSemaphoreGive(oledMutex);
                msgEndTime = HAL_GetTick() + msg.duration_ms;
            }
        }

        // Süre dolduysa ekranı temizle
        if (msgEndTime > 0 && HAL_GetTick() > msgEndTime) {
            if (xSemaphoreTake(oledMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                ssd1306_Fill(Black);
                ssd1306_UpdateScreen();
                xSemaphoreGive(oledMutex);
                msgEndTime = 0;
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}




/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */

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

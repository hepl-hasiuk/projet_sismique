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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "lwip/tcp.h"
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRESENCE_PORT 12345 // même port que ton script Python
#define WINDOW_SIZE 100     // 💡 Taille de la fenêtre pour le calcul de la moyenne (1s à 100Hz)
#define SEISMIC_THRESHOLD 300.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId logMessageTaskHandle;
osThreadId clientTaskHandle;
osThreadId serverTaskHandle;
osThreadId heartBeatTaskHandle;
osMessageQId messageQueueHandle;
osMutexId uartMutexHandle;
/* USER CODE BEGIN PV */


osThreadId presenceTaskHandle; // handle de la tâche de présence
uint16_t adc_raw[3];           // 💡 Buffer rempli automatiquement par le DMA (X, Y, Z)

//seismic part
osThreadId seismicTaskHandle;
// 💡 Variables pour le lissage (filtre passe-bas)
float avg_x = 0, avg_y = 0, avg_z = 0;
// 💡 Variables finales calculées (énergie du signal)
float rms_x = 0, rms_y = 0, rms_z = 0;
// 💡 Historique des valeurs lissées pour le calcul RMS
float buf_x[WINDOW_SIZE];
float buf_y[WINDOW_SIZE];
float buf_z[WINDOW_SIZE];
//uint16_t idx = 0; // 💡 Index pour parcourir le tableau circulaire

// Commandes SPI standards FRAM
#define RTC_ADDR 0xD0
#define FRAM_WREN  0x06     // Write Enable (Autoriser écriture)
#define FRAM_WRITE 0x02     // Write Data
#define FRAM_READ  0x03     // Read Data


// Structure pour stocker un événement (Taille = 8 octets)
typedef struct {
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t reserved; // Juste pour aligner la mémoire
    float intensity;  // La valeur RMS max
} SeismicEvent;

#define FRAM_EVENT_ADDR 0x10


// --- ETAPE 3 : INTELLIGENCE COLLECTIVE ---
volatile uint8_t my_alert_status = 0;       // 0=Calme, 1=Je tremble
volatile uint8_t neighbor_alert_status = 0; // 0=Voisin Calme, 1=Voisin tremble

float neighbor_peaks[10];
uint8_t neighbor_idx = 0;
// --- ETAPE 3 : CONFIG NTP ---
#define NTP_SERVER_IP "216.239.35.0" // IP d'un serveur be.pool.ntp.org // utilisrer le serceur google
#define NTP_PORT 123
#define NTP_MSG_LEN 48
// Différence entre 1900 (Epoch NTP) et 1970 (Epoch Unix) en secondes
#define NTP_TIMESTAMP_DELTA 2208988800u

#define TIMEZONE_OFFSET 1
volatile uint8_t ntp_synced = 0; // Drapeau : 0 = Pas à l'heure, 1 = À l'heure

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);
void LogMessageTask(void const * argument);
void StartClientTask(void const * argument);
void StartServerTask(void const * argument);
void StartHeartBeatTask(void const * argument);

/* USER CODE BEGIN PFP */
void StartPresenceTask(void const * argument);
void send_presence_broadcast(void);
void StartSeismicTask(void const * argument);
/*server*/
void tcp_server_init(void);
err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

void send_data_request_tcp(const char *ip);
err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);
err_t tcp_client_recv_response(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

osSemaphoreId adcReadySemHandle;
osMutexId dataMutexHandle;

/* Fonctions RTC (Gestion Heure) */
uint8_t decToBcd(int val);
int bcdToDec(uint8_t val);
void RTC_SetTime(uint8_t hour, uint8_t min, uint8_t sec);
void RTC_GetTime(uint8_t *hour, uint8_t *min, uint8_t *sec);

/* Fonctions FRAM (Gestion Mémoire) */
void FRAM_Write(uint32_t addr, uint8_t *pData, uint16_t size);
void FRAM_Read(uint32_t addr, uint8_t *pData, uint16_t size);

void ntp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  // 💡 Démarrage du Timer qui cadence l'ADC (pour avoir 100Hz précis)
  HAL_TIM_Base_Start(&htim2);
  // 💡 Démarrage de l'ADC en mode DMA Circulaire.
  // Le CPU n'a rien à faire, le DMA remplit le tableau 'adc_raw' tout seul en arrière-plan.
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, 3);
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of uartMutex */
  osMutexDef(uartMutex);
  uartMutexHandle = osMutexCreate(osMutex(uartMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  osMutexDef(dataMutex);
  dataMutexHandle = osMutexCreate(osMutex(dataMutex));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */


  /* definition and creation of adcReadySem */
  osSemaphoreDef(adcReadySem);
  adcReadySemHandle = osSemaphoreCreate(osSemaphore(adcReadySem), 1);
  // On le prend une fois au début pour qu'il soit "vide" au démarrage
  osSemaphoreWait(adcReadySemHandle, 0);


  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of messageQueue */
  osMessageQDef(messageQueue, 16, uint32_t);
  messageQueueHandle = osMessageCreate(osMessageQ(messageQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 2048);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of logMessageTask */
  osThreadDef(logMessageTask, LogMessageTask, osPriorityNormal, 0, 512);
  logMessageTaskHandle = osThreadCreate(osThread(logMessageTask), NULL);

  /* definition and creation of clientTask */
  osThreadDef(clientTask, StartClientTask, osPriorityBelowNormal, 0, 1024);
  clientTaskHandle = osThreadCreate(osThread(clientTask), NULL);

  /* definition and creation of serverTask */
  osThreadDef(serverTask, StartServerTask, osPriorityBelowNormal, 0, 1024);
  serverTaskHandle = osThreadCreate(osThread(serverTask), NULL);

  /* definition and creation of heartBeatTask */
  osThreadDef(heartBeatTask, StartHeartBeatTask, osPriorityIdle, 0, 256);
  heartBeatTaskHandle = osThreadCreate(osThread(heartBeatTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* definition and creation of presenceTask */
  // 💡 Tâche Présence : Envoie un message UDP "Je suis là" à tout le monde
  osThreadDef(presenceTask, StartPresenceTask, osPriorityBelowNormal, 0, 512);
  presenceTaskHandle = osThreadCreate(osThread(presenceTask), NULL);
  /* add threads, ... */


  /* definition and creation of seismicTask */
  // 💡 Tâche Sismique : Priorité ÉLEVÉE car calcul critique en temps réel
  osThreadDef(seismicTask, StartSeismicTask, osPriorityAboveNormal, 0, 2048);
  seismicTaskHandle = osThreadCreate(osThread(seismicTask), NULL);


  /* 🚨 IMPORTANT : tout suspendre au début */
  // 💡 Au démarrage, on met tout en pause. Seule la tâche par défaut tourne
  // pour attendre l'appui sur le bouton bleu.
  osThreadSuspend(heartBeatTaskHandle);
  osThreadSuspend(presenceTaskHandle);
  osThreadSuspend(logMessageTaskHandle);
  osThreadSuspend(seismicTaskHandle);
  osThreadSuspend(clientTaskHandle);
  /* USER CODE END RTOS_THREADS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x40000A0B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 2399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FRAM_CS_Pin */
  GPIO_InitStruct.Pin = FRAM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FRAM_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief Tâche pour la découverte réseau (Protocole de présence)
  * @note  Envoie périodiquement un broadcast UDP pour dire "Je suis là"
  */
void StartPresenceTask(void const * argument)
{
  /* On attend que LwIP soit initialisé */
  extern struct netif gnetif;
  // 💡 Boucle d'attente : tant que le câble n'est pas branché ou l'IP non reçue
  while (!netif_is_up(&gnetif))
  {
	  osDelay(100);
  }

  for(;;)
  {
    send_presence_broadcast(); // envoi du JSON en broadcast
    osDelay(10000); // toutes les 10 secondes
  }
}

/**
  * @brief Fonction d'envoi du message UDP Broadcast
  * @note  Crée un socket UDP temporaire, envoie le JSON, puis ferme le socket.
  */
void send_presence_broadcast(void)
{
  struct udp_pcb *pcb;
  struct pbuf *p;
  ip_addr_t dest_ip;
  err_t err;

  // 💡 Création d'un "Protocol Control Block" (socket léger LwIP)
  pcb = udp_new();
  if (!pcb)
  {
    return;
  }

  // 💡 Option SOF_BROADCAST indispensable pour envoyer à .255
  pcb->so_options |= SOF_BROADCAST;

  // Broadcast global (ça marche bien avec ton PC en 169.254.x.x)
  ipaddr_aton("192.168.129.255", &dest_ip);

  // Bind sur n'importe quelle IP / n'importe quel port
  udp_bind(pcb, IP_ADDR_ANY, 0);

  // Construction du JSON
  // Construction du JSON
    char json[256];

    // 💡 CORRECTION : Récupération dynamique de l'IP réelle via LwIP
    extern struct netif gnetif;
    char *device_ip = ipaddr_ntoa(&gnetif.ip_addr);

    // 💡 snprintf formate le message JSON avec l'IP dynamique
    snprintf(json, sizeof(json),
        "{ \"type\": \"presence\", \"id\": \"nucleo-8\", \"ip\": \"%s\", \"timestamp\": \"2025-10-02T08:20:00Z\" }",
        device_ip);

  // 💡 Allocation d'un buffer LwIP (pbuf) en RAM
  p = pbuf_alloc(PBUF_TRANSPORT, strlen(json), PBUF_RAM);
  if (!p)
  {
    udp_remove(pcb);
    return;
  }

  // 💡 Copie des données dans le buffer
  memcpy(p->payload, json, strlen(json));
  // 💡 Envoi effectif du paquet UDP
  err = udp_sendto(pcb, p, &dest_ip, PRESENCE_PORT);

  // 💡 TRES IMPORTANT : Libération mémoire pour éviter les fuites (Memory Leak)
  pbuf_free(p);
  udp_remove(pcb);

  //  éventuellement afficher err via UART si tu veux debug
}

// 💡 Callback appelée automatiquement par le hardware quand le DMA a fini de remplir le buffer ADC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1)
	  {
	    // On libère le sémaphore pour réveiller la tâche sismique
	    osSemaphoreRelease(adcReadySemHandle);
	    // Toggle LED pour debug visuel (optionnel)
	    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  }
}

/**
  * @brief Tâche de traitement du signal sismique
  * @note  Lit les valeurs brutes, filtre (moyenne) et calcule l'énergie (RMS)
  */
void StartSeismicTask(void const * argument) //lle inclut une "mémoire" de 3 secondes.
{
    memset(buf_x,0,sizeof(buf_x)); memset(buf_y,0,sizeof(buf_y)); memset(buf_z,0,sizeof(buf_z));
    int buffer_idx = 0;

    // Variables de temps
    uint32_t last_tremor_time = 0; // Moment du dernier RMS > 300
    uint32_t last_print_time = 0;  // Pour éviter le spam UART
    uint32_t last_save_time = 0;   // Pour l'écriture FRAM

    char msg[100];

    for(;;) {
        osSemaphoreWait(adcReadySemHandle, osWaitForever);

        // --- 1. Calcul RMS (inchangé) ---
        float raw_x = adc_raw[0], raw_y = adc_raw[1], raw_z = adc_raw[2];
        buf_x[buffer_idx]=raw_x; buf_y[buffer_idx]=raw_y; buf_z[buffer_idx]=raw_z;

        float mean_x=0, mean_y=0, mean_z=0;
        for(int i=0; i<WINDOW_SIZE; i++) { mean_x+=buf_x[i]; mean_y+=buf_y[i]; mean_z+=buf_z[i]; }
        mean_x/=WINDOW_SIZE; mean_y/=WINDOW_SIZE; mean_z/=WINDOW_SIZE;

        float var_x=0, var_y=0, var_z=0;
        for(int i=0; i<WINDOW_SIZE; i++) {
             var_x+=powf(buf_x[i]-mean_x,2); var_y+=powf(buf_y[i]-mean_y,2); var_z+=powf(buf_z[i]-mean_z,2);
        }
        float calc_rms_x = sqrtf(var_x/WINDOW_SIZE);
        float calc_rms_y = sqrtf(var_y/WINDOW_SIZE);
        float calc_rms_z = sqrtf(var_z/WINDOW_SIZE);

        // Mise à jour variables globales protégées
        osMutexWait(dataMutexHandle, osWaitForever);
        rms_x = calc_rms_x; rms_y = calc_rms_y; rms_z = calc_rms_z;
        osMutexRelease(dataMutexHandle);


        // --- 2. Logique de Détection Améliorée ---

        // Est-ce qu'on tremble MAINTENANT ?
        if (calc_rms_x > SEISMIC_THRESHOLD || calc_rms_y > SEISMIC_THRESHOLD || calc_rms_z > SEISMIC_THRESHOLD) {

            my_alert_status = 1; // 🚨 ON PASSE EN ALERTE
            last_tremor_time = HAL_GetTick(); // On remet le chrono à zéro

            // Anti-Spam UART : On affiche seulement toutes les 500ms
            if (HAL_GetTick() - last_print_time > 500) {
                float max_val = (calc_rms_x > calc_rms_y) ? calc_rms_x : calc_rms_y;
                sprintf(msg, "⚠️ TREMBLEMENT LOCAL (RMS: %.0f)\r\n", max_val);
                //affichage heure durant lla secousse
                uint8_t h, m, s;

                RTC_GetTime(&h, &m, &s);
                sprintf(msg, "[%02d:%02d:%02d] ⚠️ TREMBLEMENT LOCAL (RMS: %.0f)\r\n", h, m, s, max_val);
                HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
                last_print_time = HAL_GetTick();
            }

            // Sauvegarde FRAM (toutes les 2s max)
            if (HAL_GetTick() - last_save_time > 2000) {
                float max_val = calc_rms_x;
                if(calc_rms_y > max_val) max_val = calc_rms_y;
                if(calc_rms_z > max_val) max_val = calc_rms_z;

                uint8_t h, m, s; RTC_GetTime(&h, &m, &s);
                SeismicEvent evt; evt.hour=h; evt.min=m; evt.sec=s; evt.intensity=max_val;
                FRAM_Write(FRAM_EVENT_ADDR, (uint8_t*)&evt, sizeof(SeismicEvent));
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET); // LED Rouge Fixe
                last_save_time = HAL_GetTick();
            }
        }
        else {
            // C'est calme maintenant.
            // MAIS on garde l'alerte active pendant 3 secondes après la dernière secousse !
            if (HAL_GetTick() - last_tremor_time > 3000) {
                if (my_alert_status == 1) {
                     // Fin de l'alerte
                     my_alert_status = 0;
                     HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
                     HAL_UART_Transmit(&huart3, (uint8_t*)"✅ Fin de l'alerte locale.\r\n", 30, 100);
                }
            }
        }

        // --- 3. Consensus (Alerte Collective) ---
        // Si JE tremble (ou j'ai tremblé il y a <3s) ET le VOISIN tremble
        if (my_alert_status == 1 && neighbor_alert_status == 1) {
             HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin); // Clignotement panique
        }

        buffer_idx = (buffer_idx + 1) % WINDOW_SIZE;
    }
}

static struct tcp_pcb *server_pcb;

/**
  * @brief Initialisation du serveur TCP
  * @note  Ouvre le port 12345 en écoute
  */
void tcp_server_init(void)
{
    server_pcb = tcp_new();
    if (server_pcb == NULL) return;

    // 💡 Bind : on attache le PCB au port 12345
    tcp_bind(server_pcb, IP_ADDR_ANY, 12345);
    // 💡 Listen : on passe en mode écoute
    server_pcb = tcp_listen(server_pcb);

    // 💡 Accept : on définit quelle fonction appeler quand un client se connecte
    tcp_accept(server_pcb, tcp_server_accept);
}

// 💡 Callback appelée quand un nouveau client se connecte au serveur
err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    char msg[80];
    sprintf(msg, "📥 Client connecté depuis %s\r\n", ipaddr_ntoa(&newpcb->remote_ip));
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // 💡 On définit la fonction de réception pour cette nouvelle connexion
    tcp_recv(newpcb, tcp_server_recv);

    return ERR_OK;



}

// 💡 Callback appelée quand des données arrivent sur le serveur
err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (err!=ERR_OK || p==NULL) { if(p) pbuf_free(p); tcp_close(tpcb); return ERR_OK; }

    char buffer[256];
    uint16_t len = (p->len < sizeof(buffer)-1)?p->len:sizeof(buffer)-1;
    memcpy(buffer, p->payload, len); buffer[len]=0;
    tcp_recved(tpcb, p->tot_len);

    if (strstr(buffer, "\"data_request\"")) {
        float tx_x, tx_y, tx_z;
        osMutexWait(dataMutexHandle, osWaitForever);
        tx_x = rms_x; tx_y = rms_y; tx_z = rms_z;
        osMutexRelease(dataMutexHandle);

        char response[300];
        // Format JSON conforme PDF avec statut dynamique
        snprintf(response, sizeof(response),
                "{ \"type\": \"data_response\", \"id\": \"nucleo-8\", "
                "\"timestamp\": \"2025-10-02T...\", "
                "\"acceleration\": {\"x\": %.2f, \"y\": %.2f, \"z\": %.2f}, "
                "\"status\": \"%s\" }",
                tx_x, tx_y, tx_z, my_alert_status ? "alert" : "normal");

        tcp_write(tpcb, response, strlen(response), TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);
        tcp_close(tpcb);
    }
    pbuf_free(p);
    return ERR_OK;
}

/**
  * @brief Fonction Client TCP : Initie une connexion vers une IP
  */
void send_data_request_tcp(const char *ip)
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) {
        HAL_UART_Transmit(&huart3, (uint8_t*)"❌ tcp_new FAILED\r\n", 20, HAL_MAX_DELAY);
        return;
    }

    ip_addr_t dest_ip;
    ipaddr_aton(ip, &dest_ip);

    char msg[80];
    snprintf(msg, sizeof(msg), "🔵 Connexion vers %s...\r\n", ip);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // 💡 Tentative de connexion sur le port 12345
    // Si succès -> tcp_client_connected sera appelée
    err_t err = tcp_connect(pcb, &dest_ip, 12345, tcp_client_connected);

    if (err != ERR_OK) {
        snprintf(msg, sizeof(msg),
                 "❌ tcp_connect ERROR = %d\r\n", err);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        tcp_abort(pcb); // Annule le PCB en cas d'erreur
    }
}


// 💡 Callback appelée quand la connexion client est réussie
err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    if (err != ERR_OK) {
        tcp_close(tpcb);
        return err;
    }

    // 💡 Etape 2 : On définit la fonction qui va écouter la RÉPONSE du serveur
    tcp_recv(tpcb, tcp_client_recv_response);

    // 💡 Construction de la REQUÊTE JSON (Conforme PDF [cite: 110-116])
    char sendbuf[256];
    int len = snprintf(sendbuf, sizeof(sendbuf),
        "{ \"type\": \"data_request\", \"from\": \"nucleo-8\", \"to\": \"broadcast\", \"timestamp\": \"2025-10-02T08:00:00Z\" }");

    // Envoi
    tcp_write(tpcb, sendbuf, len, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    // ⚠️ IMPORTANT : On ne ferme PAS la connexion (tcp_close) ici !
    // On attend que le serveur réponde. C'est tcp_client_recv_response qui fermera.

    return ERR_OK;
}

err_t tcp_client_recv_response(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (err!=ERR_OK || p==NULL) { if(p) pbuf_free(p); tcp_close(tpcb); return ERR_OK; }

    char buffer[512];
    uint16_t len = (p->len < sizeof(buffer)-1)?p->len:sizeof(buffer)-1;
    memcpy(buffer, p->payload, len); buffer[len]=0;
    tcp_recved(tpcb, p->tot_len);

    // Analyse Voisin
    if (strstr(buffer, "\"status\": \"alert\"")) {
        neighbor_alert_status = 1;
        HAL_UART_Transmit(&huart3, (uint8_t*)"⚠️ VOISIN EN ALERTE !\r\n", 24, 100);
        neighbor_peaks[neighbor_idx] = 999.0f;
        neighbor_idx = (neighbor_idx + 1) % 10;
    } else {
        neighbor_alert_status = 0;
        // --- CORRECTION : AJOUT LOG CONNECTION REUSSIE ---
        // Affiche un petit message pour confirmer qu'on a bien reçu la réponse du voisin
        HAL_UART_Transmit(&huart3, (uint8_t*)"✅ Voisin calme (Ping OK)\r\n", 28, 100);
    }

    if (my_alert_status && neighbor_alert_status) {
        HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n🚨🚨🚨 ALERTE GENERALE CONFIRMEE ! 🚨🚨🚨\r\n", 54, 100);
    }

    pbuf_free(p); tcp_close(tpcb);
    return ERR_OK;
}


// --- OUTILS CONVERSION (Le RTC parle en BCD, nous en Décimal) ---
// Exemple : 45 secondes -> 0x45 (BCD)
uint8_t decToBcd(int val) { return (uint8_t)((val/10*16) + (val%10)); }
int bcdToDec(uint8_t val) { return (int)((val/16*10) + (val%16)); }

// --- DRIVER RTC (BQ32000 via I2C1) ---


void RTC_SetTime(uint8_t hour, uint8_t min, uint8_t sec)
{
    uint8_t data[3];
    data[0] = decToBcd(sec); // Registre 0x00 : Secondes
    data[1] = decToBcd(min); // Registre 0x01 : Minutes
    data[2] = decToBcd(hour); // Registre 0x02 : Heures

    // On écrit à partir de l'adresse mémoire 0x00 du RTC
    HAL_I2C_Mem_Write(&hi2c1, RTC_ADDR, 0x00, 1, data, 3, 1000);
}

void RTC_GetTime(uint8_t *hour, uint8_t *min, uint8_t *sec)
{
    uint8_t data[3];
    // On lit 3 octets à partir de l'adresse 0x00
    HAL_I2C_Mem_Read(&hi2c1, RTC_ADDR, 0x00, 1, data, 3, 1000);

    *sec  = bcdToDec(data[0] & 0x7F); // Masque pour ignorer le bit d'arrêt
    *min  = bcdToDec(data[1]);
    *hour = bcdToDec(data[2]);
}

// --- DRIVER FRAM (CY15B104Q via SPI2) ---


// Fonction interne pour activer l'écriture (WREN)
void FRAM_WriteEnable(void)
{
    uint8_t cmd = FRAM_WREN;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // CS Low (Active)
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);   // CS High (Inactive)
}

void FRAM_Write(uint32_t addr, uint8_t *pData, uint16_t size)
{
    FRAM_WriteEnable(); // Indispensable avant chaque écriture !

    uint8_t cmd[4];
    cmd[0] = FRAM_WRITE;
    // La FRAM a des adresses sur 3 octets (24 bits)
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // CS Low
    HAL_SPI_Transmit(&hspi2, cmd, 4, 100);       // Envoi Commande + Adresse
    HAL_SPI_Transmit(&hspi2, pData, size, 100);  // Envoi Données
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);   // CS High
}

void FRAM_Read(uint32_t addr, uint8_t *pData, uint16_t size)
{
    uint8_t cmd[4];
    cmd[0] = FRAM_READ;
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // CS Low
    HAL_SPI_Transmit(&hspi2, cmd, 4, 100);       // Envoi Commande + Adresse
    HAL_SPI_Receive(&hspi2, pData, size, 100);   // Lecture Données
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);   // CS High
}

//fonction qui appele quand le serveur répond
void ntp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    if (p != NULL && p->tot_len >= 48) // Un paquet NTP fait 48 octets min
    {
        // Le timestamp se trouve à l'octet 40 du paquet NTP
        uint8_t *payload = (uint8_t *)p->payload;

        // Les données arrivent en Big Endian, il faut les reconstruire
        uint32_t ntp_seconds = (payload[40] << 24) | (payload[41] << 16) | (payload[42] << 8) | payload[43];

        // 1. Convertir NTP (1900) vers UNIX (1970)
        // On soustrait 2 208 988 800 secondes
        uint32_t unix_time = ntp_seconds - NTP_TIMESTAMP_DELTA;

        // 2. Appliquer le fuseau horaire (Belgique)
        unix_time += (TIMEZONE_OFFSET * 3600);

        // 3. Calculer Heure/Minute/Seconde (Maths simples modulo)
        uint8_t h = (unix_time / 3600) % 24;
        uint8_t m = (unix_time / 60) % 60;
        uint8_t s = unix_time % 60;

        // 4. Mettre à jour le RTC hardware
        RTC_SetTime(h, m, s);
        ntp_synced = 1;
        // 5. Feedback UART
        char msg[64];
        sprintf(msg, "✅ HEURE SYNC NTP : %02d:%02d:%02d\r\n", h, m, s);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
    }

    // Nettoyage indispensable
    pbuf_free(p);
    udp_remove(pcb); // On ferme la connexion une fois l'heure reçue
}

// Fonction pour demander l'heure au serveur NTP
void Sync_Time_NTP(void)
{
    struct udp_pcb *pcb;
    struct pbuf *p;
    ip_addr_t dest_ip;
    err_t err;

    HAL_UART_Transmit(&huart3, (uint8_t*)"🌍 NTP: Connexion au serveur de temps...\r\n", 44, 100);

    pcb = udp_new();
    if (!pcb) return;

    // 💡 IMPORTANT : On dit à LwIP quelle fonction appeler quand une réponse arrive
    udp_recv(pcb, ntp_recv_callback, NULL);

    // IP Belge (be.pool.ntp.org -> exemple 85.201.16.177 ou 193.191.177.6)
    // Si tu veux changer, assure-toi que l'IP est valide. Celle de ton code est OK.
    ipaddr_aton(NTP_SERVER_IP, &dest_ip);

    // Création du paquet NTP
    uint8_t *buff = (uint8_t *)mem_malloc(NTP_MSG_LEN);
    if (!buff) { udp_remove(pcb); return; }

    memset(buff, 0, NTP_MSG_LEN);
    buff[0] = 0x1B; // LI=0, VN=3, Mode=3 (Client)

    p = pbuf_alloc(PBUF_TRANSPORT, NTP_MSG_LEN, PBUF_RAM);
    if (!p) { mem_free(buff); udp_remove(pcb); return; }

    memcpy(p->payload, buff, NTP_MSG_LEN);
    mem_free(buff);

    // Envoi de la requête
    err = udp_sendto(pcb, p, &dest_ip, NTP_PORT);
    pbuf_free(p);

    if (err == ERR_OK) {
        // On ne ferme PAS le pcb ici ! On attend la réponse dans la callback.
        HAL_UART_Transmit(&huart3, (uint8_t*)"✅ NTP: Requete envoyee, attente...\r\n", 38, 100);
    } else {
        HAL_UART_Transmit(&huart3, (uint8_t*)"❌ NTP: Erreur envoi.\r\n", 22, 100);
        udp_remove(pcb); // Si l'envoi échoue, on ferme tout de suite
    }
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  //uint8_t system_running = 0; // Drapeau d'état système
  //une fois
  //mettre à lehure ici
  //RTC_SetTime(17, 21, 24); // On force l'heure à midi pile

  // Demande NTP au démarrage (Etape 3)

  extern struct netif gnetif;
    char msg[60];

    // --- 1. ATTENTE RESEAU ---
    HAL_UART_Transmit(&huart3, (uint8_t*)"⏳ Attente IP DHCP...\r\n", 24, 100);
    while (!netif_is_up(&gnetif) || ip4_addr_isany_val(*netif_ip4_addr(&gnetif))) {
        osDelay(500);
    }

    sprintf(msg, "✅ IP Obtenue : %s\r\n", ipaddr_ntoa(&gnetif.ip_addr));
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

    // --- 2. SYNCHRO NTP ---
    int retry_count = 0;
    while (ntp_synced == 0) {
        Sync_Time_NTP();
        // On attend 2 secondes la réponse
        for(int i=0; i<20; i++) {
            osDelay(100);
            if(ntp_synced) break;
        }
        if(ntp_synced == 0) {
            retry_count++;
            sprintf(msg, "⚠️ Pas de reponse NTP, tentative %d...\r\n", retry_count);
            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
        }
    }

    // --- 3. SYSTEME PRET ---
    HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n>>> SYSTEME PRET (Appuyez sur BLEU) <<<\r\n", 44, 100);

    uint8_t system_running = 0; // Variable d'état

    for(;;) {
        // Détection Bouton Bleu
        if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET) {
          // Anti-rebond
          osDelay(50);
          while (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET);

          if (!system_running) {
            // --- DEMARRAGE ---
            system_running = 1;
            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n>>> SYSTEM STARTED <<<\r\n", 26, 100);

            // Affichage de l'heure actuelle
            uint8_t h, m, s; RTC_GetTime(&h, &m, &s);
            sprintf(msg, "🕒 Heure systeme : %02d:%02d:%02d\r\n", h, m, s);
            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

            			SeismicEvent last;
                      // On met tout à 0 par sécurité avant de lire
                      memset(&last, 0, sizeof(SeismicEvent));

                      FRAM_Read(FRAM_EVENT_ADDR, (uint8_t*)&last, sizeof(SeismicEvent));

                      // Vérification de sécurité : Est-ce que les données semblent valides ?
                      // (Intensité réaliste et heure cohérente 0-23h)
                      if(last.intensity > 1.0f && last.intensity < 20000.0f && last.hour < 24) {
                          sprintf(msg, "💾 DERNIER SEISME : %02d:%02d:%02d (Force: %.0f)\r\n", last.hour, last.min, last.sec, last.intensity);
                      } else {
                          sprintf(msg, "💾 Pas de seisme valide en memoire.\r\n");
                      }
                      HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

            // Réveil des tâches
            osThreadResume(heartBeatTaskHandle);
            osThreadResume(presenceTaskHandle);
            osThreadResume(logMessageTaskHandle);
            osThreadResume(seismicTaskHandle);
            osThreadResume(clientTaskHandle);

          } else {
            // --- ARRET ---
            system_running = 0;
            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n>>> SYSTEM STOPPED <<<\r\n", 26, 100);

            // Suspension des tâches
            osThreadSuspend(heartBeatTaskHandle);
            osThreadSuspend(presenceTaskHandle);
            osThreadSuspend(logMessageTaskHandle);
            osThreadSuspend(seismicTaskHandle);
            osThreadSuspend(clientTaskHandle);
          }
        }
        osDelay(100);
    }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LogMessageTask */
/**
* @brief Function implementing the logMessageTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LogMessageTask */
void LogMessageTask(void const * argument)
{
  /* USER CODE BEGIN LogMessageTask */
  /* Infinite loop */
  char msg[80];
  for(;;)
  {
    uint16_t ax = adc_raw[0];
    uint16_t ay = adc_raw[1];
    uint16_t az = adc_raw[2];
    uint32_t t = HAL_GetTick();

    int len;
    // Log des valeurs brutes ADC
    len = snprintf(msg, sizeof(msg), "[%8lu ms] X=%4u Y=%4u Z=%4u\r\n", t, ax, ay, az);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, HAL_MAX_DELAY);

    /* affichage temporaire pour verifier les RMS*/
    // Log des valeurs calculées RMS
    /* affichage temporaire pour verifier les RMS*/

        // 💡 CORRECTION : Lecture protégée par Mutex
        float log_x, log_y, log_z;
        osMutexWait(dataMutexHandle, osWaitForever);
        log_x = rms_x;
        log_y = rms_y;
        log_z = rms_z;
        osMutexRelease(dataMutexHandle);

        // Log des valeurs calculées RMS (avec les copies locales)
        len = snprintf(msg, sizeof(msg),
           "[%lu ms] RMS: X=%.2f Y=%.2f Z=%.2f\r\n", t, log_x, log_y, log_z);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, HAL_MAX_DELAY);


    /*connect to qq1*/


 /*Time uart envoie données*/
    osDelay(1000); // ≈10 Hz => recommandé en debug
  }
  /* USER CODE END LogMessageTask */
}

/* USER CODE BEGIN Header_StartClientTask */
/**
* @brief Function implementing the clientTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartClientTask */
void StartClientTask(void const * argument)
{
  /* USER CODE BEGIN StartClientTask */
  /* Infinite loop */
    /* attendre init réseau */
    extern struct netif gnetif;
    while (!netif_is_up(&gnetif))
        osDelay(100);

    // Liste des voisins à interroger
    /*const char *node_list[] = {
        "192.168.1.180",
		"192.168.1.177",
        "192.168.1.185",
        "192.168.129.59"
    };*/

    const char *node_list[] = {"192.168.129.59"};
    const uint8_t node_count = 1;

    /*const uint8_t node_count = 4;*/ /* changer absolument quand je rajoute une ip*/

    for(;;)
    {
        // 💡 Boucle pour contacter chaque nœud de la liste
    	for (int i = 0; i < node_count; i++)
    	{
    	    send_data_request_tcp(node_list[i]);
    	    osDelay(200); // Petite pause pour ne pas saturer
    	}
        osDelay(2000); // Pause longue (10s) avant le prochain cycle
    }
  /* USER CODE END StartClientTask */
}

/* USER CODE BEGIN Header_StartServerTask */
/**
* @brief Function implementing the serverTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServerTask */
void StartServerTask(void const * argument)
{
  /* USER CODE BEGIN StartServerTask */
  /* Infinite loop */
    extern struct netif gnetif;
    while (!netif_is_up(&gnetif))
        osDelay(100);
    HAL_UART_Transmit(&huart3, (uint8_t*)"SERVER TASK STARTED\r\n", 21, HAL_MAX_DELAY);

    tcp_server_init(); // <-- nouveau serveur TCP (mise en écoute)
    for(;;)
    {
        osDelay(1); // Le thread reste vivant mais ne fait rien (LwIP gère par interruptions)
    }
  /* USER CODE END StartServerTask */
}

/* USER CODE BEGIN Header_StartHeartBeatTask */
/**
* @brief Function implementing the heartBeatTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHeartBeatTask */
void StartHeartBeatTask(void const * argument)
{
  /* USER CODE BEGIN StartHeartBeatTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    osDelay(500); // 1 Hz (500ms ON, 500ms OFF)
  }
  /* USER CODE END StartHeartBeatTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
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

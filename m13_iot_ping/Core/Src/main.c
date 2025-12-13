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

RTC_HandleTypeDef hrtc;

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
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

  osMutexDef(dataMutex);
  dataMutexHandle = osMutexCreate(osMutex(dataMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of logMessageTask */
  osThreadDef(logMessageTask, LogMessageTask, osPriorityNormal, 0, 256);
  logMessageTaskHandle = osThreadCreate(osThread(logMessageTask), NULL);

  /* definition and creation of clientTask */
  osThreadDef(clientTask, StartClientTask, osPriorityBelowNormal, 0, 256);
  clientTaskHandle = osThreadCreate(osThread(clientTask), NULL);

  /* definition and creation of serverTask */
  osThreadDef(serverTask, StartServerTask, osPriorityBelowNormal, 0, 256);
  serverTaskHandle = osThreadCreate(osThread(serverTask), NULL);

  /* definition and creation of heartBeatTask */
  osThreadDef(heartBeatTask, StartHeartBeatTask, osPriorityIdle, 0, 256);
  heartBeatTaskHandle = osThreadCreate(osThread(heartBeatTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* definition and creation of presenceTask */
  // 💡 Tâche Présence : Envoie un message UDP "Je suis là" à tout le monde
  osThreadDef(presenceTask, StartPresenceTask, osPriorityBelowNormal, 0, 256);
  presenceTaskHandle = osThreadCreate(osThread(presenceTask), NULL);
  /* add threads, ... */


  /* definition and creation of seismicTask */
  // 💡 Tâche Sismique : Priorité ÉLEVÉE car calcul critique en temps réel
  osThreadDef(seismicTask, StartSeismicTask, osPriorityAboveNormal, 0, 512);
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
void StartSeismicTask(void const * argument)
{
    // Initialisation des buffers à 0
    memset(buf_x, 0, sizeof(buf_x));
    memset(buf_y, 0, sizeof(buf_y));
    memset(buf_z, 0, sizeof(buf_z));

    // Index circulaire et timer pour éviter le spam UART
    int buffer_idx = 0;
    uint32_t last_alert_time = 0; // 👇 Pour limiter l'affichage

    for(;;)
    {
        // Attente synchro DMA (100Hz)
        osSemaphoreWait(adcReadySemHandle, osWaitForever);

        // 1) Lire valeurs brutes et conversion float
        float raw_x = (float)adc_raw[0];
        float raw_y = (float)adc_raw[1];
        float raw_z = (float)adc_raw[2];

        // 2) Remplissage du Buffer (SANS FILTRE 0.9/0.1)
        buf_x[buffer_idx] = raw_x;
        buf_y[buffer_idx] = raw_y;
        buf_z[buffer_idx] = raw_z;

        // 3) Calcul de la MOYENNE (Gravité / Offset DC)
        float mean_x = 0, mean_y = 0, mean_z = 0;
        for(int i = 0; i < WINDOW_SIZE; i++) {
            mean_x += buf_x[i];
            mean_y += buf_y[i];
            mean_z += buf_z[i];
        }
        mean_x /= WINDOW_SIZE;
        mean_y /= WINDOW_SIZE;
        mean_z /= WINDOW_SIZE;

        // 4) Calcul de la VARIANCE (Énergie de la vibration)
        float var_x = 0, var_y = 0, var_z = 0;
        for(int i = 0; i < WINDOW_SIZE; i++) {
            var_x += powf(buf_x[i] - mean_x, 2);
            var_y += powf(buf_y[i] - mean_y, 2);
            var_z += powf(buf_z[i] - mean_z, 2);
        }

        // 5) Calcul RMS final (Écart-type)
        float calc_rms_x = sqrtf(var_x / WINDOW_SIZE);
        float calc_rms_y = sqrtf(var_y / WINDOW_SIZE);
        float calc_rms_z = sqrtf(var_z / WINDOW_SIZE);

        // 6) Mise à jour thread-safe des globales
        osMutexWait(dataMutexHandle, osWaitForever);
        rms_x = calc_rms_x;
        rms_y = calc_rms_y;
        rms_z = calc_rms_z;
        osMutexRelease(dataMutexHandle);

        // 👇 7) NOUVEAU : Détection de seuil et Alerte UART
        // On vérifie si ça dépasse 300 ET si on n'a pas déjà crié il y a moins d'1 seconde
        if ((calc_rms_x > SEISMIC_THRESHOLD || calc_rms_y > SEISMIC_THRESHOLD || calc_rms_z > SEISMIC_THRESHOLD)
             && (HAL_GetTick() - last_alert_time > 1000))
        {
            char alert_msg[100];
            // On affiche le message avec la valeur max détectée pour info
            float max_val = calc_rms_x;
            if(calc_rms_y > max_val) max_val = calc_rms_y;
            if(calc_rms_z > max_val) max_val = calc_rms_z;

            int len = snprintf(alert_msg, sizeof(alert_msg),
                               "\r\n>>> ⚠️ TREMBLEMENT DETECTE ! (Intensite: %.0f) <<<\r\n", max_val);

            // Note: C'est mieux d'utiliser un Mutex sur l'UART si possible,
            // mais ici HAL_UART_Transmit est "thread-safe" par défaut sur STM32 (il bloque).
            HAL_UART_Transmit(&huart3, (uint8_t*)alert_msg, len, 100);

            // On allume la LED Rouge (LD3) pour le fun, si tu veux !
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

            last_alert_time = HAL_GetTick();
        }
        else if (HAL_GetTick() - last_alert_time > 1000)
        {
             // On éteint la LED Rouge si tout est calme
             HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
        }

        // Gestion de l'index circulaire
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
err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (err != ERR_OK)
    {
        if (p) pbuf_free(p);
        return err;
    }

    // 💡 Si p == NULL, cela signifie que le client a fermé la connexion
    if (p == NULL)
    {
        tcp_close(tpcb);
        return ERR_OK;
    }

    // Buffer local pour impression
    char buffer[256];
    memset(buffer, 0, sizeof(buffer));

    // Copie sécurisée des données
    uint16_t len = (p->len < sizeof(buffer) - 1) ? p->len : sizeof(buffer) - 1;
    memcpy(buffer, p->payload, len);
    buffer[len] = 0;

    // 🔥 LOG UART : afficher tout ce qui arrive
    char msg[300];
    snprintf(msg, sizeof(msg),
             "📩 RECU DE %s : %s\r\n",
             ipaddr_ntoa(&tpcb->remote_ip),
             buffer);

    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // 💡 Indiquer à LwIP qu'on a bien traité les données (pour libérer la fenêtre TCP)
    tcp_recved(tpcb, p->tot_len);

    // --- Traitement logique ---
    // Si la requête contient "data_request", on renvoie nos valeurs RMS
    if (strstr(buffer, "\"data_request\""))
    {
    	// 💡 CORRECTION : Variables temporaires pour la lecture atomique
    	        float tx_x, tx_y, tx_z;

    	        // On verrouille le Mutex pour lire proprement
    	        osMutexWait(dataMutexHandle, osWaitForever);
    	        tx_x = rms_x;
    	        tx_y = rms_y;
    	        tx_z = rms_z;
    	        osMutexRelease(dataMutexHandle);

    	        char response[256];
    	        int resp_len = snprintf(response, sizeof(response),
    	                "{ \"type\": \"data_response\", \"id\": \"nucleo-8\", "
    	                "\"timestamp\": \"2025-10-02T08:21:01Z\", "
    	                "\"acceleration\": {\"x\": %.2f, \"y\": %.2f, \"z\": %.2f}, "
    	                "\"status\": \"normal\" }",
    	                tx_x, tx_y, tx_z); // On utilise les copies locales

        // Envoi de la réponse
        tcp_write(tpcb, response, resp_len, TCP_WRITE_FLAG_COPY);
        // Force l'envoi immédiat
        tcp_output(tpcb);
        // Fermeture de la connexion (modèle requête/réponse simple)
        tcp_close(tpcb);
    }

    // Libération du buffer de réception
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

err_t tcp_client_recv_response(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (err != ERR_OK) {
        if (p) pbuf_free(p);
        return err;
    }

    if (p == NULL) {
        // Le serveur a fermé la connexion
        tcp_close(tpcb);
        return ERR_OK;
    }

    // Lecture des données reçues
    char buffer[256];
    memset(buffer, 0, sizeof(buffer));
    uint16_t len = (p->len < sizeof(buffer) - 1) ? p->len : sizeof(buffer) - 1;
    memcpy(buffer, p->payload, len);
    buffer[len] = 0;

    // 💡 Affichage de la réponse du voisin (Acceleration X, Y, Z)
    // C'est ici que tu valides l'exigence "Récupérer leurs données"
    char msg[300];
    snprintf(msg, sizeof(msg), "✅ REPONSE REÇUE : %s\r\n", buffer);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Acquittement LwIP
    tcp_recved(tpcb, p->tot_len);

    // Une fois la réponse reçue, on peut fermer proprement
    pbuf_free(p);
    tcp_close(tpcb);

    return ERR_OK;
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
  uint8_t system_running = 0; // Drapeau d'état système

  for(;;)
  {
    // 💡 Détection de l'appui bouton (Actif Haut)
    if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
    {
      osDelay(50); // anti-rebond simple
      while (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET); // Attente relâchement

      // Bascule de l'état
      if (!system_running) {
        system_running = 1;
        HAL_UART_Transmit(&huart3, (uint8_t*)"SYSTEM STARTED\r\n", 16, HAL_MAX_DELAY);
        // 💡 RÉVEIL de toutes les tâches
        osThreadResume(heartBeatTaskHandle);
        osThreadResume(presenceTaskHandle);
        osThreadResume(logMessageTaskHandle);
        osThreadResume(seismicTaskHandle);
        osThreadResume(clientTaskHandle);
      } else {
        system_running = 0;
        HAL_UART_Transmit(&huart3, (uint8_t*)"SYSTEM STOPPED\r\n", 16, HAL_MAX_DELAY);
        // 💡 Mise en PAUSE de toutes les tâches
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
        osDelay(10000); // Pause longue (10s) avant le prochain cycle
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

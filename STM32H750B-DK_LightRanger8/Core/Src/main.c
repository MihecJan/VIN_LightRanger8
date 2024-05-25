/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

	#include "lightranger8_VIN.h"

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
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

ETH_HandleTypeDef heth;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c4;

LTDC_HandleTypeDef hltdc;

QSPI_HandleTypeDef hqspi;

RTC_HandleTypeDef hrtc;

SAI_HandleTypeDef hsai_BlockA2;
SAI_HandleTypeDef hsai_BlockB2;

MMC_HandleTypeDef hmmc1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

	// Descriptions are a guess!

	#define DEV_SOFT_RESET_CMD_SET                          0x00
	#define DEV_SOFT_RESET_CMD_CLEAR                        0x01

	// I2C address of lightRanger8.
	#define LIGHTRANGER_ADDRESS								0x52

	// We check that the value we read from LIGHTRANGER8_IDENTIFICATION_MODEL_ID register matches with this one.
	#define DEV_IDENTIFICATION_MODEL_ID						0xEA

	// Write this value into LIGHTRANGER8_SYSTEM_MODE_START register to enable/disable ranging mode.
	#define DEV_CMD_RANGING_ENABLE							0x40
	#define DEV_CMD_RANGING_DISABLE                         0x00

	// Write this value into LIGHTRANGER8_SYSTEM_INTERRUPT_CLEAR register to clear interrupt.
	#define DEV_SYS_INT_CLEAR_BIT                           0x01

	// If the value read from LIGHTRANGER8_GPIO_TIO_HV_STATUS register does not match this bit mask,
	// it indicates that new data is not yet available.
	#define DEV_DATA_READY_BIT                              0x03

	// LightRanger 8 distance modes. Settings of ranging distance modes of LightRanger 8 Click driver.
	#define LIGHTRANGER8_DISTANCE_MODE_SHORT                0x00
	#define LIGHTRANGER8_DISTANCE_MODE_MEDIUM               0x01
	#define LIGHTRANGER8_DISTANCE_MODE_LONG                 0x02

	// Distance mode config macros
	#define DEV_SHORT_MODE_RAN_CONF_PERIOD_A                0x07
	#define DEV_SHORT_MODE_RAN_CONF_PERIOD_B                0x05
	#define DEV_SHORT_MODE_RAN_CONF_VAL_PH_H                0x38
	#define DEV_SHORT_MODE_SD_CONFIG_WOI_SD0                0x07
	#define DEV_SHORT_MODE_SD_CONFIG_WOI_SD1                0x05
	#define DEV_SHORT_MODE_SD_CONF_IN_PH_SD0                0x06
	#define DEV_SHORT_MODE_SD_CONF_IN_PH_SD1                0x06

	#define DEV_MEDIUM_MODE_RAN_CONF_PERIOD_A               0x0B
	#define DEV_MEDIUM_MODE_RAN_CONF_PERIOD_B               0x09
	#define DEV_MEDIUM_MODE_RAN_CONF_VAL_PH_H               0x78
	#define DEV_MEDIUM_MODE_SD_CONFIG_WOI_SD0               0x0B
	#define DEV_MEDIUM_MODE_SD_CONFIG_WOI_SD1               0x09
	#define DEV_MEDIUM_MODE_SD_CONF_IN_PH_SD0               0x0A
	#define DEV_MEDIUM_MODE_SD_CONF_IN_PH_SD1               0x0A

	#define DEV_LONG_MODE_RAN_CONF_PERIOD_A                 0x0F
	#define DEV_LONG_MODE_RAN_CONF_PERIOD_B                 0x0D
	#define DEV_LONG_MODE_RAN_CONF_VAL_PH_H                 0xB8
	#define DEV_LONG_MODE_SD_CONFIG_WOI_SD0                 0x0F
	#define DEV_LONG_MODE_SD_CONFIG_WOI_SD1                 0x0D
	#define DEV_LONG_MODE_SD_CONF_IN_PH_SD0                 0x0E
	#define DEV_LONG_MODE_SD_CONF_IN_PH_SD1                 0x0E

	// Calibration
	#define DEV_PART_TO_PART_RANGE_OFFSET_RESET             0x0000
	#define DEV_MM_CONF_INNER_OFFSET_RESET                  0x0000
	#define DEV_MM_CONF_OUTER_OFFSET_RESET                  0x0000

	#define DEV_PART_TO_PART_RANGE_OFFSET_MULT              4

	// User input
	#define USER_INVALID_INPUT								0x00
	#define USER_HELP										0x01
	#define USER_DISTANCE_MODE								0x02
	#define USER_CALIBRATION								0x03

	#define		BUFSIZE 256
	char		SendBuffer[BUFSIZE];

	uint8_t received_char;

	HAL_StatusTypeDef retval;

	int period_ms = 1000;
	int calibration_distance_mm = 100;

	uint8_t paused = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ETH_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_RTC_Init(void);
static void MX_SAI2_Init(void);
static void MX_SDMMC1_MMC_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C4_Init(void);
/* USER CODE BEGIN PFP */

	uint8_t read_data_u8(uint16_t);
	uint16_t read_data_u16(uint16_t);
	uint32_t read_data_u32(uint16_t);
	void write_data_u8(uint16_t, uint8_t);
	void write_data_u16(uint16_t, uint16_t);
	void write_data_u32(uint16_t, uint32_t);

	uint8_t user_input(uint8_t);

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
	int i=0;
	uint8_t Space[] = " - ";

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ETH_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_QUADSPI_Init();
  MX_RTC_Init();
  MX_SAI2_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C4_Init();
  /* USER CODE BEGIN 2 */

	// Check if sensor is ready for communication
  	checkForLightRangerCommunication();

	// Now the sensor is ready for communication
  	print("Acknowledgment received, ready for communication.\n\r");

  	checkModelID();
  	HAL_Delay(100);

  	// Reset calibration offset
  	write_data_u16( LIGHTRANGER8_ALGO_PART_TO_PART_RANGE_OFFSET_MM, DEV_PART_TO_PART_RANGE_OFFSET_RESET );
    write_data_u16( LIGHTRANGER8_MM_CONFIG_INNER_OFFSET_MM, DEV_MM_CONF_INNER_OFFSET_RESET );
    write_data_u16( LIGHTRANGER8_MM_CONFIG_OUTER_OFFSET_MM, DEV_MM_CONF_OUTER_OFFSET_RESET );

  	// Default distance mode
  	//lightranger8_set_distance_mode( LIGHTRANGER8_DISTANCE_MODE_SHORT );
  	lightranger8_set_distance_mode( LIGHTRANGER8_DISTANCE_MODE_MEDIUM );
  	//lightranger8_set_distance_mode( LIGHTRANGER8_DISTANCE_MODE_LONG );
  	HAL_Delay(3000);

  	snprintf(SendBuffer, BUFSIZE, "Period: %dms\n\r", period_ms);
  	HAL_UART_Transmit(&huart3, SendBuffer, strlen(SendBuffer), 100);
  	HAL_Delay(3000);

	lightranger8_start_measurement( period_ms );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while ( 1 )
	{
		if ( HAL_UART_Receive(&huart3, &received_char, 1, 1) == HAL_OK )
		{
			user_input( received_char );
		}

		while ( paused )
		{
			if ( HAL_UART_Receive(&huart3, &received_char, 1, HAL_MAX_DELAY) == HAL_OK )
			{
				user_input( received_char );
			}
		}

		/* In lightranger8.c this function is defined and implemented,
		but it is not used. It should return 0 if new data is ready and 1 if it is still measuring.
		Uncommenting the bellow while loop should prevent the same measurement from coming more than once,
		but shortly (immediately or a few seconds - random) the program gets stuck in the while loop and
		just keeps sending "not ready". If you keep it commented, same measurement might come more than once
		if the delay is short (example: 10ms).

		uint8_t dataReady;
		do {
			dataReady = lightranger8_new_data_ready();
			print("not ready\n\r");
			HAL_Delay(10);
		} while (!dataReady);*/

		uint16_t distance = lightranger8_get_distance();
		snprintf(SendBuffer, BUFSIZE, "\rDistance: %d mm       \n\rPress 'p' to pause.", distance);
		HAL_UART_Transmit(&huart3, SendBuffer, strlen(SendBuffer), 100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay( period_ms );
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 22;
  RCC_OscInitStruct.PLL.PLLN = 169;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_0;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_MII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 16;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 2;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 0;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x10707DBC;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 654;
  hltdc.Init.AccumulatedActiveH = 485;
  hltdc.Init.TotalWidth = 660;
  hltdc.Init.TotalHeigh = 487;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

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
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA2.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA2.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockA2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA2.Init.PdmInit.Activation = DISABLE;
  hsai_BlockA2.Init.PdmInit.MicPairsNbr = 1;
  hsai_BlockA2.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockA2.FrameInit.FrameLength = 8;
  hsai_BlockA2.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA2.SlotInit.FirstBitOffset = 0;
  hsai_BlockA2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA2.SlotInit.SlotNumber = 1;
  hsai_BlockA2.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockA2) != HAL_OK)
  {
    Error_Handler();
  }
  hsai_BlockB2.Instance = SAI2_Block_B;
  hsai_BlockB2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB2.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB2.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockB2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB2.Init.Synchro = SAI_SYNCHRONOUS;
  hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockB2.Init.PdmInit.Activation = DISABLE;
  hsai_BlockB2.Init.PdmInit.MicPairsNbr = 1;
  hsai_BlockB2.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockB2.FrameInit.FrameLength = 8;
  hsai_BlockB2.FrameInit.ActiveFrameLength = 1;
  hsai_BlockB2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockB2.SlotInit.FirstBitOffset = 0;
  hsai_BlockB2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockB2.SlotInit.SlotNumber = 1;
  hsai_BlockB2.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockB2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */

  /* USER CODE END SAI2_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_MMC_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hmmc1.Instance = SDMMC1;
  hmmc1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hmmc1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hmmc1.Init.BusWide = SDMMC_BUS_WIDE_8B;
  hmmc1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hmmc1.Init.ClockDiv = 0;
  if (HAL_MMC_Init(&hmmc1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MII_TX_ER_nINT_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PH15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DISPD7_Pin */
  GPIO_InitStruct.Pin = LCD_DISPD7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_DISPD7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE5 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS2_ID_Pin OTG_FS2_PSO_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS2_ID_Pin|OTG_FS2_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : audio_Int_Pin */
  GPIO_InitStruct.Pin = audio_Int_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(audio_Int_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS2_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS2_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS2_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MII_TX_ER_nINT_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = MII_TX_ER_nINT_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PH12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint8_t user_input (uint8_t input)
{

	if ( input == 'p' )
	{
		paused = 1;
		lightranger8_stop_measurement ();
		print("\n\r>p\n\r");
		print("Paused. Press h for help.\n\r");
		return;
	}

	if ( !paused )
	{
		return;
	}

	if ( input == 'g')
	{
		paused = 0;
		print(">g\n\r");
		print("\n\r");
		lightranger8_start_measurement( period_ms );
		return;
	}

	switch ( input )
	{
        case 'c': {
        	print(">c\n\r");
        	snprintf(SendBuffer, BUFSIZE, "Place an object at %d mm distance from sensor.\n\r", calibration_distance_mm);
        	HAL_UART_Transmit(&huart3, SendBuffer, strlen(SendBuffer), 100);

        	HAL_Delay(3000);

          	print("Calibration starting in 5");
          	HAL_Delay(1000);
          	print(" 4");
          	HAL_Delay(1000);
          	print(" 3");
          	HAL_Delay(1000);
          	print(" 2");
          	HAL_Delay(1000);
          	print(" 1");
          	HAL_Delay(1000);
          	print("\n\r");

          	lightranger8_calibrate_offset( calibration_distance_mm, period_ms );
            break;
        }
        case '+': {
        	print(">+\n\r");
        	if ( calibration_distance_mm < 1000 )
        	{
        		calibration_distance_mm = calibration_distance_mm + 100;
        	}

        	snprintf(SendBuffer, BUFSIZE, "Calibration distance: %dmm\n\r", calibration_distance_mm);
        	HAL_UART_Transmit(&huart3, SendBuffer, strlen(SendBuffer), 100);
			break;
		}
        case '-': {
        	print(">-\n\r");
        	if ( calibration_distance_mm > 100 )
			{
				calibration_distance_mm = calibration_distance_mm - 100;
			}

			snprintf(SendBuffer, BUFSIZE, "Calibration distance: %dmm\n\r", calibration_distance_mm);
			HAL_UART_Transmit(&huart3, SendBuffer, strlen(SendBuffer), 100);
			break;
		}
        case 's': {
        	print(">s\n\r");
			lightranger8_set_distance_mode( LIGHTRANGER8_DISTANCE_MODE_SHORT );
			break;
		}
        case 'm': {
        	print(">m\n\r");
			lightranger8_set_distance_mode( LIGHTRANGER8_DISTANCE_MODE_MEDIUM );
			break;
		}
        case 'l': {
        	print(">l\n\r");
			lightranger8_set_distance_mode( LIGHTRANGER8_DISTANCE_MODE_LONG );
			break;
		}
        case 'f': {
        	print(">f\n\r");
			if ( period_ms > 100 )
			{
				period_ms = period_ms - 100;
			}

			snprintf(SendBuffer, BUFSIZE, "Period: %dms\n\r", period_ms);
			HAL_UART_Transmit(&huart3, SendBuffer, strlen(SendBuffer), 100);
			break;
		}
        case 'v': {
        	print(">v\n\r");
			if ( period_ms < 2000 )
			{
				period_ms = period_ms + 100;
			}

			snprintf(SendBuffer, BUFSIZE, "Period: %dms\n\r", period_ms);
			HAL_UART_Transmit(&huart3, SendBuffer, strlen(SendBuffer), 100);
			break;
		}
        case 'h': {
        	print(">h\n\r");
			print(" c\tStart calibration.\n\r");
			print(" +\tIncrease calibration distance.\n\r");
			print(" -\tDecrease calibration distance.\n\r");

			print(" s\tSet distance mode to short.\n\r");
			print(" m\tSet distance mode to medium.\n\r");
			print(" l\tSet distance mode to long.\n\r");

			print(" p\tPause.\n\r");
			print(" g\tContinue measuring.\n\r");
			print(" f\tFaster measurements.\n\r");
			print(" v\tSlower measurements.\n\r");

			print(" h\tHelp.\n\r");
			break;
		}
        default: {
        	print("Press h for help.\n\r");
        	return;
        }
	}
}

void print ( char * string )
{
	snprintf(SendBuffer, BUFSIZE, string);
	HAL_UART_Transmit(&huart3, SendBuffer, strlen(SendBuffer), 100);
}

/**
 * Reads 1 byte from MEMORY_ADDRESS and returns it.
 *
 * @param[in] MEMORY_ADDRESS : Register to read 1 byte from.
 */
uint8_t read_data_u8 ( uint16_t MEMORY_ADDRESS )
{
	uint8_t bytes[1];

	retval = HAL_I2C_Mem_Read(
		&hi2c4,
		LIGHTRANGER_ADDRESS,
		MEMORY_ADDRESS,
		I2C_MEMADD_SIZE_16BIT,
		bytes,
		1,
		HAL_MAX_DELAY
	);

	return bytes[0];
}

/**
 * Reads 2 bytes from MEMORY_ADDRESS and returns concatenated uint16_t.
 *
 * @param[in] MEMORY_ADDRESS : Register to read 2 bytes from.
 */
uint16_t read_data_u16 ( uint16_t MEMORY_ADDRESS )
{
	uint8_t bytes[2];

	retval = HAL_I2C_Mem_Read(
		&hi2c4,
		LIGHTRANGER_ADDRESS,
		MEMORY_ADDRESS,
		I2C_MEMADD_SIZE_16BIT,
		bytes,
		2,
		HAL_MAX_DELAY
	);

	return (uint16_t)bytes[0] << 8 | bytes[1];
}

/**
 * Reads 4 bytes from MEMORY_ADDRESS and returns concatenated uint32_t.
 *
 * @param[in] MEMORY_ADDRESS : Register to read 4 bytes from.
 */
uint32_t read_data_u32 ( uint16_t MEMORY_ADDRESS )
{
	uint8_t bytes[4];

	retval = HAL_I2C_Mem_Read(
		&hi2c4,
		LIGHTRANGER_ADDRESS,
		MEMORY_ADDRESS,
		I2C_MEMADD_SIZE_16BIT,
		bytes,
		4,
		HAL_MAX_DELAY
	);

	return (uint32_t)bytes[0] << 24 | (uint32_t)bytes[1] << 16 | (uint32_t)bytes[2] << 8 | bytes[3];
}

/**
 * Writes 1 byte to MEMORY_ADDRES.
 *
 * @param[in] MEMORY_ADDRESS : Register to write 1 byte in.
 * @param[in] data : Value to write.
 */
void write_data_u8 ( uint16_t MEMORY_ADDRESS, uint8_t data )
{
	retval = HAL_I2C_Mem_Write(
		&hi2c4,
		LIGHTRANGER_ADDRESS,
		MEMORY_ADDRESS,
		I2C_MEMADD_SIZE_16BIT,
		&data,
		1,
		HAL_MAX_DELAY
	);
}

/**
 * Writes 2 bytes to MEMORY_ADDRES.
 *
 * @param[in] MEMORY_ADDRESS : Register to write 2 bytes in.
 * @param[in] data : Value to write.
 */
void write_data_u16 ( uint16_t MEMORY_ADDRESS, uint16_t data )
{
	uint8_t bytes[2];
	bytes[0] = data >> 8;
	bytes[1] = data & 0x00FF;

	retval = HAL_I2C_Mem_Write(
		&hi2c4,
		LIGHTRANGER_ADDRESS,
		MEMORY_ADDRESS,
		I2C_MEMADD_SIZE_16BIT,
		bytes,
		2,
		HAL_MAX_DELAY
	);
}

/**
 * Writes 4 bytes to MEMORY_ADDRES.
 *
 * @param[in] MEMORY_ADDRESS : Register to write 4 bytes in.
 * @param[in] data : Value to write.
 */
void write_data_u32 ( uint16_t MEMORY_ADDRESS, uint32_t data )
{
	uint8_t bytes[4];
	bytes[0] = data >> 24;
	bytes[1] = (data >> 16) & 0x000000FF;
	bytes[2] = (data >> 8) & 0x000000FF;
	bytes[3] = data & 0x000000FF;

	retval = HAL_I2C_Mem_Write(
		&hi2c4,
		LIGHTRANGER_ADDRESS,
		MEMORY_ADDRESS,
		I2C_MEMADD_SIZE_16BIT,
		bytes,
		4,
		HAL_MAX_DELAY
	);
}

void checkForLightRangerCommunication()
{
	retval = !HAL_OK;
	while (retval != HAL_OK)
	{
		snprintf(SendBuffer, BUFSIZE, "\n\rWaiting for acknowledge from LightRanger8 Click...\n\r");
		HAL_UART_Transmit(&huart3, SendBuffer, strlen(SendBuffer), 100);

		retval = HAL_I2C_IsDeviceReady(&hi2c4, (uint16_t)(LIGHTRANGER_ADDRESS), 3, 5);

		if (retval != HAL_OK) /* No ACK Received At That Address */
		{
			snprintf(SendBuffer, BUFSIZE, "Didn't respond.\n\r");
			HAL_UART_Transmit(&huart3, SendBuffer, strlen(SendBuffer), 100);
		}

		HAL_Delay(1000);
	}
}

void checkModelID()
{
  	uint8_t readModelID = read_data_u8 ( LIGHTRANGER8_IDENTIFICATION_MODEL_ID );

  	if (readModelID != DEV_IDENTIFICATION_MODEL_ID)
  	{
  	  	snprintf(SendBuffer, BUFSIZE, "Your model ID: %X, it should be 0xEA\n\r", readModelID);
  		HAL_UART_Transmit(&huart3, SendBuffer, strlen(SendBuffer), 100);
  		exit(1);
  	}
  	else
  	{
  		print("Model ID validated. \n\r");
  	}
}

void lightranger8_software_reset()
{
	write_data_u8(LIGHTRANGER8_SOFT_RESET, DEV_SOFT_RESET_CMD_SET);
	HAL_Delay(1);
	write_data_u8(LIGHTRANGER8_SOFT_RESET, DEV_SOFT_RESET_CMD_CLEAR);
	HAL_Delay(1);
}

void lightranger8_set_distance_mode ( uint8_t distance_mode )
{
    switch ( distance_mode )
    {
        case LIGHTRANGER8_DISTANCE_MODE_SHORT: {
            write_data_u8( LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_A, DEV_SHORT_MODE_RAN_CONF_PERIOD_A );
            write_data_u8( LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_B, DEV_SHORT_MODE_RAN_CONF_PERIOD_B );
            write_data_u8( LIGHTRANGER8_RANGE_CONFIG_VALID_PHASE_HIGH, DEV_SHORT_MODE_RAN_CONF_VAL_PH_H );
            write_data_u8( LIGHTRANGER8_SD_CONFIG_WOI_SD0, DEV_SHORT_MODE_SD_CONFIG_WOI_SD0 );
            write_data_u8( LIGHTRANGER8_SD_CONFIG_WOI_SD1, DEV_SHORT_MODE_SD_CONFIG_WOI_SD1 );
            write_data_u8( LIGHTRANGER8_SD_CONFIG_INITIAL_PHASE_SD0, DEV_SHORT_MODE_SD_CONF_IN_PH_SD0 );
            write_data_u8( LIGHTRANGER8_SD_CONFIG_INITIAL_PHASE_SD1, DEV_SHORT_MODE_SD_CONF_IN_PH_SD1 );
            print("Distance mode set to SHORT.\n\r");
            break;
        }
        case LIGHTRANGER8_DISTANCE_MODE_MEDIUM: {
            write_data_u8( LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_A, DEV_MEDIUM_MODE_RAN_CONF_PERIOD_A );
            write_data_u8( LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_B, DEV_MEDIUM_MODE_RAN_CONF_PERIOD_B );
            write_data_u8( LIGHTRANGER8_RANGE_CONFIG_VALID_PHASE_HIGH, DEV_MEDIUM_MODE_RAN_CONF_VAL_PH_H );
            write_data_u8( LIGHTRANGER8_SD_CONFIG_WOI_SD0, DEV_MEDIUM_MODE_SD_CONFIG_WOI_SD0 );
            write_data_u8( LIGHTRANGER8_SD_CONFIG_WOI_SD1, DEV_MEDIUM_MODE_SD_CONFIG_WOI_SD1 );
            write_data_u8( LIGHTRANGER8_SD_CONFIG_INITIAL_PHASE_SD0, DEV_MEDIUM_MODE_SD_CONF_IN_PH_SD0 );
            write_data_u8( LIGHTRANGER8_SD_CONFIG_INITIAL_PHASE_SD1, DEV_MEDIUM_MODE_SD_CONF_IN_PH_SD1 );
            print("Distance mode set to MEDIUM.\n\r");
            break;
        }
        case LIGHTRANGER8_DISTANCE_MODE_LONG: {
            write_data_u8( LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_A, DEV_LONG_MODE_RAN_CONF_PERIOD_A );
            write_data_u8( LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_B, DEV_LONG_MODE_RAN_CONF_PERIOD_B );
            write_data_u8( LIGHTRANGER8_RANGE_CONFIG_VALID_PHASE_HIGH, DEV_LONG_MODE_RAN_CONF_VAL_PH_H );
            write_data_u8( LIGHTRANGER8_SD_CONFIG_WOI_SD0, DEV_LONG_MODE_SD_CONFIG_WOI_SD0 );
            write_data_u8( LIGHTRANGER8_SD_CONFIG_WOI_SD1, DEV_LONG_MODE_SD_CONFIG_WOI_SD1 );
            write_data_u8( LIGHTRANGER8_SD_CONFIG_INITIAL_PHASE_SD0, DEV_LONG_MODE_SD_CONF_IN_PH_SD0 );
            write_data_u8( LIGHTRANGER8_SD_CONFIG_INITIAL_PHASE_SD1, DEV_LONG_MODE_SD_CONF_IN_PH_SD1 );
            print("Distance mode set to LONG.\n\r");
            break;
        }
        default: {
        	print("Wrong distance mode.");
            return;
        }
    }
}

void lightranger8_calibrate_offset ( uint16_t target_distance_mm, uint32_t period_ms )
{
	int16_t offset;
    int16_t average_distance = 0;
    uint16_t distance;
    uint8_t averaging_number = 50;

    lightranger8_start_measurement( period_ms );
    for (uint8_t i = 0; i < averaging_number; i++)
    {
        while ( lightranger8_get_interrupt_state() != 0 ) {
        	print(".");
        	HAL_Delay(1);
        }

        distance = lightranger8_get_distance();
        average_distance = average_distance + distance;
    }
    lightranger8_stop_measurement();

    average_distance = average_distance / averaging_number;
    offset = target_distance_mm - average_distance;

    write_data_u16( LIGHTRANGER8_ALGO_PART_TO_PART_RANGE_OFFSET_MM, offset * DEV_PART_TO_PART_RANGE_OFFSET_MULT );

    snprintf(SendBuffer, BUFSIZE, "\n\rCalibration complete.\n\rOffset: %d mm\n\r", offset);
    HAL_UART_Transmit(&huart3, SendBuffer, strlen(SendBuffer), 100);
}

uint8_t lightranger8_get_interrupt_state ()
{
    return HAL_GPIO_ReadPin( GPIOH, GPIO_PIN_12 );
}

/**
 * Returns if new data is ready or not for acquiring distance measured.
 * 0 -> Not ready
 * 1 -> Ready
 */
uint8_t lightranger8_new_data_ready()
{
    uint8_t tmp = read_data_u8 ( LIGHTRANGER8_GPIO_TIO_HV_STATUS );

    if ( tmp != DEV_DATA_READY_BIT ) {
        return 0;
    }

    return 1;
}

void lightranger8_system_interrupt_clear()
{
	uint8_t data = DEV_SYS_INT_CLEAR_BIT;
    write_data_u8(LIGHTRANGER8_SYSTEM_INTERRUPT_CLEAR, data);
}


void lightranger8_start_measurement ( uint32_t period_ms )
{
  	uint16_t oscCalibrateVal = read_data_u16 ( LIGHTRANGER8_RESULT_OSC_CALIBRATE_VAL );

  	uint32_t oscData = oscCalibrateVal * period_ms;
  	write_data_u32(LIGHTRANGER8_SYSTEM_INTERMEASUREMENT_PERIOD, oscData);

  	lightranger8_system_interrupt_clear();

  	// Enable ranging mode
	write_data_u8( LIGHTRANGER8_SYSTEM_MODE_START, DEV_CMD_RANGING_ENABLE );
}

void lightranger8_stop_measurement ()
{
	// Disable ranging mode
	write_data_u8( LIGHTRANGER8_SYSTEM_MODE_START, DEV_CMD_RANGING_DISABLE );
}

uint16_t lightranger8_get_distance()
{
	uint16_t distance = read_data_u16 ( LIGHTRANGER8_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 );

	return distance;
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

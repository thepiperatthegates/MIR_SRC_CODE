/*

Author : Hijazi 
Email : hijaziaziz@gmail.com



********************************************************************************
This is the src file, for filtering src code, check conv-direct.c and conv-direct.h for headers file.

Toolchain : 
i)            STM32CubeIDE (original toolchain)
ii)           VSCode with STM32CubeMX extension, clangd for compiler front-end, arm-none-eabi-gcc from ARM GNU for on-board chip flashing,
              ST-Link GBD for debugging tool-chain, CMake for cross-compile software development


STM32CubeIDE is recommended as it requires the least amount of step needed to compile, debug and flash the code onto the chips, 
but the IDE is very slow in this case. 

********************************************************************************


*/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal_gpio.h"  //HAL header files
#include "usb_device.h"         //USB driver header files

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include "conv-direct.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


//Put the data on the upper 32-bits
#define COMPUTATION_DUALMODEINTERLEAVED_ADCMASTER_RESULT(DATA) \
  ((DATA) & 0x0000FFFF)


//Put the data on the lower 32-bits
#define COMPUTATION_DUALMODEINTERLEAVED_ADCSLAVE_RESULT(DATA) \
  ((DATA) >> 16)


#define COMBINE_BYTES(msb, b2, b3, lsb) ((msb << 24) | (b2 << 16) | (b3 << 8) | (lsb))

#define ARRAY_LENGTH(x) (sizeof(x) / sizeof((x)[0]))
#define SAMPLE_DAC 4096
#define Vref (float)3.3

#define res_8b 0xFF
#define res_12b 0xFFF
#define res_16b 0xFFFF

#define A_MAX (double) 500  // Maximum amplifier output current (miliampere)
#define A_MIN (double)-500 // Minimum amplifier output current	(miliampere)


#define OFFSET_LINEAR_ERROR_DAC 28


#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;
DMA_HandleTypeDef hdma_dac1_ch2;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

//Functions declaration

void sin_cos_generator(uint32_t amplitude_1, uint32_t amplitude_2, int offset_1, int offset_2, uint32_t direction);
void dac_start(void);
void adc_start(void);
void inital_setup(float32_t dac_frequency, uint32_t running_time, uint32_t amplitude_1,
                  int offset_1, uint32_t amplitude_2, int offset_2, uint32_t direction,
				  uint32_t mode_button_flag, uint32_t reset_mcu_flag);
void myTimerChangeDAC(float want_frequency_dac);
void myTimerSetupADC(void);
void timers_Start(void);
void run_time_timer_2(uint32_t running_time);
void tim_2_callback(void);
void rx_function(void);
void filter_function_half(void);
void filter_function_complete(void);
void combining_transmission_total(void);
void combining_transmission_total_filter(void);
void dc_generator(uint32_t amplitude_1, uint32_t amplitude_2);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//debugging purposes
uint32_t half_callback;
uint8_t adc_flag = RESET; // adc flag
uint32_t trigger_time;
__IO uint8_t send_flag_filter, send_flag_raw;
uint8_t interrupt_flag = 0;


// Variables for ADC purpose
__attribute__((section(".section_data"), used)) ALIGN_32BYTES(__IO uint32_t adc_full_buff[4]);   // Mixed of two ADC buffer			(16+16=32 Bit)
__attribute__((section(".section_data"), used)) ALIGN_32BYTES(__IO uint16_t adc_buff_first[DMA_ADC_BUFF_SIZE]);  // DMA OF FIRST ADC1	(16 bit)
__attribute__((section(".section_data"), used)) ALIGN_32BYTES(__IO uint16_t adc_buff_second[DMA_ADC_BUFF_SIZE]); // DMA OF SECOND ADC2      (16 bit)

__attribute__((section(".section_data"), used)) ALIGN_32BYTES(__IO uint32_t adc_dual_channel[2 * DMA_ADC_BUFF_SIZE]); // DMA OF THIRD ADC3 (16 bit) (2*DMA_ADC_BUFF_SIZE because we have dual channel mode
__attribute__((section(".section_data"), used)) ALIGN_32BYTES(__IO uint16_t adc_buff_third[DMA_ADC_BUFF_SIZE]);
__attribute__((section(".section_data"), used)) ALIGN_32BYTES(__IO uint16_t adc_buff_fourth[DMA_ADC_BUFF_SIZE]);


__attribute__((section(".section_data"), used)) ALIGN_32BYTES(__IO uint8_t half_buffer_flag);
__attribute__((section(".section_data"), used)) ALIGN_32BYTES(__IO uint8_t full_buffer_flag);

__attribute__((section(".section_data"), used)) uint8_t hsem_noti = 3;

__attribute__((section(".section_data"), used)) uint8_t convert_noti = 3;
__attribute__((section(".section_data"), used)) ALIGN_32BYTES(float outbuffer1[DMA_ADC_BUFF_SIZE]);
__attribute__((section(".section_data"), used)) ALIGN_32BYTES(float outbuffer2[DMA_ADC_BUFF_SIZE]);
__attribute__((section(".section_data"), used)) ALIGN_32BYTES(float outbuffer3[DMA_ADC_BUFF_SIZE]);
__attribute__((section(".section_data"), used)) ALIGN_32BYTES(float outbuffer4[DMA_ADC_BUFF_SIZE]);

__attribute__((section(".section_data"), used)) ALIGN_32BYTES(__IO uint16_t send_back_1[DMA_ADC_BUFF_SIZE]);
__attribute__((section(".section_data"), used)) ALIGN_32BYTES(__IO uint16_t send_back_2[DMA_ADC_BUFF_SIZE]);
__attribute__((section(".section_data"), used)) ALIGN_32BYTES(__IO uint16_t send_back_3[DMA_ADC_BUFF_SIZE]);
__attribute__((section(".section_data"), used)) ALIGN_32BYTES(__IO uint16_t send_back_4[DMA_ADC_BUFF_SIZE]);



//  Variables for DAC purpose
__attribute__((section(".dac_section"), used)) ALIGN_32BYTES(__IO uint16_t sinVal_1[SAMPLE_DAC]);
__attribute__((section(".dac_section"), used)) ALIGN_32BYTES(__IO uint16_t sinVal_2[SAMPLE_DAC]); // AND LAGI SATU, CUBA HILANGKAN IO
__attribute__((section(".dac_section"), used)) ALIGN_32BYTES( uint32_t dac_dma_combine[SAMPLE_DAC]);

uint8_t interrupt_callback = 0, callback_1, callback_2;
uint8_t measure_flag = 1;


//  Variables for data packaging
uint8_t payload_usb[USB_PAYLOAD];
__attribute__((section(".my_ram_section"), used)) ALIGN_32BYTES( uint8_t combined_transmission_buffer[MAX_TOTAL_TRANSMISSION_BUFFER]);


// Identifier bits 
static char hall_1 = 'H';
static char hall_2 = 'I';
static char current_1 = 'J';
static char current_2 = 'K';

typedef struct
{
  unsigned char payload_ascii[TOTAL_RX_BUFFER * 9];
  unsigned char first_ascii[TOTAL_RX_BUFFER], second_ascii[TOTAL_RX_BUFFER],
      third_ascii[TOTAL_RX_BUFFER], fourth_ascii[TOTAL_RX_BUFFER], fifth_ascii[TOTAL_RX_BUFFER],
      sixth_ascii[TOTAL_RX_BUFFER], seventh_ascii[TOTAL_RX_BUFFER], eight_ascii[TOTAL_RX_BUFFER],
	     ninth_ascii[TOTAL_RX_BUFFER];
} rx_data;

typedef struct
{
  uint32_t incoming_packetUINT[INCOMING_PACKET_UINT];
  int incoming_packetINT[INCOMING_PACKET_INT];
  float incoming_packetFLOAT;

} rx_separate_data;


// FILTER variable 
ConvDirect_Container conv1;
ConvDirect_Container conv2;
ConvDirect_Container conv3;
ConvDirect_Container conv4;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_ADC2_Init();
  MX_DAC1_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOK, Start_LED_Pin, 0);


  // init FIR filter for 4 signals
  ConvDirect_Init(&conv1, filter_taps);
  ConvDirect_Init(&conv2, filter_taps);
  ConvDirect_Init(&conv3, filter_taps);
  ConvDirect_Init(&conv4, filter_taps);


  adc_start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (adc_flag == RESET)
    {

      if (measure_flag == 1 || measure_flag == 0 || measure_flag == 3)
      {
        combining_transmission_total_filter();
      }
      if (measure_flag == 2)
      {
        combining_transmission_total();
      }


    }
    else if (adc_flag == SET)
    {
      //		  	Half_Buffer_Complete();


      //		  filter_function_half();
    }

    if (interrupt_flag == 1)
    {
      // Blue LED turned off to show the programm has stopped running
      tim_2_callback();
      interrupt_flag = 0;
    }
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
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 1;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
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
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T4_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = 4;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DualModeData = ADC_DUALMODEDATAFORMAT_32_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_7CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = 4;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
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
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T4_TRGO;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = ENABLE;
  hadc3.Init.Oversampling.Ratio = 4;
  hadc3.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
  hadc3.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc3.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_16CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T5_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 5-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOK, Start_LED_Pin|Filter_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Start_LED_Pin Filter_LED_Pin */
  GPIO_InitStruct.Pin = Start_LED_Pin|Filter_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



/* 

Receive data from software function

@pars: my_rx_data, my_rx_seperate_data
@macros: COMBINE_BYTES(msb, b2, b3, lsb) ((msb << 24) | (b2 << 16) | (b3 << 8) | (lsb))
@summary:    i) my_rx_data is the total unpacking data received from the software
            ii) my_rx_seperate_data is the data after unpacking
            iii) call initial_setup at last to sort the unpacked receiving data

*/
void rx_function(void)
{

  rx_data my_rx_data;
  rx_separate_data my_rx_separate_data;

  memcpy(my_rx_data.payload_ascii, payload_usb, sizeof(my_rx_data.payload_ascii));

  // Divide the payload for easier conversion
  memcpy(my_rx_data.first_ascii, my_rx_data.payload_ascii, TOTAL_RX_BUFFER);
  memcpy(my_rx_data.second_ascii, my_rx_data.payload_ascii + TOTAL_RX_BUFFER, TOTAL_RX_BUFFER);
  memcpy(my_rx_data.third_ascii, my_rx_data.payload_ascii + (TOTAL_RX_BUFFER * 2), TOTAL_RX_BUFFER);
  memcpy(my_rx_data.fourth_ascii, my_rx_data.payload_ascii + (TOTAL_RX_BUFFER * 3), TOTAL_RX_BUFFER);
  memcpy(my_rx_data.fifth_ascii, my_rx_data.payload_ascii + (TOTAL_RX_BUFFER * 4), TOTAL_RX_BUFFER);
  memcpy(my_rx_data.sixth_ascii, my_rx_data.payload_ascii + (TOTAL_RX_BUFFER * 5), TOTAL_RX_BUFFER);
  memcpy(my_rx_data.seventh_ascii, my_rx_data.payload_ascii + (TOTAL_RX_BUFFER * 6), TOTAL_RX_BUFFER);
  memcpy(my_rx_data.eight_ascii, my_rx_data.payload_ascii + (TOTAL_RX_BUFFER * 7), TOTAL_RX_BUFFER);
  memcpy(my_rx_data.ninth_ascii, my_rx_data.payload_ascii + (TOTAL_RX_BUFFER * 8), TOTAL_RX_BUFFER);

  // Change BYTES Data to integer to use it
  my_rx_separate_data.incoming_packetUINT[0] = COMBINE_BYTES(my_rx_data.first_ascii[0], my_rx_data.first_ascii[1], my_rx_data.first_ascii[2], my_rx_data.first_ascii[3]); // RUN TIME
  memcpy(&my_rx_separate_data.incoming_packetFLOAT, my_rx_data.second_ascii, sizeof(my_rx_separate_data.incoming_packetFLOAT));
  my_rx_separate_data.incoming_packetUINT[1] = COMBINE_BYTES(my_rx_data.third_ascii[0], my_rx_data.third_ascii[1], my_rx_data.third_ascii[2], my_rx_data.third_ascii[3]);         // AMPLITUDE DAC 1
  my_rx_separate_data.incoming_packetINT[0] = COMBINE_BYTES(my_rx_data.fourth_ascii[0], my_rx_data.fourth_ascii[1], my_rx_data.fourth_ascii[2], my_rx_data.fourth_ascii[3]);      // offset 1
  my_rx_separate_data.incoming_packetUINT[2] = COMBINE_BYTES(my_rx_data.fifth_ascii[0], my_rx_data.fifth_ascii[1], my_rx_data.fifth_ascii[2], my_rx_data.fifth_ascii[3]);         // amplitude2
  my_rx_separate_data.incoming_packetINT[1] = COMBINE_BYTES(my_rx_data.sixth_ascii[0], my_rx_data.sixth_ascii[1], my_rx_data.sixth_ascii[2], my_rx_data.sixth_ascii[3]);          // OFFSET DAC2
  my_rx_separate_data.incoming_packetUINT[3] = COMBINE_BYTES(my_rx_data.seventh_ascii[0], my_rx_data.seventh_ascii[1], my_rx_data.seventh_ascii[2], my_rx_data.seventh_ascii[3]); // ROTATION DIRECTION
  my_rx_separate_data.incoming_packetUINT[4] = COMBINE_BYTES(my_rx_data.eight_ascii[0], my_rx_data.eight_ascii[1], my_rx_data.eight_ascii[2], my_rx_data.eight_ascii[3]); // decides to use filter or not
  my_rx_separate_data.incoming_packetUINT[5] = COMBINE_BYTES(my_rx_data.ninth_ascii[0], my_rx_data.ninth_ascii[1], my_rx_data.ninth_ascii[2], my_rx_data.ninth_ascii[3]);


  inital_setup(my_rx_separate_data.incoming_packetFLOAT, my_rx_separate_data.incoming_packetUINT[0], my_rx_separate_data.incoming_packetUINT[1],
               my_rx_separate_data.incoming_packetINT[0], my_rx_separate_data.incoming_packetUINT[2], my_rx_separate_data.incoming_packetINT[1],
               my_rx_separate_data.incoming_packetUINT[3], my_rx_separate_data.incoming_packetUINT[4], my_rx_separate_data.incoming_packetUINT[5]);

  // LETS DO THIS LATER TO SEE IF I CAN SEND JEVEYRTHING IN FLOAT INSTEAD OF USING MAPPING FUNCTION
}



/* 

Sorting station for Rx data from Python software

@pars: dac_frequency, amplitude_1, offset_1, amplitude_2, offset_2, direction, mode_button_flag, reset_mcu_flag
@summary:   

The MCU receives the unpacked data and will now starts the DAC according to input user GUI. 
if (mode_button_flag == 0) is for default sin waves generation.
else if (mode_button_flag == 3) if the calibration process starts.    
 if(reset_mcu_flag == 1){  NVIC_SystemReset();} activates software implementation of RESET button.

*/

void inital_setup(float32_t dac_frequency, uint32_t running_time, uint32_t amplitude_1,
                  int offset_1, uint32_t amplitude_2, int offset_2, uint32_t direction, uint32_t mode_button_flag, uint32_t reset_mcu_flag)
{

  measure_flag = mode_button_flag;			//1,2,3 values are used
  // mode for send parameter
  if (mode_button_flag == 0)
  {
    sin_cos_generator(amplitude_1, amplitude_2, offset_1, offset_2, direction);
    myTimerChangeDAC(dac_frequency);
    dac_start();
  }
  // THIS MODE IS UNUSED
  else if (mode_button_flag == 1)
  {
    // use tim4 as interrupt disable
    //		run_time_timer_2(running_time);
    // myTimerChangeDAC(dac_frequency);
  }
  // THIS MODE IS UNUSED
  else if (mode_button_flag == 2)
  {
    // stop function
    //    tim_2_callback();
  }
  else if (mode_button_flag == 3)
  {
	 dc_generator(amplitude_1, amplitude_2);
	 myTimerChangeDAC(dac_frequency);
	 dac_start();
  }

  //hardware reset for microcontroller
  if(reset_mcu_flag == 1){
	  NVIC_SystemReset();
  }

}

void sin_cos_generator(uint32_t amplitude_1, uint32_t amplitude_2,
                       int offset_1, int offset_2, uint32_t direction) /// sine wave for DAC
{


		if (direction == 1)
		{ // clockwise rotation
		for (int i = 0; i < SAMPLE_DAC; i++)
		{
		  // Generate sine wave with configurable amplitude
		  double a_amp1 = amplitude_1 * cos(i * 2 * PI / SAMPLE_DAC) + offset_1;
		  double a_amp2 = amplitude_2 * sin(i * 2 * PI / SAMPLE_DAC) + offset_2;

		  // Convert to DAC value using derived formula
		  sinVal_1[i] = (uint16_t)((A_MAX - a_amp1) * (res_12b / (A_MAX - A_MIN)));
		  // Convert to DAC value using derived formula
		  sinVal_2[i] = (uint16_t)((A_MAX - a_amp2) * (res_12b / (A_MAX - A_MIN)));

		  // combine it into one buffer
		  dac_dma_combine[i] = ((uint32_t)sinVal_1[i] << 16) | sinVal_2[i];
		}
		}
		else if (direction == 2)
		{ // anti-clockwise rotation
		for (int i = 0; i < SAMPLE_DAC; i++)
		{
		  // Generate sine wave with configurable amplitude
		  double a_amp1 = amplitude_1 * cos(i * 2 * PI / SAMPLE_DAC) + offset_1;
		  double a_amp2 = amplitude_2 * (-sin(i * 2 * PI / SAMPLE_DAC)) + offset_2;
		  // Convert to DAC value using derived formula
		  sinVal_1[i] = (uint16_t)((A_MAX - a_amp1) * (res_12b / (A_MAX - A_MIN)));
		  // Convert to DAC value using derived formula
		  sinVal_2[i] = (uint16_t)((A_MAX - a_amp2) * (res_12b / (A_MAX - A_MIN)));
		  // combine it into one buffer
		  dac_dma_combine[i] = ((uint32_t)sinVal_1[i] << 16) | sinVal_2[i];
		}
		}


  if (amplitude_1 == 0)
  {
	  uint16_t dc1 = (uint16_t)((A_MAX - amplitude_1) * (res_12b / (A_MAX - A_MIN)));
	  uint16_t dc2 = (uint16_t)((A_MAX - amplitude_2) * (res_12b / (A_MAX - A_MIN)));
	  	 for (int i = 0; i < SAMPLE_DAC; i++)
	  	 {
	  		dac_dma_combine[i] = ((uint32_t)dc2 << 16 | dc1);
	  	   }
  }
}

void dc_generator(uint32_t amplitude_1, uint32_t amplitude_2)
{


    uint16_t dc1 = (uint16_t)((A_MAX - amplitude_1) * (res_12b / (A_MAX - A_MIN)));
    uint16_t dc2 = (uint16_t)((A_MAX - amplitude_2) * (res_12b / (A_MAX - A_MIN)));
	 for (int i = 0; i < SAMPLE_DAC; i++)
	 {
		dac_dma_combine[i] = ((uint32_t)dc2 << 16 | dc1);
	   }

}
// void run_time_timer_2(uint32_t running_time)
//{
//	RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;      //enable tim5 clock
//
//	//make it atomic
//	HAL_NVIC_DisableIRQ(TIM2_IRQn);
//	TIM2->CR1 &= ~(TIM_CR1_CEN);
//	//make sure everything is off
//     RCC->APB1LRSTR |=  (RCC_APB1LRSTR_TIM2RST);
//     RCC->APB1LRSTR &= ~(RCC_APB1LRSTR_TIM2RST);

//
//	__IO uint32_t arr_placeholder  = 0;
//	__IO uint32_t psc_placeholder = 0;
//	__IO uint32_t psc_variable = 20000-1;
//	psc_placeholder = HAL_RCC_GetPCLK1Freq()*2/(20000);
//	arr_placeholder = (psc_placeholder*running_time)-1;            //in Sekunde
//
//	TIM2->PSC = psc_variable; //
//
//	TIM2->EGR  |= TIM_EGR_UG;	// Send an update event to reset the timer and apply settings.
//	//TIM2->CCR1=0;              // some delay
//	__NOP();
//	__NOP();
//	__NOP();
//	__NOP();
//	TIM2->SR = 0;
//
//
//	TIM2->ARR = arr_placeholder; //in second
//	TIM2->CNT = 0;
//
//	TIM2->DIER |= TIM_DIER_UIE; //enable overflow interrupt
//
//	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
//	HAL_NVIC_EnableIRQ(TIM2_IRQn);
//
//
//	TIM2->CR1 |= TIM_CR1_CEN;      //enable timer
//
//	//NVIC_SetPriority(TIM5_IRQn, 0x03);			//set priority for Timer 5
//		//NVIC_EnableIRQ(TIM5_IRQn);				//emable Interrupt for Timer 5
//
//	callback_1++;
// }

void myTimerChangeDAC(float want_frequency_dac)
{
  __IO uint32_t psc_variable = 4 - 1;
  __IO uint32_t preTimer = 100000000 * 2 / (4);
  __IO uint32_t bottom_part = (uint32_t)(want_frequency_dac * SAMPLE_DAC);
  __IO uint32_t chosenARR = (uint32_t)(preTimer / bottom_part); // for all the steps (4096 in total)

  RCC->APB1LENR |= RCC_APB1LENR_TIM5EN; // ENABLE tim4 clock

  TIM5->CR1 &= ~(TIM_CR1_CEN);
  // make sure everything is off
  RCC->APB1LRSTR |= (RCC_APB1LRSTR_TIM5RST);
  RCC->APB1LRSTR &= ~(RCC_APB1LRSTR_TIM5RST);

  // PRESCALER SHOULD BE HUGE (OR ZERO WHO KNOWS) CAUSE WE WANT ACTUALLY NOT THAT HUGE OF FREQUENCY
  TIM5->PSC = psc_variable;
  TIM5->EGR |= TIM_EGR_UG; // Send an update event to reset the timer and apply settings.
  // TIM5->CCR1=0;              // some delay
  // 	__NOP();
  // 	__NOP();
  // 	__NOP();
  //	__NOP();

  TIM5->SR = 0;
  // REGISTER COUNTER      //ALRIGHT WE SHOULD MAKE THIS CHANGE ON THE FLY IF IT IS POSSIBLE
  TIM5->ARR = chosenARR - 1;
  TIM5->CNT = 0;

  // SET THE MASTER MODE TO OUTPUT THE UPDATE EVENT (TRGO = UPDATE)
  TIM5->CR2 &= ~((1 << 6) | (1 << 5) | (1 << 4)); // Clear MMS bits
  TIM5->CR2 |= ((0 << 6) | (1 << 5) | (0 << 4));  // SET MMS TO '010' -> UPDATE EVENT AS TRIGGER OUTPUT (TRGO)
  TIM5->CR1 &= ~(TIM_CR1_DIR);                    // count up
  TIM5->CR1 |= TIM_CR1_CEN;
}

void dac_start(void)
{
  SCB_CleanDCache_by_Addr((uint32_t *)&dac_dma_combine, sizeof(dac_dma_combine)); // CLEAN THE DIRRTY CACHE, PUSH THE DATA INTO THE MEMORY
  if (HAL_DACEx_DualSetValue(&hdac1, DAC_ALIGN_12B_R, 0	, 0) != HAL_OK)
  {
    /* DAC value set error */
    Error_Handler();
  }
  HAL_DACEx_DualStart_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)dac_dma_combine, SAMPLE_DAC, DAC_ALIGN_12B_R);
}

void adc_start(void)
{
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK) // ADC1 Calibration
  {
    Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK) // ADC1 Calibration
  {
    Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK) // ADC1 Calibration
  {
    Error_Handler();
  }

  HAL_TIM_Base_Start(&htim4);



  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_full_buff, DMA_ADC_BUFF_SIZE) != HAL_OK) // Start ADC in Multi Mode
  {
    Error_Handler();
  }

  if (HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc_dual_channel, 2 * DMA_ADC_BUFF_SIZE) != HAL_OK) // Start third ADC
  {
    Error_Handler();
  }
}

void combining_transmission_total(void)
{

  if (send_flag_raw == 1)
  {
    send_flag_raw = 0;
    uint8_t *ptr = (uint8_t *) combined_transmission_buffer;
    for (int32_t i = 0; i < DMA_ADC_BUFF_SIZE; i++)
    for (int32_t i = 0; i < DMA_ADC_BUFF_SIZE; i++)
    {
        // 1) Copy hall_1 (1 byte)
        *ptr++ = hall_1;   // ptr might now be odd

        // 2) Copy send_back_1[i] (2 bytes) as two STRB instructions
        {
            uint8_t *src16 = (uint8_t *)&adc_buff_first[i];
            *ptr++ = src16[0];  // low byte
            *ptr++ = src16[1];  // high byte
        }

        // 3) Copy hall_2 (1 byte)
        *ptr++ = hall_2;   // ptr might now be odd

        // 4) Copy send_back_2[i] (2 bytes):
        {
            uint8_t *src16 = (uint8_t *)&adc_buff_second[i];
            *ptr++ = src16[0];  // low byte
            *ptr++ = src16[1];  // high byte
        }

        // 5) Copy current_1 (1 byte)
        *ptr++ = current_1; // ptr might now be odd

        // 6) Copy send_back_3[i] (2 bytes):
        {
            uint8_t *src16 = (uint8_t *)&adc_buff_third[i];
            *ptr++ = src16[0];  // low byte
            *ptr++ = src16[1];  // high byte
        }

        // 7) Copy current_2 (1 byte)
        *ptr++ = current_2; // ptr might now be odd

        // 8) Copy send_back_4[i] (2 bytes):
        {
            uint8_t *src16 = (uint8_t *)&adc_buff_fourth[i];
            *ptr++ = src16[0];  // low byte
            *ptr++ = src16[1];  // high byte
        }
    }
  CDC_Transmit_FS( (uint8_t *) combined_transmission_buffer, MAX_TOTAL_TRANSMISSION_BUFFER);
  }
}

void combining_transmission_total_filter(void)
{
  if (send_flag_filter == 1)
  {
      send_flag_filter = 0;
      uint8_t *ptr = (uint8_t *)combined_transmission_buffer;

      for (int32_t i = 0; i < DMA_ADC_BUFF_SIZE; i++)
      {
          // 1) Copy hall_1 (1 byte)
          *ptr++ = hall_1;   // ptr might now be odd

          // 2) Copy send_back_1[i] (2 bytes) as two STRB instructions
          {
              uint8_t *src16 = (uint8_t *)&send_back_1[i];
              *ptr++ = src16[0];  // low byte
              *ptr++ = src16[1];  // high byte
          }

          // 3) Copy hall_2 (1 byte)
          *ptr++ = hall_2;   // ptr might now be odd

          // 4) Copy send_back_2[i] (2 bytes):
          {
              uint8_t *src16 = (uint8_t *)&send_back_2[i];
              *ptr++ = src16[0];  // low byte
              *ptr++ = src16[1];  // high byte
          }

          // 5) Copy current_1 (1 byte)
          *ptr++ = current_1; // ptr might now be odd

          // 6) Copy send_back_3[i] (2 bytes):
          {
              uint8_t *src16 = (uint8_t *)&send_back_3[i];
              *ptr++ = src16[0];  // low byte
              *ptr++ = src16[1];  // high byte
          }

          // 7) Copy current_2 (1 byte)
          *ptr++ = current_2; // ptr might now be odd

          // 8) Copy send_back_4[i] (2 bytes):
          {
              uint8_t *src16 = (uint8_t *)&send_back_4[i];
              *ptr++ = src16[0];  // low byte
              *ptr++ = src16[1];  // high byte
          }
      }

      CDC_Transmit_FS((uint8_t *) combined_transmission_buffer, MAX_TOTAL_TRANSMISSION_BUFFER);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) // CALLBACK FUNCTION COMPLETE
{

  SCB_InvalidateDCache_by_Addr(((uint32_t *)&adc_full_buff[DMA_ADC_BUFF_SIZE / 2]), (4 * DMA_ADC_BUFF_SIZE / 2)); // INVALIDATE THE CACHE TO GET THE MOST RECENT DATA
  SCB_InvalidateDCache_by_Addr(((uint32_t *)&adc_dual_channel[DMA_ADC_BUFF_SIZE]), (8 * DMA_ADC_BUFF_SIZE / 2));
  for (uint32_t i = DMA_ADC_BUFF_SIZE / 2; i < (DMA_ADC_BUFF_SIZE); i++) // ERRROR : MIGHT HAVE SOMETHING TO DO WITH FOR LOOP
  {
    adc_buff_first[i] = (uint16_t)COMPUTATION_DUALMODEINTERLEAVED_ADCMASTER_RESULT(adc_full_buff[i]); // Hall sensor1
    adc_buff_second[i] = (uint16_t)COMPUTATION_DUALMODEINTERLEAVED_ADCSLAVE_RESULT(adc_full_buff[i]); // Hall sensor2

    adc_buff_third[i] = adc_dual_channel[i * 2];      // Current 1
    adc_buff_fourth[i] = adc_dual_channel[i * 2 + 1]; // Current 2
  }
  // Give information to main function when DMA is finished
  adc_flag = RESET;
  // Set the flag for starting angle measurement
  send_flag_raw = 1;

  filter_function_complete();

}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) // HALF COMPLETE
{
  SCB_InvalidateDCache_by_Addr(((uint32_t *)&adc_full_buff[0]), (4 * DMA_ADC_BUFF_SIZE / 2));    // Invalidate the cache to get the most recent data
  SCB_InvalidateDCache_by_Addr(((uint32_t *)&adc_dual_channel[0]), (8 * DMA_ADC_BUFF_SIZE / 2)); // Invalidate the cache to get the most recent data

  for (uint32_t i = 0; i < (DMA_ADC_BUFF_SIZE / 2); i++) // ERRROR : MIGHT HAVE SOMETHING TO DO WITH FOR LOOP
  {
    adc_buff_first[i] = (uint16_t)COMPUTATION_DUALMODEINTERLEAVED_ADCMASTER_RESULT(adc_full_buff[i]); // Hall sensor 1
    adc_buff_second[i] = (uint16_t)COMPUTATION_DUALMODEINTERLEAVED_ADCSLAVE_RESULT(adc_full_buff[i]); // Hall sensor 2

    adc_buff_third[i] = adc_dual_channel[i * 2];      // Current 1
    adc_buff_fourth[i] = adc_dual_channel[i * 2 + 1]; // Current 2
  }
  /* Debugging purpose*/
  half_callback++;

  // Give information to main function when DMA is half finished
  adc_flag = SET;



  filter_function_half();
}


// TO BE CONTINUED, IGNORE THIS 
void TIM2_IRQHandler(void)
{
  TIM2->SR = 0;
  interrupt_flag = 1;
}


// TO BE CONTINUED, IGNORE THIS 
void tim_2_callback(void)
{
  //	HAL_ADC_Stop_DMA(&hadc3);
  //	HAL_ADCEx_MultiModeStop_DMA(&hadc1); // Stop DMA for ADC1
  //	HAL_ADC_Stop_DMA(&hadc2); // Stop DMA for ADC2 (if applicable)
  ////
  //	HAL_DACEx_DualStop_DMA(&hdac1, DAC_CHANNEL_1);
  //	HAL_DACEx_DualStop_DMA(&hdac1, DAC_CHANNEL_2);
  interrupt_callback++;

  // DISABLE TIMERS

  //
  //	TIM5->CR1 &= ~(TIM_CR1_CEN);
  //	TIM5->CNT = 0;
  //
  ////	TIM4->SR &= ~(1 << 0);
  //	TIM5->SR &= ~(1 << 0);

  TIM2->CR1 &= ~(TIM_CR1_CEN); // Stop the timer
  TIM2->CNT = 0;
  TIM2->DIER &= ~(TIM_DIER_UIE); // Disable update interrupt
}




void filter_function_half(void)
{

  __IO uint32_t temp_32_half[DMA_ADC_BUFF_SIZE / 2];
  float32_t temp_float_half[DMA_ADC_BUFF_SIZE / 2];

  for (int i = 0; i < DMA_ADC_BUFF_SIZE / 2; i++)
  {
    temp_32_half[i] = adc_buff_first[i];
    temp_float_half[i] = (float32_t)temp_32_half[i];
  }
  // HAL_GPIO_WritePin(Test_Pin_GPIO_Port, Test_Pin_Pin, SET);
  ConvDirect_Update(&conv1, temp_float_half);
  // HAL_GPIO_WritePin(Test_Pin_GPIO_Port, Test_Pin_Pin, RESET);

  for (int i = 0; i < DMA_ADC_BUFF_SIZE / 2; i++)
  {

    temp_32_half[i] = (uint32_t)conv1.out[i];
    send_back_1[i] = temp_32_half[i];
  }

  ///////////////////////////////////////////////////
  for (int i = 0; i < DMA_ADC_BUFF_SIZE / 2; i++)
  {
    temp_32_half[i] = adc_buff_second[i];
    temp_float_half[i] = (float32_t)temp_32_half[i];
  }

  ConvDirect_Update(&conv2, temp_float_half);

  for (int i = 0; i < DMA_ADC_BUFF_SIZE / 2; i++)
  {

    temp_32_half[i] = (uint32_t)conv2.out[i];
    send_back_2[i] = temp_32_half[i];
  }
  ////////////////////////////////////////////////

  for (int i = 0; i < DMA_ADC_BUFF_SIZE / 2; i++)
  {
    temp_32_half[i] = adc_buff_third[i];
    temp_float_half[i] = (float32_t)temp_32_half[i];
  }

  ConvDirect_Update(&conv3, temp_float_half);

  for (int i = 0; i < DMA_ADC_BUFF_SIZE / 2; i++)
  {

    temp_32_half[i] = (uint32_t)conv3.out[i];
    send_back_3[i] = temp_32_half[i];
  }
  ////////////////////////////////////////////////

  for (int i = 0; i < DMA_ADC_BUFF_SIZE / 2; i++)
  {
    temp_32_half[i] = adc_buff_fourth[i];
    temp_float_half[i] = (float32_t)temp_32_half[i];
  }

  ConvDirect_Update(&conv4, temp_float_half);

  for (int i = 0; i < DMA_ADC_BUFF_SIZE / 2; i++)
  {

    temp_32_half[i] = (uint32_t)conv4.out[i];
    send_back_4[i] = temp_32_half[i];
  }



//  HAL_GPIO_WritePin(Test_Pin_GPIO_Port, Test_Pin_Pin, RESET); // TEST PIN
  //////////////////////////////////////////////////////////
}

void filter_function_complete(void)
{

  __IO uint32_t temp_32_complete[DMA_ADC_BUFF_SIZE / 2];
  float32_t temp_float_complete[DMA_ADC_BUFF_SIZE / 2];

  for (int i = DMA_ADC_BUFF_SIZE / 2; i < DMA_ADC_BUFF_SIZE; i++)
  {
    temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2] = adc_buff_first[i];
    temp_float_complete[i - DMA_ADC_BUFF_SIZE / 2] = (float32_t)temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2];
  }

  ConvDirect_Update(&conv1, temp_float_complete);

  for (int i = DMA_ADC_BUFF_SIZE / 2; i < DMA_ADC_BUFF_SIZE; i++)
  {
    temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2] = (uint32_t)conv1.out[i - DMA_ADC_BUFF_SIZE / 2];
    send_back_1[i] = temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2];
  }
  ///////////////////////////////////////////////////////////////////////
  for (int i = DMA_ADC_BUFF_SIZE / 2; i < DMA_ADC_BUFF_SIZE; i++)
  {
    temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2] = adc_buff_second[i];
    temp_float_complete[i - DMA_ADC_BUFF_SIZE / 2] = (float32_t)temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2];
  }

  ConvDirect_Update(&conv2, temp_float_complete);

  for (int i = DMA_ADC_BUFF_SIZE / 2; i < DMA_ADC_BUFF_SIZE; i++)
  {

    temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2] = (uint32_t)conv2.out[i - DMA_ADC_BUFF_SIZE / 2];
    send_back_2[i] = temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2];
  }
  //////////////////////////////////////////////////////////
  for (int i = DMA_ADC_BUFF_SIZE / 2; i < DMA_ADC_BUFF_SIZE; i++)
  {
    temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2] = adc_buff_third[i];
    temp_float_complete[i - DMA_ADC_BUFF_SIZE / 2] = (float32_t)temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2];
  }

  ConvDirect_Update(&conv3, temp_float_complete);

  for (int i = DMA_ADC_BUFF_SIZE / 2; i < DMA_ADC_BUFF_SIZE; i++)
  {

    temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2] = (uint32_t)conv3.out[i - DMA_ADC_BUFF_SIZE / 2];
    send_back_3[i] = temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2];
  }
  ///////////////////////////////////////////////////////////////////////
  for (int i = DMA_ADC_BUFF_SIZE / 2; i < DMA_ADC_BUFF_SIZE; i++)
  {
    temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2] = adc_buff_fourth[i];
    temp_float_complete[i - DMA_ADC_BUFF_SIZE / 2] = (float32_t)temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2];
  }

  ConvDirect_Update(&conv4, temp_float_complete);

  for (int i = DMA_ADC_BUFF_SIZE / 2; i < DMA_ADC_BUFF_SIZE; i++)
  {

    temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2] = (uint32_t)conv4.out[i - DMA_ADC_BUFF_SIZE / 2];
    send_back_4[i] = temp_32_complete[i - DMA_ADC_BUFF_SIZE / 2];
  }
  ////////////////////////////////////////////////////////////////////////////
  send_flag_filter = 1;
}
//
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

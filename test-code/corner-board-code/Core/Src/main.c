/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STD_READY 0xF0
#define STD_STARTUP 0xE0

#define NST_INSTR 0xC0
#define NST_STARTUP 0xE0
#define NST_FAILED 0xD0
#define NST_READY 0xF0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
// standard transmission (1ksps) - configured once and not modified
// transmit only, no receive defined
// format: 29-bit extended identifier
// id[28:16] = 0x0001 
// id[15:12] = 0x0
// id[11:8] = addr
// id[7:4] = 0xF if successful, 0xE if setting up
// id[3:0] = sensor # 

// new format: 11-bit standard identifier
// id[10:8] = 0x1
// id[7:4] = addr
// id[3:0] = sensor #
CAN_TxHeaderTypeDef txh_std[4];

// non standard transmission (higher priority than data)
// transmit and receive - receive means control signals, transmit for feedback
// id[28:16] = 0x0000
// id[15:12] = 0x0
// id[11:8] = addr
// id[7:4] = 0xF if successful, 0xE if setting up, 0xD if failed, 0xC if instruction
// id[3:0] = instruction

// new format: 11-bit standard identifier
// id[10:8] = 0x0
// id[7:4] = addr
// id[3:0] = instruction
CAN_TxHeaderTypeDef txh_nst;
CAN_RxHeaderTypeDef rxh_nst;

CAN_FilterTypeDef filter;

// ADC data array
int32_t adc_data[4];

// configuration
uint8_t adc_configured = 0;

// address
uint8_t addr = 0x00;

// for CAN
uint32_t std_addr_base = 0x100;
uint8_t std_state = STD_READY;
uint32_t nst_addr_base = 0x000;
uint8_t nst_state = NST_READY;
uint32_t txMailbox;
uint8_t can_std_channel = 0x00;
uint8_t rx_nst_data[8];
uint8_t tx_nst_data[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim16,0);  
	while (__HAL_TIM_GET_COUNTER(&htim16) < us);  
}

void addr_setup() {  
  uint8_t a0 = HAL_GPIO_ReadPin(A0_GPIO_Port, A0_Pin) & 1;
  uint8_t a1 = HAL_GPIO_ReadPin(A1_GPIO_Port, A1_Pin) & 1;
  uint8_t a2 = HAL_GPIO_ReadPin(A2_GPIO_Port, A2_Pin) & 1;
  uint8_t a3 = HAL_GPIO_ReadPin(A3_GPIO_Port, A3_Pin) & 1;
  addr = (a3 << 3) | (a2 << 2) | (a1 << 1) | a0;
  addr = (~addr) & 0x0F; // remove if dip s
  std_addr_base = 0x100 | (addr << 4);
}

void can_setup() {
  // txh std
  for (int j = 0; j < 4; j++) {
    txh_std[j].DLC = 4; // send only top 8 bits of ADC data
    txh_std[j].StdId = std_addr_base | (addr << 4) | j; // not used
    txh_std[j].ExtId = 0; // not used std_addr_base | std_state | j;
    txh_std[j].IDE = CAN_ID_STD;
    txh_std[j].RTR = CAN_RTR_DATA;
    txh_std[j].TransmitGlobalTime = DISABLE;
  }

  // txh nst
  txh_nst.DLC = 4; // default
  txh_nst.StdId = nst_addr_base; // not used
  txh_nst.ExtId = 0; // not used nst_addr_base ;
  txh_nst.IDE = CAN_ID_STD;
  txh_nst.RTR = CAN_RTR_DATA;
  txh_nst.TransmitGlobalTime = DISABLE;

  // filter config
  filter.FilterMaskIdHigh = 0x07F0;
  filter.FilterMaskIdLow = 0x07F0;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000 | (addr << 4);
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.FilterBank = 0x00; // use first filter bank

  HAL_CAN_ConfigFilter(&hcan, &filter);
}

void can_std_transmit(uint8_t sensor) {
  uint8_t* adc_ptr = (uint8_t*) adc_data;
  uint8_t adc_data_conv[4];
  adc_data_conv[0] = adc_ptr[1];
  adc_data_conv[1] = adc_ptr[5];
  adc_data_conv[2] = adc_ptr[9];
  adc_data_conv[3] = adc_ptr[13];
  
    while (HAL_CAN_AddTxMessage(&hcan, &(txh_std[sensor]), adc_data_conv, &txMailbox) != HAL_OK) {
    // too many messages! FIXME
    delay_us(128); // delay one message
  } 
}

void can_nst_transmit(uint8_t cmd, uint8_t len) {
  txh_nst.ExtId = nst_addr_base | nst_state | cmd;
  // right now we can't transmit too many messages too quickly or they will be lost
  // we can use TX completion interrupts and a FIFO to fix this, but it may not be necessary
  if (HAL_CAN_AddTxMessage(&hcan, &txh_nst, tx_nst_data, &txMailbox) != HAL_OK) {
    // too many messages! FIXME
    while (1);
  }
}

void can_tx_transmit_timer_handler() {
  can_std_transmit(can_std_channel);
  //can_std_channel++;
  //can_std_channel %= 4;
}

void can_rx_handler() {
  // handle various commands
  // parameters are in rxh and rx_nst_data
}

uint32_t ads131m04_transfer_word(uint16_t word) {
  uint8_t lower = word & 0xff;
  uint8_t upper = word >> 8;
  uint8_t zero = 0;
  uint8_t recv[3];
  HAL_SPI_TransmitReceive(&hspi1, &upper, recv, 1, 0.1); // CMD
  HAL_SPI_TransmitReceive(&hspi1, &lower, recv+1, 1, 0.1);
  HAL_SPI_TransmitReceive(&hspi1, &zero, recv+2, 1, 0.1);
  return (recv[0] << 16) | (recv[1] << 8) | recv[2];
}

void ads131m04_transfer_frame(uint32_t* out, uint16_t* words, uint16_t tx_rx_delay) {
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
  
  for (uint8_t i = 0; i < 6; i++) {
    out[i] = ads131m04_transfer_word(words[i]);
  }
  // need to wait length of one word. at 6mhz this is 4us
  delay_us(4);
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
}

int32_t ads131m04_adc_format_convert(int32_t data){
  return ((data & 0x8000) ? -0x8000 : 0x0000) + (data & 0x7FFF);
}

void ads131m04_read_adc_nonblocking(int32_t* out) {
  uint32_t recv[24];
  uint16_t words[6] = {0, 0, 0, 0, 0, 0};
  ads131m04_transfer_frame(recv, words, 0);
  ads131m04_transfer_frame(recv+12, words, 0);
  out[0] = ads131m04_adc_format_convert(recv[1]);
  out[1] = ads131m04_adc_format_convert(recv[2]);
  out[2] = ads131m04_adc_format_convert(recv[3]);
  out[3] = ads131m04_adc_format_convert(recv[4]);
}

void ads131m04_drdy_exti_handler() {
  if (adc_configured)
    ads131m04_read_adc_nonblocking(adc_data);
}

void ads131m04_cmd(uint16_t cmd, uint32_t* out, uint16_t tx_rx_delay) {
  HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
  uint16_t words[6] = {cmd, 0, 0, 0, 0, 0};
  uint16_t zeros[6] = {0, 0, 0, 0, 0, 0};
  ads131m04_transfer_frame(out, words, tx_rx_delay);
  ads131m04_transfer_frame(out+6, zeros, 0);
  if (tx_rx_delay) delay_us(tx_rx_delay); 
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}

uint16_t ads131m04_reset() {
    uint32_t recv[12];
    ads131m04_cmd(0x0011, recv, 12); // reset time
    return recv[9];
}

uint16_t ads131m04_status() {
    uint32_t recv[12];
    ads131m04_cmd(0x0000, recv, 0);
    return recv[6] >> 8;
}

uint16_t ads131m04_standby() {
    uint32_t recv[12];
    ads131m04_cmd(0x0022, recv, 0);
    return recv[6] >> 8;
}

uint16_t ads131m04_wake() {
    uint32_t recv[12];
    ads131m04_cmd(0x0033, recv, 0);
    return recv[6] >> 8;
}

uint16_t ads131m04_lock() {
    uint32_t recv[12];
    ads131m04_cmd(0x0555, recv, 0);
    return recv[6] >> 8;
}

uint16_t ads131m04_unlock() {
    uint32_t recv[12];
    ads131m04_cmd(0x0655, recv, 0);
    return recv[6] >> 8;
}

// returns register value
uint16_t ads131m04_rreg(uint8_t reg) {
    uint16_t cmd = 0xA000 | ((reg & 0x3F) << 7);
    uint32_t recv[12];
    ads131m04_cmd(cmd, recv, 0);
    return recv[6] >> 8;
}

// read multiple registers
// returns read acknowledgement in out[0]
// returns register i in out[1+i]
void ads131m04_rreg_multiple(uint8_t start_reg, uint8_t count, uint32_t* out) {
  HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
  // if count is 1 then just use regular rreg and duplicate out[6] to out[7]
  if (count == 0) return;
  if (count == 1) {
    uint16_t cmd = 0xA000 | ((start_reg & 0x3F) << 7);
    ads131m04_cmd(cmd, out, 0);
    out[7] = out[6];
    return;
  }

  // write first frame asking to read
  uint16_t cmd = 0xA000 | ((start_reg & 0x3F) << 7) | ((count - 1) & 0x7F);
  uint16_t words[6] = {cmd, 0, 0, 0, 0, 0};
  ads131m04_transfer_frame(out, words, 0);

  // manually transfer the second frame
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
  
  // write min(6, count + 2) zero words
  uint8_t max = (count < 4) ? 6 : (count + 2);
  for (uint8_t i = 0; i < max; i++) {
    out[i] = ads131m04_transfer_word(0);
  }

  delay_us(4); // need to wait length of one word. at 6mhz this is 4us
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);

  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}

// returns write acknowledgement
uint16_t ads131m04_wreg(uint8_t reg, uint16_t data) {
  HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
  uint16_t cmd = 0x6000 | ((reg & 0x3F) << 7);
  uint32_t recv[12];
  uint16_t words[6] = {cmd, data, 0, 0, 0, 0};
  uint16_t zeros[6] = {0, 0, 0, 0, 0, 0};
  ads131m04_transfer_frame(recv, words, 0);
  ads131m04_transfer_frame(recv+6, zeros, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
  return recv[6] >> 8;
}

// returns write acknowledgement
uint16_t ads131m04_wreg_multiple(uint8_t start_reg, uint8_t count, uint16_t* data) {
  HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
  if (count == 0) return 0;

  uint16_t cmd = 0x6000 | ((start_reg & 0x3F) << 7) | ((count - 1) & 0x7F);
  // manually transfer the first frame
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);

  // write the command
  ads131m04_transfer_word(cmd);

  // write all registers to write
  for (uint8_t i = 0; i < count; i++) {
    ads131m04_transfer_word(data[i]);
  }

  // pad if not written enough (need to have written at least 6 words)
  for (int8_t i = 0; i < 4 - count; i++) {
    ads131m04_transfer_word(0);
  }

  // write one zero word
  ads131m04_transfer_word(0);

  delay_us(4); // need to wait length of one word. at 6mhz this is 4us
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);

  // transfer the second frame to get the write acknowledgement
  uint16_t zeros[6] = {0, 0, 0, 0, 0, 0};
  uint32_t recv[12];
  ads131m04_transfer_frame(recv, zeros, 0);

  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
  return recv[6] >> 8;  
}

// returns 1 if success, 0 if failed
uint8_t ads131m04_test() {
  uint16_t id = ads131m04_rreg(0x00);
  return id == 0x24;
}

// returns 1 if success, 0 if failed
uint8_t adc_configure() {
  uint8_t attempts = 0;
  while (ads131m04_reset()) {
    delay_us(10);
    attempts++;
    if (attempts > 64) break;
  };
  while (!ads131m04_test()) {
    delay_us(10);
    attempts++;
    if (attempts > 128) break;
  };
  uint16_t mode = 0x0110; // clear reset bit, disable all CRCs
  ads131m04_wreg(0x02, mode);

  uint8_t osr = 0b100; // 2048, results in a data rate of ~1.4ksps
  uint16_t clock = 0x0F03 | (osr << 2); // enable all, highest precision
  ads131m04_wreg(0x03, clock);

  uint8_t gain = 0b110; // 64
  uint16_t gain1 = (gain << 12) | (gain << 8) | (gain << 4) | gain;
  ads131m04_wreg(0x04, gain1);

  uint16_t cfg = 0x0000; // disable global chop and current detect
  ads131m04_wreg(0x06, cfg);
  
  adc_configured = 1;

  // add calibration here if we want to do that
  return 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // disable IRQ until it's ready
  HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
  HAL_NVIC_DisableIRQ(TIM14_IRQn);

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
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_Base_Start(&htim16);
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
  
  addr_setup();
  can_setup();
  adc_configure();

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_NVIC_EnableIRQ(TIM14_IRQn);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(500);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 4;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 10000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 47;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : A3_Pin */
  GPIO_InitStruct.Pin = A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(A3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A2_Pin A1_Pin A0_Pin */
  GPIO_InitStruct.Pin = A2_Pin|A1_Pin|A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DRDY_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxh_nst, rx_nst_data);
  can_rx_handler();
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

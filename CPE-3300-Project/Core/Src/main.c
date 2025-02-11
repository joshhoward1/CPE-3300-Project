/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - CPE 3300 Networking Project
  *
  * Milestones:
  * 	1. Channel Monitor - COMPLETE
  * 	2. Transmitter - COMPLETE
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Timeout period of 1.11ms
#define DELAY_WITH_TOLERANCE 1110

// max tx buffer size
#define MAX_SIZE 256

// Define wait between transmitter edges in microseconds
// Less than 500 to accommodate for hardware delays
// Measured within spec.
#define TX_WAIT 487

// Defining UART functions in terms of C standard IO functions
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


// RX Channel States
// IDLE: logic high triggered after timeout (1.11ms)
// COL: logic out triggered after timeout (1.11ms)
// BUSY: logic high or low and within timeout threshold (1.11ms)
typedef enum {
	IDLE,
	BUSY,
	COL
} channel_state_t;

// state variable holding the current state based on our custom enum type
volatile channel_state_t state;

// These variables are only temporarily global to watch in the debugging window
volatile GPIO_PinState line_level; // RX pin voltage level
volatile uint32_t TOC_compare_value; // 1.11ms offset from tic value (tic should be ideally 0)
volatile uint32_t timestamp; // offset from entering the tic ISR to reading the tic value

uint8_t tx_buffer[MAX_SIZE];
uint8_t tx_index = 0; // Character index in buffer
uint8_t bit_index = 0; // Current bit we are sending of current char
bool is_transmitting = false;
uint32_t edge_count = 0; // keep track of in TOC ISR to figure out send behavior

bool repeat_mode = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void all_leds_off();
void handle_input();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
return ch;
}

GETCHAR_PROTOTYPE
{
uint8_t ch = 0;
__HAL_UART_CLEAR_OREFLAG(&huart2);
HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

HAL_UART_Transmit(&huart2, &ch, 1, HAL_MAX_DELAY); // Echo character back
return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // For UART - clears stdin char buffer
  setvbuf(stdin, NULL, _IONBF, 0);
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Set channel_state to IDLE to start
  state = IDLE; // Initialize the state of our machine to IDLE before receiving
  HAL_GPIO_WritePin(IDLE_GPIO_Port, IDLE_Pin, GPIO_PIN_SET); // Setting IDLE LED to on

  // start input capture timer
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //printf("Enter chars: ");
  while (1)
  {
	  // Milestone 2 for demo purposes
	  printf("Enter input: ");
	  scanf("%s", tx_buffer);

	  // If ready to initialize TX, check that state is not BUSY or COL
	  if (state != BUSY && state != COL) {
		  handle_input();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart2.Init.BaudRate = 57600;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IDLE_Pin|BUSY_Pin|COL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TX_GPIO_Port, TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IDLE_Pin BUSY_Pin COL_Pin */
  GPIO_InitStruct.Pin = IDLE_Pin|BUSY_Pin|COL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TX_Pin */
  GPIO_InitStruct.Pin = TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(TX_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 *                Milestone 1:  CHANNEL MONITOR
 * TIM1 Input Capture ISR Callback Function
 *
 * Entered any time we detect an edge on the RX pin.
 * Records the tic timestamp immediately to avoid offset from running code
 * Writes the BUSY led value to be on, also changes the state variable to BUSY
 * Starts an output capture event that will end exactly
 * 1.11ms after the edge was detected
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
	// Verifies that the correct timer and channel are active
	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		// Read TIC captured value first thing to reset counter
		// This reduces the delay that running code in between would add.
		timestamp = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

		// Stop output compare before resetting it
		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);

		// State always equals busy after we see a rising edge
		state = BUSY;

		// Turns off IDLE and COL leds
		HAL_GPIO_WritePin(COL_GPIO_Port, COL_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IDLE_GPIO_Port, IDLE_Pin, GPIO_PIN_RESET);

		// Set the BUSY led to on
		HAL_GPIO_WritePin(BUSY_GPIO_Port, BUSY_Pin, GPIO_PIN_SET);

		TOC_compare_value = timestamp + DELAY_WITH_TOLERANCE;

		// Set up the TOC capture-compare value
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, TOC_compare_value);

		// Check if TOC interrupt flag is pending
		// If it is, clear it.
		if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC2)) {
			__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC2);
		}

		// Start TOC
		HAL_TIM_OC_Start_IT(htim, TIM_CHANNEL_2);
	}
}

/**
 *                    Milestone 1: CHANNEL MONITOR
 * Triggers when there is a timeout of 1.11ms
 * This signifies that we are no longer in BUSY (IDLE or COL)
 *
 * IMPORTANT NOTE:
 * If IC is triggered while in this interrupt, it will be pending and
 * waiting to trigger, so if there was an edge detected while
 * in this loop, ideally, our system will trigger the TIC ISR immediately
 * after exiting this function
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim) {

	// CHANNEL MONITOR OC
	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		all_leds_off();

		// Checking the current logic level to determine if IDLE (1) or COL (0)
		line_level = HAL_GPIO_ReadPin(RX_GPIO_Port, RX_Pin);

		if (line_level == GPIO_PIN_SET) { // voltage is high
			state = IDLE;
			HAL_GPIO_WritePin(IDLE_GPIO_Port, IDLE_Pin, GPIO_PIN_SET);
		} else if (line_level == GPIO_PIN_RESET) { // voltage is low
			state = COL;

			// stop the TX interrupt if in collision
			HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
			is_transmitting = false;

			HAL_GPIO_WritePin(COL_GPIO_Port, COL_Pin, GPIO_PIN_SET);
		}
	}

	// Transmitter OC complete callback
	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
										__HAL_TIM_GET_COUNTER(&htim1) + TX_WAIT);
		// Collision check done in channel monitor ISR

		// Get current char we are sending and the current bit
		uint8_t current_char = tx_buffer[tx_index];

		// Move from MSB to LSB, shift the desired bit into the LSB
		// and mask with 0x1 to remove everything with LSB
		uint8_t current_bit = (current_char >> (7 - bit_index)) & 1;

		// edge_count is 0 indexed, every even edge means we are
		// sending a new bit, not the second half of a bit
		if (edge_count % 2 == 0) {
			// may need to play nice with GPIO_PINSTATE
			HAL_GPIO_WritePin(TX_GPIO_Port, TX_Pin, !current_bit);
		} else {
			HAL_GPIO_TogglePin(TX_GPIO_Port, TX_Pin);
			++bit_index;
		}

		++edge_count;

		// 2 edges per bit * 8 bits per char = 16 edges per char
		if (edge_count % 16 == 0) {
			++tx_index; // increment which character to use
			bit_index = 0; // reset which bit we are looking at to the MSB
		}

		// End of character buffer if realterm terminates with a newline
		if (tx_buffer[tx_index] == '\0') {
			edge_count = 0;
			bit_index = 0;

			if (repeat_mode) {
				tx_index = 0;
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
						__HAL_TIM_GET_COUNTER(&htim1) + TX_WAIT);
			} else {
				is_transmitting = false;
				HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
			}
		}
	}
}

// goes through and sets all pins to logic 0
void all_leds_off() {
	HAL_GPIO_WritePin(BUSY_GPIO_Port, BUSY_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_GPIO_Port, COL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IDLE_GPIO_Port, IDLE_Pin, GPIO_PIN_RESET);
}

// handles starting the OC when a character is received
void handle_input() {
	repeat_mode = true;

	if (!is_transmitting) { // Only transmit if idle
		tx_index = 0;
		bit_index = 0;
		edge_count = 0;
		is_transmitting = true;

		// Set up the TOC capture-compare value
		// to schedule the next edge
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
				__HAL_TIM_GET_COUNTER(&htim1) + TX_WAIT);
		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);
	}
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

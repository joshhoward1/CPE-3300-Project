/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - CPE 3300 Networking Project
  *
  * Milestones:
  * 	1. Channel Monitor - COMPLETE
  * 	2. Transmitter - COMPLETE
  * 	3. Receiver - COMPLETE
  * 	4. Integration - COMPLETE
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
#include <stdlib.h> // srand
//#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Timeout period of 1.11ms for channel monitor
#define DELAY_WITH_TOLERANCE 1110

#define TOLERANCE 0.0132  // +-1.32% tolerance for 1000bps RX
#define MONITOR_TIMEOUT 3000  // 10ms timeout before switching back to command mode

// max tx and rx buffer size with header (adds 6 bytes)
#define MAX_SIZE 261

// Milestone 4
#define DEVICE_ADDR 44
#define BROADCAST_ADDR 255
#define PREAMBLE 0x55
#define TRAILER 0xAA
#define NMAX 128 // Controls random backoff wait time
#define MAX_BACKOFFS 10 // number of backoffs that it will attempt
#define ONE_SEC_TO_USEC 1000000

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
volatile channel_state_t prev_state;

// Message header struct
typedef struct {
	uint8_t preamble; // 0x55
	uint8_t src_addr; // 1-byte hex, address of sender
	uint8_t dest_addr; // 1-byte hex, address of receiver
	uint8_t length; // 1-255 valid
	uint8_t crc_flag; // 0 = off, 1 = on
	uint8_t trailer; // 0xAA
} message_header_t;
message_header_t tx_header, rx_header; // save two current headers

volatile bool allow_input = true;

volatile uint16_t times_idle = 0;
volatile GPIO_PinState line_level; // RX pin voltage level
volatile uint16_t TOC_compare_value; // 1.11ms offset from tic value (tic should be ideally 0)
volatile uint16_t timestamp; // offset from entering the tic ISR to reading the tic value

// TX Variables
uint8_t tx_buffer[MAX_SIZE];
uint16_t tx_index = 0; // Character index in buffer
uint8_t bit_index = 0; // Current bit we are sending of current char
uint16_t tx_length = 0;
volatile bool is_transmitting = false;
uint32_t edge_count = 0; // keep track of in TOC ISR to figure out send behavior
bool repeat_mode = false;
volatile bool tx_complete = false; // flag that message was transmitted
volatile uint8_t num_backoffs = 0;
volatile bool was_backoff = false;
volatile bool pull_to_idle = false;
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;

// RX Variables

// Timing Variables
volatile uint16_t last_capture = 0;
volatile uint32_t period = 0;
volatile int bit_state = 0;
// Decoded RX Message Buffer
char received_message[MAX_SIZE];
volatile int received_index = 0;
// UART Input Buffer
char uart_rx_buffer[MAX_SIZE];
volatile int uart_rx_ready = 0;
// Tracks if new data arrived. If so, update the console
volatile int new_data_received = 0;
char user_input[MAX_SIZE];

// Monitor Mode Timer
volatile uint32_t last_data_time = 0;
volatile int monitor_mode_active = 0;
volatile int tx_mode_active = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void all_leds_off();
void handle_input(char* message);
void process_UART_command(char mode, uint8_t dest_addr, char* message);
void decode_bit(uint32_t pulse_width, bool initial_bit); // RX read "callback"
void print_new_data();
void strip_newline(char *str);
void schedule_backoff();
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

// These lines make the serial console a little friendlier
// Translate /r to /n for stdin, but echo /r/n for terminal
if (ch == '\r'){  // If character is CR
	ch = '\n';   // Return LF. fgets is terminated by LF
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
}

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
  prev_state = IDLE;
  HAL_GPIO_WritePin(IDLE_GPIO_Port, IDLE_Pin, GPIO_PIN_SET); // Setting IDLE LED to on

  // Initial IDLE high
  if (state == IDLE) {
    HAL_GPIO_WritePin(TX_GPIO_Port, TX_Pin, GPIO_PIN_SET);
  }
  // start input capture timer
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

  // seed srand with current timer value
  srand(__HAL_TIM_GET_COUNTER(&htim1));



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  // After transmitting, don't allow input until
	  // message is either fully transmitted or timed out
	  if (allow_input == true) {


		  uint8_t dest_address;
		  char message[255] = {0};
		  char mode;

		  // Blocking fgets() in command mode
		  printf("CMD> ");
		  //fflush(stdout);
//		  scanf(" %c", &mode);

		  //printf("Mode: %c\n", mode);
		  fgets(user_input, sizeof(user_input), stdin);
		  user_input[strcspn(user_input, "\n")] = '\0';
//		  mode = user_input[0];
//		  if (mode == 's') {
//			  printf("\nEnter node addr: ");
//			  scanf("%u", &dest_address);
//			  printf("\nEnter message: ");
//			  scanf("%s", message);
//		  } else {
//			  dest_address = 0;
//			  message[0] = '\0';
//		  }
		  //printf("Input: %s\n", user_input);


		  // Parse user input for commands
		  sscanf(user_input, "%c %u %[^\n]", &mode, &dest_address, message);

		 // strip_newline(user_input);
		  process_UART_command(mode, dest_address, message);
	  }

	  // Poll flag to know if message was sent or timed out
	  if (tx_complete == true) {
		  allow_input = true;
		  tx_complete = false;


		  if (was_backoff) {
			  printf("--> Message send failed after %u attempts\n",
					  MAX_BACKOFFS);
			  printf("Backed off and attempted retransmissions over %u ms\n", end_time - start_time);
		  } else {
			  printf("--> Message sent\n");
		  }
		  was_backoff = false;
	  }




//	  if (!monitor_mode_active && !tx_mode_active) {
//
//	  }
//	  } else if (monitor_mode_active && !tx_mode_active) {
//		  // Monitor mode: Non-blocking display of new data
//		  if (new_data_received) {
//			  print_new_data();
//		  }
//
//		  // Check if 10ms have passed with no new data
//		  // if so, console gives control back to user
//		  if (HAL_GetTick() - last_data_time > MONITOR_TIMEOUT) {
//			  monitor_mode_active = 0;
//			  printf("Switching back to command mode.\n");
//		  }
//	  } else if (tx_mode_active && !monitor_mode_active) {
//		  printf("Type something to TX pin: ");
//		  scanf("%s", tx_buffer);
//		  // If ready to initialize TX, check that state is not BUSY or COL
//		  if (state != BUSY && state != COL) {
//			  handle_input();
//		  }
//		  tx_mode_active = 0;
//	  }
	  // Milestone 2 for demo purposes
//	  printf("Enter input: ");
//	  scanf("%s", tx_buffer);
//
//	  // If ready to initialize TX, check that state is not BUSY or COL
//	  if (state != BUSY && state != COL) {
//		  handle_input();
//	  }


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
  HAL_GPIO_WritePin(GPIOA, IDLE_Pin|BUSY_Pin|COL_Pin|ISR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TX_GPIO_Port, TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IDLE_Pin BUSY_Pin COL_Pin ISR_Pin */
  GPIO_InitStruct.Pin = IDLE_Pin|BUSY_Pin|COL_Pin|ISR_Pin;
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
//		HAL_GPIO_WritePin(ISR_GPIO_Port, ISR_Pin, GPIO_PIN_SET);

		// State always equals busy after we see a rising edge
		prev_state = state;
		state = BUSY;

		/**
		 * RX Behavior
		 */
		uint16_t pulse_width;
		bool initial_bit = false;
		if (prev_state == IDLE) {
			++times_idle;
			initial_bit = true;
		}

		pulse_width = timestamp - last_capture;
		last_capture = timestamp;

		decode_bit(pulse_width, initial_bit); // figure out if 1T or 2T has passed

		/**
		 * Channel monitor behavior
		 */
		// Stop output compare before resetting it
		HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);



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
//		HAL_GPIO_WritePin(ISR_GPIO_Port, ISR_Pin, GPIO_PIN_RESET);
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
			HAL_GPIO_WritePin(COL_GPIO_Port, COL_Pin, GPIO_PIN_SET);
		}
	}

	// Transmitter OC complete callback
	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		// For monitoring TX vs Backoffs
		//HAL_GPIO_WritePin(ISR_GPIO_Port, ISR_Pin, GPIO_PIN_SET);

		// If we are in the last one, we need to pull back high
		if (pull_to_idle == true) {
			HAL_GPIO_WritePin(TX_GPIO_Port, TX_Pin, GPIO_PIN_SET);

			// if not in last one, do normal behaviors
		} else {
			// schedules next edge (assuming not in COL)
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
											__HAL_TIM_GET_COUNTER(&htim1) + TX_WAIT);

			// Collision check done in channel monitor ISR
			if (state == COL) {
				if (num_backoffs == 0) {
					start_time = HAL_GetTick();
				}
				HAL_GPIO_WritePin(ISR_GPIO_Port, ISR_Pin, GPIO_PIN_SET);
				HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
				// quit trying to re-transmit
				if (num_backoffs == MAX_BACKOFFS) {
					end_time = HAL_GetTick();
					tx_complete = true;
					was_backoff = true;

					num_backoffs = 0;
					tx_index = 0;
					edge_count = 0;
					bit_index = 0;
					is_transmitting = false;

					HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
					// "clears" the TX buffer
					tx_buffer[0] = '\0';

					HAL_GPIO_WritePin(ISR_GPIO_Port, ISR_Pin, GPIO_PIN_RESET);
					return;
				}

				// schedule a backoff
				schedule_backoff();
				return;
			}

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

		}

		// End of tx buffer is message length + header
		if (tx_index == tx_length && !pull_to_idle) {
			pull_to_idle = true;
			// schedule one more edge to pull high
			HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
							__HAL_TIM_GET_COUNTER(&htim1) + TX_WAIT);
			HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);
			HAL_GPIO_WritePin(ISR_GPIO_Port, ISR_Pin, GPIO_PIN_RESET);
			return;

		} else if (tx_index == tx_length && pull_to_idle == true) {
			pull_to_idle = false;
			// Pulled to logic high and we are fully done
			edge_count = 0;
			bit_index = 0;

			// Flag to main program to print that the message was successfully sent
			tx_complete = true;
			was_backoff = false;

			if (repeat_mode) {
				tx_index = 0;
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
						__HAL_TIM_GET_COUNTER(&htim1) + TX_WAIT);
				HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);
			} else {
				is_transmitting = false;
				HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
			}
		}
	}
	HAL_GPIO_WritePin(ISR_GPIO_Port, ISR_Pin, GPIO_PIN_RESET);
}



//// UART Interrupt Callback
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//    if (huart->Instance == USART2) {
//        uart_rx_ready = 1;  // Signal main loop that a command is ready
//        // Restart interrupt
//        HAL_UART_Receive_IT(&huart2, (uint8_t*)uart_rx_buffer, MAX_SIZE);
//    }
//}

// goes through and sets all pins to logic 0
void all_leds_off() {
	HAL_GPIO_WritePin(BUSY_GPIO_Port, BUSY_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_GPIO_Port, COL_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IDLE_GPIO_Port, IDLE_Pin, GPIO_PIN_RESET);
}

// Processes Console Input
void process_UART_command(char mode, uint8_t dest_addr, char* message) {
	//printf("mode: %c\n", mode);
	switch (mode) {
		case 's':
			if (dest_addr != 255) {
				printf("--> Sending message to node %d: \"%s\"\n",
						dest_addr, message);
			} else {
				printf("--> Broadcasting message: \"%s\"\n", message);
			}

			// Schedule a TX
			// It will handle backoffs if necessary

			// Set up for TX header
			tx_header.preamble = PREAMBLE;
			tx_header.src_addr = DEVICE_ADDR;
			tx_header.dest_addr = dest_addr;
			tx_header.length = strlen(message);
			tx_header.crc_flag = 0;
			tx_header.trailer = TRAILER;

			allow_input = false;
			handle_input(message);

			break;
		case 'r':
			print_new_data();
			break;
		default:
			printf("--> Command not recognized.\n");
			break;
	}
}


void print_new_data() {
	// Prints only new data received if RX buffer has a message
	// and its dest_addr is a valid address to read from

	//printf("%d\n", received_message[2]);
    if (new_data_received &&
    		(received_message[2] == BROADCAST_ADDR
          || received_message[2] == DEVICE_ADDR))
    {
    	rx_header.length = received_message[3];
    	rx_header.src_addr = received_message[1];

        printf("--> Received message from node %u: ", rx_header.src_addr);
        // Print only up to the message length
	    for (uint8_t i = 0; i < rx_header.length; i++) {
		    putchar(received_message[5 + i]);
	    }
	    putchar('\n');
    } else {
    	printf("--> No messages received.\n");
    }

    new_data_received = 0;  // Clear flag
    for (uint16_t i = 0; i < sizeof(received_message); ++i) {
    	received_message[i] = 0;
    }
    received_index = 0;
}

void schedule_backoff() {
	++num_backoffs;
	// Get randomly generated wait time in microseconds
	uint32_t N = (rand() % (NMAX - 1) + 1);
	uint32_t w = (N / ((float)NMAX)) * ONE_SEC_TO_USEC;

	// schedule TX for w microseconds later
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
					__HAL_TIM_GET_COUNTER(&htim1) + w);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);
}

// handles starting the OC when a character is received
void handle_input(char* message) {
	repeat_mode = false; // TODO: THIS MAY NOT WORK

	// These offsets will always be in these spots
	tx_buffer[0] = tx_header.preamble;
	tx_buffer[1] = tx_header.src_addr;
	tx_buffer[2] = tx_header.dest_addr;
	tx_buffer[3] = tx_header.length;
	tx_buffer[4] = tx_header.crc_flag;

	// assuming message size is valid
	memcpy(tx_buffer + 5, message, tx_header.length);
	tx_buffer[5 + tx_header.length] = tx_header.trailer;

	// Message + header length
	tx_length = strlen(message) + 6;

	if (!is_transmitting) { // do initial setup if not currently transmitting
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

// Remove trailing '\r' or '\n' if present
void strip_newline(char *str) {
    size_t len = strlen(str);
    if (len > 0 && (str[len - 1] == '\n' || str[len - 1] == '\r')) {
        str[len - 1] = '\0';  // Replace it with null terminator
    }
}

// Decodes Manchester signal timing
void decode_bit(uint32_t pulse_width, bool initial_bit) {

	// Store received bit in buffer
	static uint8_t bit_counter = 0;
	static uint8_t byte_buffer = 0;
	static uint8_t one_t_counter = 0;

	// Start from known 0 after IDLE (true because of preamble)
	if (initial_bit == true) {
		bit_state = 0;
		// Shift byte to the left and append bit at LSB
		byte_buffer = ((byte_buffer << 1) | bit_state);
		bit_counter++;
		return;
	}

	// widened timing window to reject less input
    if ((pulse_width > 900) && (pulse_width < 1100)) {
        // Detected 2T period: Bit flip
        bit_state = !bit_state;
        one_t_counter = 0;
    } else if ((pulse_width > 400) && (pulse_width < 600)) {
    	// else, bit is the same as last period
    	++one_t_counter;
    	if (one_t_counter == 1) {
    		return;
    	}
	} else {
		return;
	}


    // Shift byte to the left and append bit at LSB
    byte_buffer = ((byte_buffer << 1) | bit_state);
    bit_counter++;

    one_t_counter = 0;

    if (bit_counter == 8) {
        // Store completed byte
        received_message[received_index++] = byte_buffer;
        received_message[received_index] = '\0';  // Null terminate string
        bit_counter = 0;
        byte_buffer = 0;
        new_data_received = 1;  // Flag new data
        last_data_time = HAL_GetTick();  // Reset monitor timeout timer

        // Prevent buffer overflow
        if (received_index >= MAX_SIZE - 1) {
        	received_index = 0;
        }

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

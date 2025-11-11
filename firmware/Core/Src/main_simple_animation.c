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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ====================================================================
 * MATRIX DIMENSIONS
 * ==================================================================== */
#define MATRIX_ROWS 20
#define MATRIX_COLS 19
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ====================================================================
 * FRAME BUFFER
 * ==================================================================== */
/**
 * @brief This is the 2D array representing your LED matrix.
 * frame_buffer[row][col]
 * A '1' means the LED at (row, col) will be ON.
 * A '0' means the LED at (row, col) will be OFF.
 */
uint8_t frame_buffer[MATRIX_ROWS][MATRIX_COLS] = {0};

/* ====================================================================
 * RENDER STATE
 * ==================================================================== */
// Variables to keep track of the currently lit LED for efficient POV
static int last_anode = -1;
static int last_cathode = -1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
/* ====================================================================
 * USER FUNCTION PROTOTYPES
 * ==================================================================== */
void DWT_Delay_Init(void);
void DWT_Delay_us(volatile uint32_t us);
void enable_gpio_clocks(void);
void set_all_pins_input(void);
void turn_off_led(int anode_index, int cathode_index);
void light_led_fast(int anode_index, int cathode_index);
void update_frame_animation(uint32_t tick); // <-- CHANGED
void render_frame(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * All of your existing helper functions are here.
 * (DWT_Delay, pin definitions, and register
 * functions are all preserved).
*/

#include "main.h" // Should be present from CubeIDE
#include <stdint.h>
/* --- Add these includes for DWT --- */
#include "core_cm3.h" // For DWT, CoreDebug

/* ====================================================================
 * 1. PIN DEFINITIONS
 * ==================================================================== */

#define NUM_PINS 20

// Structure to hold pin information
typedef struct {
    GPIO_TypeDef* port; // GPIO Port (e.g., GPIOA, GPIOB)
    uint8_t       pin_num;  // Pin number (0-15)
} GpioPin;

// Array mapping your 20 pins to their ports and numbers
// This order MUST match your physical wiring.
// USER-PROVIDED ORDER:
// PB8, PB9, PC13, PC14, PC15, PD0, PD1, PA0, PA1, PA2,
// PA3, PA4, PA5, PA6, PA7, PB0, PB1, PB2, PB10, PB11
const GpioPin matrix_pins[NUM_PINS] = {
    {GPIOB, 8},  // Pin 0: PB8
    {GPIOB, 9},  // Pin 1: PB9
    {GPIOC, 13}, // Pin 2: PC13
    {GPIOC, 14}, // Pin 3: PC14
    {GPIOC, 15}, // Pin 4: PC15
    {GPIOD, 0},  // Pin 5: PD0
    {GPIOD, 1},  // Pin 6: PD1
    {GPIOA, 0},  // Pin 7: PA0
    {GPIOA, 1},  // Pin 8: PA1
    {GPIOA, 2},  // Pin 9: PA2
    {GPIOA, 3},  // Pin 10: PA3
    {GPIOA, 4},  // Pin 11: PA4
    {GPIOA, 5},  // Pin 12: PA5
    {GPIOA, 6},  // Pin 13: PA6
    {GPIOA, 7},  // Pin 14: PA7
    {GPIOB, 0},  // Pin 15: PB0
    {GPIOB, 1},  // Pin 16: PB1
    {GPIOB, 2},  // Pin 17: PB2
    {GPIOB, 10}, // Pin 18: PB10
    {GPIOB, 11}  // Pin 19: PB11
};


/* ====================================================================
 * 2. LOW-LEVEL GPIO REGISTER FUNCTIONS (From User)
 * ==================================================================== */

/**
 * @brief Enables the GPIO clocks for ports A, B, C, and D.
 */
void enable_gpio_clocks(void) {
    // Enable clocks for GPIOA, GPIOB, GPIOC, GPIOD in the APB2ENR register
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN |
                    RCC_APB2ENR_IOPDEN;
}

/**
 * @brief Sets a specific pin to be a high-impedance (Hi-Z) input.
 * @note  We use Analog Input mode (MODE=00, CNF=00) as it's the
 * best way to achieve Hi-Z and disable the digital input buffer.
 */
void set_pin_input(GPIO_TypeDef* port, uint8_t pin_num) {
    if (pin_num < 8) { // Use CRL register
        uint8_t shift = pin_num * 4;
        port->CRL &= ~(0xF << shift); // Clear MODE and CNF bits
        port->CRL |= (0x0 << shift);  // Set MODE=00, CNF=00 (Analog Input)
    } else { // Use CRH register
        uint8_t shift = (pin_num - 8) * 4;
        port->CRH &= ~(0xF << shift); // Clear MODE and CNF bits
        port->CRH |= (0x0 << shift);  // Set MODE=00, CNF=00 (Analog Input)
    }
}

/**
 * @brief Sets a specific pin to be a push-pull output (50MHz).
 */
void set_pin_output(GPIO_TypeDef* port, uint8_t pin_num) {
    if (pin_num < 8) { // Use CRL register
        uint8_t shift = pin_num * 4;
        port->CRL &= ~(0xF << shift); // Clear MODE and CNF bits
        port->CRL |= (0x3 << shift);  // Set MODE=11 (50MHz), CNF=00 (GP Output PP)
    } else { // Use CRH register
        uint8_t shift = (pin_num - 8) * 4;
        port->CRH &= ~(0xF << shift); // Clear MODE and CNF bits
        port->CRH |= (0x3 << shift);  // Set MODE=11 (50MHz), CNF=00 (GP Output PP)
    }
}

/**
 * @brief Sets a specific pin HIGH using the atomic BSRR register.
 */
void set_pin_high(GPIO_TypeDef* port, uint8_t pin_num) {
    port->BSRR = (1 << pin_num);
}

/**
 * @brief Sets a specific pin LOW using the atomic BRR register.
 */
void set_pin_low(GPIO_TypeDef* port, uint8_t pin_num) {
    port->BRR = (1 << pin_num);
}

/* ====================================================================
 * 3. TIMING FUNCTIONS (From User)
 * ==================================================================== */

/**
 * @brief Initializes the DWT unit for a precise microsecond delay.
 */
void DWT_Delay_Init(void) {
    // Enable TRCENA
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Enable the CYCCNT counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Reset the counter
    DWT->CYCCNT = 0;
}

/**
 * @brief Provides a blocking delay in microseconds.
 * @param us: Number of microseconds to delay
 */
void DWT_Delay_us(volatile uint32_t us) {
    uint32_t start = DWT->CYCCNT;

    // Calculate the number of CPU cycles per microsecond
    // HAL_RCC_GetHCLKFreq() gets the clock speed (e.g., 72,000,000)
    uint32_t cycles_per_us = (HAL_RCC_GetHCLKFreq() / 1000000);
    uint32_t count = us * cycles_per_us;

    while ((DWT->CYCCNT - start) < count);
}


/* ====================================================================
 * 4. CHARLIEPLEXING LOGIC (Modified from User)
 * ==================================================================== */

/**
 * @brief Sets all 20 matrix pins to high-impedance input mode.
 * This is the "all off" state for charlieplexing.
 */
void set_all_pins_input(void) {
    for (int i = 0; i < NUM_PINS; i++) {
        set_pin_input(matrix_pins[i].port, matrix_pins[i].pin_num);
    }
}

/**
 * @brief Sets the two pins for a specific LED back to input (Hi-Z).
 * (From User)
 */
void turn_off_led(int anode_index, int cathode_index) {
    if (anode_index < 0 || cathode_index < 0 || anode_index >= NUM_PINS || cathode_index >= NUM_PINS) return;

    set_pin_input(matrix_pins[anode_index].port, matrix_pins[anode_index].pin_num);
    set_pin_input(matrix_pins[cathode_index].port, matrix_pins[cathode_index].pin_num);
}


/**
 * @brief Lights a single LED *efficiently* without resetting all pins.
 * @note This is the function POV rendering should use.
 * @param anode_index The index (0-19) of the pin to set HIGH.
 * @param cathode_index The index (0-19) of the pin to set LOW.
 */
void light_led_fast(int anode_index, int cathode_index) {
    // 1. Get the port and pin for the Anode
    GPIO_TypeDef* anode_port = matrix_pins[anode_index].port;
    uint8_t       anode_pin  = matrix_pins[anode_index].pin_num;

    // 2. Get the port and pin for the Cathode
    GPIO_TypeDef* cathode_port = matrix_pins[cathode_index].port;
    uint8_t       cathode_pin  = matrix_pins[cathode_index].pin_num;

    // 3. Configure Anode as Output and set it HIGH
    set_pin_output(anode_port, anode_pin);
    set_pin_high(anode_port, anode_pin);

    // 4. Configure Cathode as Output and set it LOW
    set_pin_output(cathode_port, cathode_pin);
    set_pin_low(cathode_port, cathode_pin);
}

/**
 * @brief Lights a single LED (safe version).
 * @note This function is less efficient for POV but good for testing
 * as it guarantees all other pins are off.
 * @param anode_index The index (0-19) of the pin to set HIGH.
 * @param cathode_index The index (0-19) of the pin to set LOW.
 */
void light_led(int anode_index, int cathode_index) {
    // 1. Set ALL pins to Hi-Z first
    set_all_pins_input();

    // 2. Light the specific LED
    light_led_fast(anode_index, cathode_index);
}


/**
 * @brief Call this function from main() to test all LEDs.
 * It will loop forever, scanning one LED at a time.
 * (From User)
 */
void test_all_leds(void) {
    // Ensure all GPIO clocks are enabled
    enable_gpio_clocks();

    while (1) {
        // Loop through all pins as anodes
        for (int i = 0; i < NUM_PINS; i++) {
            // Loop through all pins as cathodes
            for (int j = 0; j < NUM_PINS; j++) {
                // Skip if anode and cathode are the same pin
                if (i == j) {
                    continue;
                }

                // Light the LED defined by (anode i, cathode j)
                light_led(i, j);

                // Keep it on for a short time to see it
                // This is a HAL delay, which is fine for slow testing.
                HAL_Delay(5);
            }
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

  /* USER CODE BEGIN 1 */
  // This static variable will keep track of our animation "time"
  static uint32_t anim_tick = 0;
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
  /* USER CODE BEGIN 2 */

  // 1. Enable clocks for all our GPIO ports
  enable_gpio_clocks();

  // 2. Initialize the high-precision delay timer
  DWT_Delay_Init();

  // 3. Fill the frame_buffer with the *first* animation frame
  update_frame_animation(0); // <-- CHANGED

  // 4. Set all pins to input (Hi-Z) to start
  set_all_pins_input();

  // 5. Uncomment this line to test LEDs one-by-one
  // test_all_leds();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// This one function call will scan and render
	// the entire frame_buffer onto your matrix.
	render_frame();

	// After a full frame is rendered, calculate
	// the *next* frame for the animation.
	update_frame_animation(anim_tick++);
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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* NOTE: This function is generated by CubeMX.
   * Our code uses direct register access and calls
   * enable_gpio_clocks() separately, so this
   * function may be incomplete for our matrix,
   * but we leave it as CubeMX generated it.
   */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Fills the global frame_buffer with a moving stripe pattern.
 * @param tick: A frame counter to drive the animation.
 */
void update_frame_animation(uint32_t tick)
{
    // This makes the animation move 1 pixel every frame.
    // To slow it down, you could use (tick / 10)
    uint32_t time = tick;

    for(int r = 0; r < MATRIX_ROWS; r++)
    {
        for(int c = 0; c < MATRIX_COLS; c++)
        {
            // Create diagonal stripes that move
            // (r + c + time) % 8 creates a stripe pattern
            // The '== 0' lights up one of those stripes
            if ( ((r + c + time) % 8) == 0 )
            {
                frame_buffer[r][c] = 1;
            }
            else
            {
                frame_buffer[r][c] = 0;
            }
        }
    }
}


/**
 * @brief Renders the global 'frame_buffer' to the matrix using POV.
 * This function is designed to be called continuously in the main loop.
 */
void render_frame(void)
{
    // This is the "on" time for each LED in microseconds.
    // TUNE THIS VALUE:
    //  - Higher = Brighter LEDs, but more flicker (slower refresh)
    //  - Lower  = Dimmer LEDs, but less flicker (faster refresh)
    // 39us gives ~67Hz refresh rate for all 380 LEDs.
    const uint32_t delay_per_led = 80;

    // Iterate over every pixel in our 20x19 logical frame
    for (int r = 0; r < MATRIX_ROWS; r++)
    {
        for (int c = 0; c < MATRIX_COLS; c++)
        {
            // 1. Turn OFF the *previous* LED
            //    (This is efficient, only 2 pin reconfigs)
            turn_off_led(last_anode, last_cathode);

            // 2. Check if the current pixel should be ON
            if (frame_buffer[r][c])
            {
                // 3. If YES, calculate the physical pin mapping
                //    This is the core of the charlieplex logic.
                //    'r' (0-19) maps to the anode pin.
                //    'c' (0-18) maps to the cathode pin, skipping the anode pin.

                int anode_pin = r;
                int cathode_pin = (c < r) ? c : (c + 1);

                // 4. Light the new LED
                light_led_fast(anode_pin, cathode_pin);

                // 5. Remember which LED is on
                last_anode = anode_pin;
                last_cathode = cathode_pin;
            }
            else
            {
                // 6. If NO, no LED is on, so "remember" nothing
                last_anode = -1;
                last_cathode = -1;
            }

            // 7. Hold this state (either on or off) for one time slot
            DWT_Delay_us(delay_per_led);
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

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include "main.h" // Should be present from CubeIDE
#include <stdint.h>

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
const GpioPin matrix_pins[NUM_PINS] = {
    {GPIOA, 0},  // Pin 0: PA0
    {GPIOA, 1},  // Pin 1: PA1
    {GPIOA, 2},  // Pin 2: PA2
    {GPIOA, 3},  // Pin 3: PA3
    {GPIOA, 4},  // Pin 4: PA4
    {GPIOA, 5},  // Pin 5: PA5
    {GPIOA, 6},  // Pin 6: PA6
    {GPIOA, 7},  // Pin 7: PA7
    {GPIOD, 0},  // Pin 8: PD0
    {GPIOD, 1},  // Pin 9: PD1
    {GPIOC, 13}, // Pin 10: PC13
    {GPIOC, 14}, // Pin 11: PC14
    {GPIOC, 15}, // Pin 12: PC15
    {GPIOB, 0},  // Pin 13: PB0
    {GPIOB, 1},  // Pin 14: PB1
    {GPIOB, 2},  // Pin 15: PB2
    {GPIOB, 8},  // Pin 16: PB8
    {GPIOB, 9},  // Pin 17: PB9
    {GPIOB, 10}, // Pin 18: PB10
    {GPIOB, 11}  // Pin 19: PB11
};

/* ====================================================================
 * 2. LOW-LEVEL GPIO REGISTER FUNCTIONS
 * ==================================================================== */

/**
 * @brief Enables the GPIO clocks for ports A, B, C, and D.
 * @note  CubeIDE usually does this in MX_GPIO_Init(), but it's
 * good practice to ensure it when using direct registers.
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
 * 3. CHARLIEPLEXING LOGIC
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
 * @brief Lights a single LED.
 * @param anode_index The index (0-19) of the pin to set HIGH.
 * @param cathode_index The index (0-19) of the pin to set LOW.
 */
void light_led(int anode_index, int cathode_index) {
    // 1. Set ALL pins to Hi-Z first
    set_all_pins_input();

    // 2. Get the port and pin for the Anode
    GPIO_TypeDef* anode_port = matrix_pins[anode_index].port;
    uint8_t       anode_pin  = matrix_pins[anode_index].pin_num;

    // 3. Get the port and pin for the Cathode
    GPIO_TypeDef* cathode_port = matrix_pins[cathode_index].port;
    uint8_t       cathode_pin  = matrix_pins[cathode_index].pin_num;

    // 4. Configure Anode as Output and set it HIGH
    set_pin_output(anode_port, anode_pin);
    set_pin_high(anode_port, anode_pin);

    // 5. Configure Cathode as Output and set it LOW
    set_pin_output(cathode_port, cathode_pin);
    set_pin_low(cathode_port, cathode_pin);
}

/* ====================================================================
 * 4. TEST FUNCTIONS
 * ==================================================================== */

/**
 * @brief A simple (and inaccurate) busy-wait delay.
 * @note  For real applications, use a timer or HAL_Delay().
 * The inner loop count (e.g., 7000) needs to be tuned
 * for your system clock (e.g., 72MHz).
 */
void crude_delay_ms(volatile uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 7000; j++) {
            __NOP(); // No-operation assembly instruction
        }
    }
}

/**
 * @brief Call this function from main() to test all LEDs.
 * It will loop forever, scanning one LED at a time.
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
                // crude_delay_ms(1); // 10ms delay
            }
        }
    }
}

#include "main.h"
#include <stdint.h> // Make sure this is included

/* --- Add these includes for DWT --- */
#include "core_cm3.h" // For DWT, CoreDebug

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

/**
 * @brief Call this from main() to light all LEDs using POV.
 * This function loops forever.
 */
void light_all_leds_pov(void) {
    // This is the "on" time for each LED in microseconds.
    // Feel free to tune this value.
    // 39us gives a frame rate of ~60Hz on a 72MHz clock.
    const uint32_t delay_per_led = 39; // in microseconds

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

                // 1. Turn on the specific LED
                light_led(i, j);

                // 2. Keep it on for a tiny amount of time
                DWT_Delay_us(delay_per_led);

                // 3. The light_led() function will be called again,
                //    and its first step is set_all_pins_input(),
                //    which turns this LED off before lighting the next.
            }
        }
        // The loop repeats, refreshing the "frame"
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
  /* USER CODE BEGIN 2 */

  test_all_leds();
  // light_all_leds_pov();

  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

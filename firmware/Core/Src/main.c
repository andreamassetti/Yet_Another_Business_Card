/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  * @note
  * This version fixes the progressive dimming bug.
  *
  * BUG: The DWT_Delay_us() timer overflows after ~119 seconds,
  * causing the main loop to run at max speed, which re-introduces
  * the race condition (dimming/ghosting).
  *
  * FIX: Removed all DWT code. We now call HAL_IncTick() from
  * our custom SysTick_Handler and use the standard,
  * overflow-proof HAL_Delay(5) in the main loop.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> // For memset
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Structure to hold pin information
typedef struct {
    GPIO_TypeDef* port; // GPIO Port (e.g., GPIOA, GPIOB)
    uint8_t       pin_num;  // Pin number (0-15)
} GpioPin;

// This struct holds the *entire* pre-calculated
// state for a single GPIO port.
typedef struct {
    uint32_t crl;  // Value for the CRL register
    uint32_t crh;  // Value for the CRH register
    uint32_t bsrr; // Value for the BSRR register (pin HIGH)
    uint32_t brr;  // Value for the BRR register (pin LOW)
} GpioPortState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ====================================================================
 * MATRIX DIMENSIONS & GAME DEFINES (UPGRADED FOR 20x19)
 * ==================================================================== */
#define MATRIX_ROWS 20
#define MATRIX_COLS 19
#define NUM_PINS 20 // 20 pins drive 20*19 = 380 LEDs

#define PADDLE_WIDTH 7 // Paddle width for the new grid
#define PADDLE_Y (MATRIX_ROWS - 1) // Paddle is on the last row (row 19)
#define BRICK_ROWS 8 // 8 rows of bricks

/* ====================================================================
 * BCM (BRIGHTNESS) DRIVER DEFINES (1-BIT)
 * ==================================================================== */
#define COLOR_DEPTH 1           // 1-bit brightness (on/off). Fixes RAM overflow.
#define COLOR_COUNT (1 << COLOR_DEPTH)
#define COLOR_MAX (COLOR_COUNT - 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ====================================================================
 * FRAME BUFFER (Now 20x19)
 * ==================================================================== */
// This buffer now stores 0 (off) or 1 (on)
uint8_t frame_buffer[MATRIX_ROWS][MATRIX_COLS] = {0};

/* ====================================================================
 * PIN DEFINITIONS (Full 20-pin list)
 * ==================================================================== */
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
 * DOUBLE-BUFFERED COMPILED FRAME "LUT" (Now 2 * 1.25KB = 2.5KB RAM)
 * ==================================================================== */
// Buffer A: The "Front Buffer" that the ISR reads from
GpioPortState compiledFrame_A[COLOR_DEPTH][NUM_PINS][4];
// Buffer B: The "Back Buffer" that main() writes to
GpioPortState compiledFrame_B[COLOR_DEPTH][NUM_PINS][4];

// Pointers to swap the buffers
// g_ActiveFrame is read by the ISR. It *must* be volatile.
volatile GpioPortState* g_ActiveFrame = (volatile GpioPortState*)compiledFrame_A;
// g_BackBuffer is written by main()
volatile GpioPortState* g_BackBuffer  = (volatile GpioPortState*)compiledFrame_B;


// Default "all pins input" state for each port's config registers
const uint32_t GPIO_CRL_ALL_INPUT = 0x44444444;
const uint32_t GPIO_CRH_ALL_INPUT = 0x44444444;

/* ====================================================================
 * BCM (BRIGHTNESS) DRIVER STATE
 * ==================================================================== */
uint32_t bitDuration[COLOR_DEPTH]; // Holds the "on-time" for the single bit-plane
static volatile int g_current_bit = 0;   // Will always be 0
static volatile int g_current_anode = 0; // Which anode pin we are driving (0-19)

/* ====================================================================
 * GAME STATE
 * ==================================================================== */
static uint8_t bricks[BRICK_ROWS][MATRIX_COLS];
static int ball_x, ball_y;
static int ball_vx, ball_vy; // velocity
static int paddle_x;
static uint8_t game_over = 0;
static uint32_t score = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
/* ====================================================================
 * USER FUNCTION PROTOTYPES
 * ==================================================================== */
// Core hardware drivers
// void DWT_Delay_Init(void); // No longer needed
// void DWT_Delay_us(volatile uint32_t us); // No longer needed
void enable_gpio_clocks(void);
void set_pin_input(GPIO_TypeDef* port, uint8_t pin_num); // Still needed
void set_all_pins_input(void); // Still needed
void initBCM(void);
void convertFrame(volatile GpioPortState* buffer); // Now takes a pointer

// Button input
void init_button(void);
uint8_t read_button(void);

// Game logic
void game_init(void);
void update_game_state(uint32_t tick);
void draw_game_to_framebuffer(void);
void draw_game_over_screen(uint32_t tick);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ====================================================================
 * 1. TIMING FUNCTIONS (REMOVED DWT)
 * ==================================================================== */
// DWT_Delay_Init() and DWT_Delay_us() removed.

/* ====================================================================
 * 2. LOW-LEVEL GPIO REGISTER FUNCTIONS (Simplified)
 * ==================================================================== */
void enable_gpio_clocks(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN |
                    RCC_APB2ENR_IOPDEN;
}
void set_pin_input(GPIO_TypeDef* port, uint8_t pin_num) {
    if (pin_num < 8) {
        uint8_t shift = pin_num * 4;
        port->CRL &= ~(0xF << shift);
        port->CRL |= (0x4 << shift); // 0b0100: Floating Input
    } else {
        uint8_t shift = (pin_num - 8) * 4;
        port->CRH &= ~(0xF << shift);
        port->CRH |= (0x4 << shift);
    }
}
void set_all_pins_input(void) {
    for (int i = 0; i < NUM_PINS; i++) {
        set_pin_input(matrix_pins[i].port, matrix_pins[i].pin_num);
    }
}

/* ====================================================================
 * 3. BCM (BRIGHTNESS) INIT (RE-CALCULATED FOR 36MHz & 1-Bit)
 * ==================================================================== */
void initBCM(void) {
    // HCLK is now 36MHz
    // Target 60Hz refresh rate: 36,000,000 / 60 = 600,000 cycles per frame
    // Per anode (20 anodes): 600,000 / 20 = 30,000 cycles per anode scan
    // Since COLOR_DEPTH = 1, this is our only duration.
    // Our interrupt will fire every 30,000 cycles (or 0.83ms)
    bitDuration[0] = 30000;
}

/* ====================================================================
 * 4. BUTTON HARDWARE FUNCTIONS (From User - F103 Compatible)
 * ==================================================================== */
void init_button(void) { /* ... same as before ... */
    // Button is on PB12, which is not in the matrix_pins list
    uint8_t pin_num = 12;
    uint8_t shift = (pin_num - 8) * 4;
    GPIOB->CRH &= ~(0xF << shift);
    GPIOB->CRH |= (0x8 << shift);
    GPIOB->ODR |= (1 << pin_num);
}
uint8_t read_button(void) { /* ... same as before ... */
    if ((GPIOB->IDR & (1 << 12)) == 0) {
        return 1; // Pressed
    } else {
        return 0; // Not pressed
    }
}

/* ====================================================================
 * 5. FRAME COMPILER (THE LUT-BUILDER)
 * ==================================================================== */

/**
 * @brief Helper to set a pin to OUTPUT LOW state in a pre-calc struct
 */
void calc_output_low_state(const GpioPin* pin, GpioPortState* state)
{
    uint8_t pin_num = pin->pin_num;
    if (pin_num < 8) { // CRL
        uint8_t shift = pin_num * 4;
        state->crl &= ~(0xF << shift); // Clear old config
        state->crl |= (0x3 << shift);  // Set to 0b0011 (Output PP 50MHz)
    } else { // CRH
        uint8_t shift = (pin_num - 8) * 4;
        state->crh &= ~(0xF << shift); // Clear old config
        state->crh |= (0x3 << shift);  // Set to 0b0011 (Output PP 50MHz)
    }
    state->brr = (1 << pin_num); // Set pin LOW
}

/**
 * @brief Helper to set a pin to OUTPUT HIGH state in a pre-calc struct
 */
void calc_output_high_state(const GpioPin* pin, GpioPortState* state)
{
    uint8_t pin_num = pin->pin_num;
    if (pin_num < 8) { // CRL
        uint8_t shift = pin_num * 4;
        state->crl &= ~(0xF << shift); // Clear old config
        state->crl |= (0x3 << shift);  // Set to 0b0011 (Output PP 50MHz)
    } else { // CRH
        uint8_t shift = (pin_num - 8) * 4;
        state->crh &= ~(0xF << shift); // Clear old config
        state->crh |= (0x3 << shift);  // Set to 0b0011 (Output PP 50MHz)
    }
    state->bsrr = (1 << pin_num); // Set pin HIGH
}

/**
 * @brief Pre-calculates all GPIO register states into the given buffer.
 * This is the "LUT" builder that runs in the main() loop.
 * It now writes to the provided 'buffer' (the back buffer).
 */
void convertFrame(volatile GpioPortState* buffer)
{
    // Loop for the single (bit = 0) brightness bit-plane
    for (int bit = 0; bit < COLOR_DEPTH; bit++)
    {
        uint8_t bit_mask = (1 << bit); // This will just be '1'

        // Loop for each of the 20 anode rows
        for (int anode_idx = 0; anode_idx < NUM_PINS; anode_idx++)
        {
            // Calculate the offset into the 1D buffer array
            // (bit * 20 * 4) + (anode_idx * 4)
            uint32_t offset = (bit * NUM_PINS * 4) + (anode_idx * 4);

            // Get pointers to the GpioPortState structs for this scan step
            volatile GpioPortState* stateA = &buffer[offset + 0];
            volatile GpioPortState* stateB = &buffer[offset + 1];
            volatile GpioPortState* stateC = &buffer[offset + 2];
            volatile GpioPortState* stateD = &buffer[offset + 3];

            // 1. Initialize all ports to "all pins input"
            stateA->crl = GPIO_CRL_ALL_INPUT; stateA->crh = GPIO_CRH_ALL_INPUT;
            stateB->crl = GPIO_CRL_ALL_INPUT; stateB->crh = GPIO_CRH_ALL_INPUT;
            stateC->crl = GPIO_CRL_ALL_INPUT; stateC->crh = GPIO_CRH_ALL_INPUT;
            stateD->crl = GPIO_CRL_ALL_INPUT; stateD->crh = GPIO_CRH_ALL_INPUT;

            // 2. Clear all BSRR/BRR registers
            stateA->bsrr = 0; stateA->brr = 0;
            stateB->bsrr = 0; stateB->brr = 0;
            stateC->bsrr = 0; stateC->brr = 0;
            stateD->bsrr = 0; stateD->brr = 0;

            // 3. Loop through all 19 cathodes for this anode
            for (int cathode_idx = 0; cathode_idx < NUM_PINS; cathode_idx++)
            {
                if (anode_idx == cathode_idx) continue;

                // Find logical pixel
                int row = anode_idx;
                int col = (cathode_idx < anode_idx) ? cathode_idx : (cathode_idx - 1);

                // Check if this pixel's bit is on
                // Any non-zero value is "on"
                if (frame_buffer[row][col] != 0)
                {
                    const GpioPin* pin = &matrix_pins[cathode_idx];

                    // Find the correct port state to modify
                    if (pin->port == GPIOA)      calc_output_low_state(pin, (GpioPortState*)stateA);
                    else if (pin->port == GPIOB) calc_output_low_state(pin, (GpioPortState*)stateB);
                    else if (pin->port == GPIOC) calc_output_low_state(pin, (GpioPortState*)stateC);
                    else if (pin->port == GPIOD) calc_output_low_state(pin, (GpioPortState*)stateD);
                }
            }

            // 4. Finally, set the single anode pin to OUTPUT HIGH
            const GpioPin* pin = &matrix_pins[anode_idx];
            if (pin->port == GPIOA)      calc_output_high_state(pin, (GpioPortState*)stateA);
            else if (pin->port == GPIOB) calc_output_high_state(pin, (GpioPortState*)stateB);
            else if (pin->port == GPIOC) calc_output_high_state(pin, (GpioPortState*)stateC);
            else if (pin->port == GPIOD) calc_output_high_state(pin, (GpioPortState*)stateD);
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
  static uint32_t game_tick = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init(); // This initializes the HAL and the 1ms SysTick (which we override)
  SystemClock_Config(); // Now configures for 36MHz HCLK
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  // 1. Enable clocks for all our GPIO ports
  enable_gpio_clocks();

  // 2. Initialize the high-precision delay timer for game logic
  // DWT_Delay_Init(); // No longer needed

  // 3. Initialize the button
  init_button();

  // 4. Initialize the BCM brightness durations
  initBCM();

  // 5. Initialize the game state (bricks, ball, paddle)
  game_init();

  // 6. Set all pins to input (Hi-Z) to start
  set_all_pins_input();

  // 7. ---- START THE INTERRUPT DRIVER ----
  // This overrides the default HAL_Init() SysTick config
  SysTick->LOAD = bitDuration[0] - 1; // Load duration for bit 0
  SysTick->VAL = 0;                   // Reset counter
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | // Use HCLK (36MHz)
                  SysTick_CTRL_TICKINT_Msk   |
                  SysTick_CTRL_ENABLE_Msk;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 1. Update the game state (ball, paddle, etc.)
	update_game_state(game_tick++);

    // 2. "Compile" the frame_buffer into the BACK BUFFER
    convertFrame(g_BackBuffer);

    // 3. ---- ATOMIC SWAP ----
    //    Swap the active and back buffers.
    volatile GpioPortState* temp = g_ActiveFrame;
    g_ActiveFrame = g_BackBuffer;
    g_BackBuffer = temp;
    // The ISR will now read from the new frame we just built.

    // 4. Use the overflow-proof HAL_Delay.
    //    Our SysTick handler calls HAL_IncTick() every 0.83ms,
    //    so HAL_Delay(5) will wait ~4.15ms. Perfect.
    HAL_Delay(5);
    // DWT_Delay_us(5000); // No longer needed

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration (MODIFIED FOR 36MHz HCLK)
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
  // HSI (8MHz) / 2 = 4MHz.  4MHz * 9 = 36MHz SYSCLK
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  // HCLK = SYSCLK / 1 = 36MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // APB1 = 18MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; // APB2 = 36MHz

  // Flash latency must be 1 for 36MHz
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  /* This function is mostly overridden by our manual
   * register manipulation, but we still need clocks.
   */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_AFIO_REMAP_PD01_ENABLE();
}

/* USER CODE BEGIN 4 */

/* ====================================================================
 * SysTick HANDLER (THE *FAST* DISPLAY ENGINE)
 * ==================================================================== */

/**
 * @brief This function handles System tick timer interrupt.
 * (Remember to comment out the one in stm32f1xx_it.c)
 *
 * It now reads from the volatile g_ActiveFrame pointer,
 * which is guaranteed to be a complete, valid frame.
 */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */

    // 1. Calculate the offset into the *active* frame buffer
    //    g_current_bit will always be 0, so offset is just (g_current_anode * 4)
    uint32_t offset = (g_current_bit * NUM_PINS * 4) + (g_current_anode * 4);

    // 2. Get the pre-compiled register values from our "LUT"
    //    We use the g_ActiveFrame pointer.
    volatile GpioPortState* stateA = &g_ActiveFrame[offset + 0];
    volatile GpioPortState* stateB = &g_ActiveFrame[offset + 1];
    volatile GpioPortState* stateC = &g_ActiveFrame[offset + 2];
    volatile GpioPortState* stateD = &g_ActiveFrame[offset + 3];

    // 3. Write all 12 register values
    //    (The "port-blasting" operation)
    GPIOA->CRL  = stateA->crl;
    GPIOA->CRH  = stateA->crh;
    GPIOA->BSRR = stateA->bsrr;
    GPIOA->BRR  = stateA->brr;

    GPIOB->CRL  = stateB->crl;
    GPIOB->CRH  = stateB->crh;
    GPIOB->BSRR = stateB->bsrr;
    GPIOB->BRR  = stateB->brr;

    GPIOC->CRL  = stateC->crl;
    GPIOC->CRH  = stateC->crh;
    GPIOC->BSRR = stateC->bsrr;
    GPIOC->BRR  = stateC->brr;

    GPIOD->CRL  = stateD->crl;
    GPIOD->CRH  = stateD->crh;
    GPIOD->BSRR = stateD->bsrr;
    GPIOD->BRR  = stateD->brr;

    // --- 4. Advance the scanner state ---
    g_current_anode++;
    if (g_current_anode == NUM_PINS) {
        g_current_anode = 0; // Wrap around anodes (0-19)

        // This logic is trivial now but harmless to keep
        g_current_bit++;
        if (g_current_bit == COLOR_DEPTH) { // Will be true every time (1 == 1)
            g_current_bit = 0; // Wrap around bit-planes (0)
        }
        // Set the timer's duration (will always be bitDuration[0])
        SysTick->LOAD = bitDuration[g_current_bit] - 1;
    }

    //
    // !!!!! THIS WAS THE BUG. IT IS NOW REMOVED. !!!!!
    // SysTick->VAL = 0;
    //

    /* USER CODE END SysTick_IRQn 0 */

    // This is the fix for the DWT overflow.
    // We increment the HAL tick counter, which
    // makes HAL_Delay() work again.
    HAL_IncTick();

    /* USER CODE BEGIN SysTick_IRQn 1 */
    /* USER CODE END SysTick_IRQn 1 */
}

/* ====================================================================
 * GAME LOGIC FUNCTIONS (Upgraded for 1-bit brightness)
 * ==================================================================== */

/**
 * @brief Initializes/resets the game to the starting state.
 */
void game_init(void) {
    memset(bricks, 1, sizeof(bricks));
    paddle_x = (MATRIX_COLS / 2) - (PADDLE_WIDTH / 2);
    ball_y = PADDLE_Y - 1;
    ball_x = MATRIX_COLS / 2;
    ball_vx = 1;
    ball_vy = -1;
    game_over = 0;
    score = 0;
}

/**
 * @brief Updates the game state for one logic tick.
 */
void update_game_state(uint32_t tick) {
    // 1. Check for Game Over / Restart
    if (game_over)
    {
        if (read_button())
        {
            game_init();
        }
        draw_game_over_screen(tick);
        return;
    }

    // 2. Update Paddle Position
    if (read_button()) paddle_x++;
    else paddle_x--;

    // 3. Constrain paddle
    if (paddle_x < 0) paddle_x = 0;
    if (paddle_x > (MATRIX_COLS - PADDLE_WIDTH))
    {
        paddle_x = (MATRIX_COLS - PADDLE_WIDTH);
    }

    // 4. Update Ball Position
    ball_x += ball_vx;
    ball_y += ball_vy;

    // 5. Check Ball Collisions
    if (ball_x <= 0) { ball_x = 0; ball_vx = -ball_vx; }
    if (ball_x >= (MATRIX_COLS - 1)) { ball_x = MATRIX_COLS - 1; ball_vx = -ball_vx; }
    if (ball_y <= 0) { ball_y = 0; ball_vy = -ball_vy; }
    if (ball_y >= (MATRIX_ROWS - 1)) { game_over = 1; return; }

    // Paddle
    if (ball_y == (PADDLE_Y - 1) &&
        ball_x >= paddle_x &&
        ball_x < (paddle_x + PADDLE_WIDTH))
    {
        ball_vy = -ball_vy;
    }

    // Bricks
    if (ball_y < BRICK_ROWS)
    {
        if (bricks[ball_y][ball_x] == 1)
        {
            bricks[ball_y][ball_x] = 0;
            ball_vy = -ball_vy;
            score++;
        }
    }

    // 6. Draw the new game state to the frame_buffer
    draw_game_to_framebuffer();
}

/**
 * @brief Draws all game elements into the frame_buffer.
 * (Writes 1-bit brightness values, 0 or 1)
 */
void draw_game_to_framebuffer(void) {
    memset(frame_buffer, 0, sizeof(frame_buffer));
    for (int r = 0; r < BRICK_ROWS; r++) {
        for (int c = 0; c < MATRIX_COLS; c++) {
            if (bricks[r][c]) {
                frame_buffer[r][c] = 1; // On
            }
        }
    }
    for (int i = 0; i < PADDLE_WIDTH; i++) {
        if((paddle_x + i) < MATRIX_COLS) {
            frame_buffer[PADDLE_Y][paddle_x + i] = 1; // On
        }
    }
    frame_buffer[ball_y][ball_x] = 1; // On
}

/**
 * @brief Draws a flashing "G" for "Game Over" (re-centered)
 */
void draw_game_over_screen(uint32_t tick) {
    memset(frame_buffer, 0, sizeof(frame_buffer));
    if ((tick / 40) % 2) { // Flash every 40 ticks
        uint8_t bright = 1; // On
        int x = (MATRIX_COLS / 2) - 3; // Centered X
        int y = (MATRIX_ROWS / 2) - 3; // Centered Y

        frame_buffer[y+0][x+1] = bright; frame_buffer[y+0][x+2] = bright;
        frame_buffer[y+1][x+0] = bright;
        frame_buffer[y+2][x+0] = bright; frame_buffer[y+2][x+3] = bright;
        frame_buffer[y+3][x+0] = bright; frame_buffer[y+3][x+3] = bright;
        frame_buffer[y+4][x+1] = bright; frame_buffer[y+4][x+2] = bright;
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
  __disable_irq();
  while (1) { }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


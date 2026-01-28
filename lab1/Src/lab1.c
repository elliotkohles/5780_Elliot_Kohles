#include "main.h"
#include "stm32f0xx_hal.h"

void SystemClock_Config(void);

int main(void)
{
    /* Reset peripherals, init Flash + SysTick */
    HAL_Init();

    /* Configure system clock */
    SystemClock_Config();

    // Clock enable for GPIO ports
    // Enable GPIOC (LEDs) and GPIOA (USER button)
    RCC->AHBENR |= (1 << 19); // GPIOCEN
    RCC->AHBENR |= (1 << 17); // GPIOAEN

    // GPIO Configuration, output: PC8 (LD3), PC9 (LD4); input: PA0 (USER button)

    // ASSERT: GPIO clocks enabled
    if (!(RCC->AHBENR & (1 << 19))) Error_Handler();
    if (!(RCC->AHBENR & (1 << 17))) Error_Handler();

    // MODER: 01 = output
    GPIOC->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOC->MODER |=  ((1 << (8 * 2)) | (1 << (9 * 2)));

    // ASSERT: PC8 & PC9 are output mode (01)
    if (((GPIOC->MODER >> (8 * 2)) & 0x3) != 0x1) Error_Handler();
    if (((GPIOC->MODER >> (9 * 2)) & 0x3) != 0x1) Error_Handler();

    // OTYPER: push-pull
    GPIOC->OTYPER &= ~((1 << 8) | (1 << 9));

    // ASSERT: Push-pull mode
    if (GPIOC->OTYPER & ((1 << 8) | (1 << 9))) Error_Handler();

    // OSPEEDR: low speed
    GPIOC->OSPEEDR &= ~((3 << (8 * 2)) | (3 << (9 * 2)));

    // ASSERT: Low speed
    if (((GPIOC->OSPEEDR >> (8 * 2)) & 0x3) != 0x0) Error_Handler();
    if (((GPIOC->OSPEEDR >> (9 * 2)) & 0x3) != 0x0) Error_Handler();

    // PUPDR: no pull-up/down
    GPIOC->PUPDR &= ~((3 << (8 * 2)) | (3 << (9 * 2)));

    // ASSERT: No pull-up/down
    if (((GPIOC->PUPDR >> (8 * 2)) & 0x3) != 0x0) Error_Handler();
    if (((GPIOC->PUPDR >> (9 * 2)) & 0x3) != 0x0) Error_Handler();

    // Initial LED state: PC8 ON, PC9 OFF
    GPIOC->ODR |=  (1 << 8);
    GPIOC->ODR &= ~(1 << 9);

    // ASSERT: PC8 high, PC9 low
    if (!(GPIOC->ODR & (1 << 8))) Error_Handler();
    if ( GPIOC->ODR & (1 << 9))  Error_Handler();

    // MODER: 00 = input
    GPIOA->MODER &= ~(3 << (0 * 2));

    // ASSERT: PA0 input mode
    if (((GPIOA->MODER >> (0 * 2)) & 0x3) != 0x0) Error_Handler();

    // OSPEEDR: low speed
    GPIOA->OSPEEDR &= ~(3 << (0 * 2));

    // ASSERT: Low speed
    if (((GPIOA->OSPEEDR >> (0 * 2)) & 0x3) != 0x0) Error_Handler();

    // PUPDR: 10 = pull-down
    GPIOA->PUPDR &= ~(3 << (0 * 2));
    GPIOA->PUPDR |=  (2 << (0 * 2));

    // ASSERT: Pull-down enabled
    if (((GPIOA->PUPDR >> (0 * 2)) & 0x3) != 0x2) Error_Handler();

    while (1)
  {
    uint8_t currButtonState = (GPIOA->IDR & (1 << 0)) != 0;
    uint8_t static prevButtonState = 0;

    // Rising edge detection: button was released, now pressed
    if (currButtonState && !prevButtonState)
    {
        GPIOC->ODR ^= (1 << 8) | (1 << 9);
    }

    prevButtonState = currButtonState;
    HAL_Delay(20); // small debounce
  }

}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
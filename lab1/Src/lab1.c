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

    // MODER: 01 = output
    GPIOC->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOC->MODER |=  ((1 << (8 * 2)) | (1 << (9 * 2)));

    // OTYPER: push-pull
    GPIOC->OTYPER &= ~((1 << 8) | (1 << 9));

    // OSPEEDR: low speed
    GPIOC->OSPEEDR &= ~((3 << (8 * 2)) | (3 << (9 * 2)));

    // PUPDR: no pull-up/down
    GPIOC->PUPDR &= ~((3 << (8 * 2)) | (3 << (9 * 2)));

    // Initial LED state: PC8 ON, PC9 OFF
    GPIOC->ODR |=  (1 << 8);
    GPIOC->ODR &= ~(1 << 9);

    // MODER: 00 = input
    GPIOA->MODER &= ~(3 << (0 * 2));

    // OSPEEDR: low speed
    GPIOA->OSPEEDR &= ~(3 << (0 * 2));

    // PUPDR: 10 = pull-down
    GPIOA->PUPDR &= ~(3 << (0 * 2));
    GPIOA->PUPDR |=  (2 << (0 * 2));

    while (1)
    {
        // If USER button pressed (PA0 high)
        if (GPIOA->IDR & (1 << 0))
        {
            // Toggle LEDs
            GPIOC->ODR ^= (1 << 8) | (1 << 9);
            HAL_Delay(200); // simple debounce
        }
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
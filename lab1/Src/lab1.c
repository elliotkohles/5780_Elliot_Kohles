#include "main.h"
#include "stm32f0xx_hal.h"

#define ASSERT(cond) do { if (!(cond)) Error_Handler(); } while (0)

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // Enable GPIOC (LEDs) and GPIOA (USER button)
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // MODER: 01 = output
    // NOTE THE & ~ To Clear Bits First(this is a common way to initialize bits)
    // Ex. If I want to set bits 2 and 3 to 01, I first clear them with & ~(NOT) (11(mask) << 2(indexing))
    // 
    GPIOC->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2))); // Clear bits PC6 and PC7
    GPIOC->MODER |=  ((1 << (6 * 2)) | (1 << (7 * 2))); // Set bits to 01

    // OTYPER: push-pull
    GPIOC->OTYPER &= ~((1 << 6) | (1 << 7));

    // OSPEEDR: low speed
    GPIOC->OSPEEDR &= ~((3 << (6 * 2)) | (3 << (7 * 2)));

    // ASSERT: low speed
    ASSERT(((GPIOC->OSPEEDR >> (6 * 2)) & 0x3) == 0x0);
    ASSERT(((GPIOC->OSPEEDR >> (7 * 2)) & 0x3) == 0x0);

    // PUPDR: no pull-up/down
    GPIOC->PUPDR &= ~((3 << (6 * 2)) | (3 << (7 * 2)));

    // ASSERT: no pull resistors
    ASSERT(((GPIOC->PUPDR >> (6 * 2)) & 0x3) == 0x0);
    ASSERT(((GPIOC->PUPDR >> (7 * 2)) & 0x3) == 0x0);

    // Initial LED state: PC8 ON, PC9 OFF
    GPIOC->ODR |=  (1 << 6);
    GPIOC->ODR &= ~(1 << 7);

    // MODER: 00 = input
    GPIOA->MODER &= ~(3 << (0 * 2));

    // OSPEEDR: low speed
    GPIOA->OSPEEDR &= ~(3 << (0 * 2));

    // PUPDR: 10 = pull-down
    GPIOA->PUPDR &= ~(3 << (0 * 2));
    GPIOA->PUPDR |=  (2 << (0 * 2));

    // ASSERT: pull-down enabled
    ASSERT(((GPIOA->PUPDR >> (0 * 2)) & 0x3) == 0x2);

    //static uint8_t prevButtonState = 0;

    while (1)
    {
        //uint8_t currButtonState = (GPIOA->IDR & (1 << 0)) != 0;

        // // Rising-edge detection
        // if (currButtonState && !prevButtonState)
        // {
          GPIOC->ODR ^= (1 << 6) | (1 << 7);
        // }

        // prevButtonState = currButtonState;
        HAL_Delay(100); // debounce
    }

    
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        // Trap CPU here on assertion failure
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure the HSI oscillator */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Select HSI as system clock source and configure the bus clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_HCLK   |
                                  RCC_CLOCKTYPE_PCLK1;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}



#include "stm32f1xx_hal.h"
#include "Variable.h"

#define LED_LDRIFT(x)   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,(GPIO_PinState)(x))
#define LED_RDRIFT(x)   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,(GPIO_PinState)(x))
#define LED_UNLOCK(x)   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,(GPIO_PinState)(x))
#define LED_LOCK(x)     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,(GPIO_PinState)(x))

uint8_t LLbit = 0;
uint8_t LRbit = 0;

// void LED_Init(void)      重复初始化了
// {
//     __HAL_RCC_GPIOB_CLK_ENABLE();
//     __HAL_RCC_GPIOA_CLK_ENABLE();
//     GPIO_InitTypeDef GPIO_InitStruct;
//     GPIO_InitStruct.Pin = GPIO_PIN_7  | GPIO_PIN_1 | GPIO_PIN_10;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);

//     GPIO_InitStruct.Pin = GPIO_PIN_5;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL;
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//     HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);

// }

void LED_Control(void)
{
    if(Key_LED == 7)
    {
        LED_LOCK(1);
        LED_UNLOCK(0);
    }
    if(Key_LED == 8)
    {
        LED_UNLOCK(1);
        LED_LOCK(0);
    }
    if((Encoder_number / 4 ) > 2)
    {
        LED_LDRIFT(1);
        LED_RDRIFT(0);
    }
    if((Encoder_number / 4 ) < -2)
    {
        LED_LDRIFT(0);
        LED_RDRIFT(1);
    }
    if((Encoder_number / 4 ) <= 2 && (Encoder_number / 4 ) >= -2)
    {
        LED_LDRIFT(1);
        LED_RDRIFT(1);
    }
}


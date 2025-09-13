#include "segment_display.h"

uint8_t segmentCodes[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

GPIO_TypeDef* segmentPorts[] = {GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB};
uint16_t segmentPins[] = {GPIO_PIN_8, GPIO_PIN_4, GPIO_PIN_10, GPIO_PIN_1, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14};

GPIO_TypeDef* digitPorts[] = {GPIOB, GPIOB};
uint16_t digitPins[] = {GPIO_PIN_5, GPIO_PIN_6};

static uint8_t displayValue = 0;

void segment_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    for (int i = 0; i < 7; i++) {
        GPIO_InitStruct.Pin = segmentPins[i];
        HAL_GPIO_Init(segmentPorts[i], &GPIO_InitStruct);
    }

    for (int i = 0; i < 2; i++) {
        GPIO_InitStruct.Pin = digitPins[i];
        HAL_GPIO_Init(digitPorts[i], &GPIO_InitStruct);
    }
}

void setSegment(uint8_t number) {
    uint8_t code = segmentCodes[number % 10];
    for (int i = 0; i < 7; i++) {
        HAL_GPIO_WritePin(segmentPorts[i], segmentPins[i], (code & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void setDigit(uint8_t digit) {
    for (int i = 0; i < 2; i++) {
        HAL_GPIO_WritePin(digitPorts[i], digitPins[i], (i == digit) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}

void update7Segment(uint8_t number) {
    displayValue = number;
}

void segment_refresh_loop(uint8_t value) {
    static uint8_t currentDigit = 0;
    setDigit(currentDigit);
    if (currentDigit == 0) setSegment(value / 10);
    else setSegment(value % 10);
    currentDigit = 1 - currentDigit;
}

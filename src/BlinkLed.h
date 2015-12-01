//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

#ifndef BLINKLED_H_
#define BLINKLED_H_

#include "stm32f0xx.h"

// ----- LED definitions ------------------------------------------------------

#define AUX_LED_BLUE_ON GPIO_ResetBits(GPIOB, GPIO_Pin_8)
#define AUX_LED_RED_ON GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define AUX_LED_GREEN_ON GPIO_ResetBits(GPIOB, GPIO_Pin_11)
#define AUX_LED_BLUE_OFF GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define AUX_LED_RED_OFF GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define AUX_LED_GREEN_OFF GPIO_SetBits(GPIOB, GPIO_Pin_11)

// Adjust these definitions for your own board.

// STM32F0DISCOVERY definitions (GREEN led, C9, active high)
// (SEGGER J-Link device name: STM32F051R8).

// Port numbers: 0=A, 1=B, 2=C, 3=D, 4=E, 5=F, 6=G, ...
#define BLINK_PORT_NUMBER               (1)
#define BLINK_PIN_NUMBER                (11)
#define BLINK_ACTIVE_LOW                (0)

#define BLINK_GPIOx(_N)                 ((GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(_N)))
#define BLINK_PIN_MASK(_N)              (1 << (_N))
#define BLINK_RCC_MASKx(_N)             (RCC_AHBPeriph_GPIOA << (_N))
// ----------------------------------------------------------------------------

extern
void
blink_led_init(void);

// ----------------------------------------------------------------------------

inline void
blink_led_on(void);

inline void
blink_led_off(void);

// ----------------------------------------------------------------------------

inline void
__attribute__((always_inline))
blink_led_on(void) {
#if (BLINK_ACTIVE_LOW)
	GPIO_ResetBits(BLINK_GPIOx(BLINK_PORT_NUMBER),
			BLINK_PIN_MASK(BLINK_PIN_NUMBER));
#else
	GPIO_SetBits(BLINK_GPIOx(BLINK_PORT_NUMBER),
			BLINK_PIN_MASK(BLINK_PIN_NUMBER));
#endif
}

inline void
__attribute__((always_inline))
blink_led_off(void) {
#if (BLINK_ACTIVE_LOW)
	GPIO_SetBits(BLINK_GPIOx(BLINK_PORT_NUMBER),
			BLINK_PIN_MASK(BLINK_PIN_NUMBER));
#else
	GPIO_ResetBits(BLINK_GPIOx(BLINK_PORT_NUMBER),
			BLINK_PIN_MASK(BLINK_PIN_NUMBER));
#endif
}

// ----------------------------------------------------------------------------

#endif // BLINKLED_H_

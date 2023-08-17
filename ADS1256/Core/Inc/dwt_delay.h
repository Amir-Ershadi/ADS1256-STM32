/*
 * Simple microseconds delay routine, utilizing ARM's DWT
 * (Data Watchpoint and Trace Unit) and HAL library.
 * Intended to use with gcc compiler, but I hope it can be used
 * with any other C compiler across the Universe (provided that
 * ARM and CMSIS already invented) :)
 * Max K
 *
 *
 * This file is part of DWT_Delay package.
 * DWT_Delay is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * us_delay is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 * http://www.gnu.org/licenses/.
 */
#define STM32F4 

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_gpio.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_gpio.h"
#elif defined(STM32L4)
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_spi.h"
#include "stm32l4xx_hal_gpio.h"
#elif defined(STM32F3)
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_spi.h"
#include "stm32f3xx_hal_gpio.h"
#elif defined(STM32F7)
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_spi.h"
#include "stm32f7xx_hal_gpio.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_gpio.h"
#endif

#include <stdint.h>

#ifndef INC_DWT_DELAY_H_
#define INC_DWT_DELAY_H_

#define DWT_DELAY_NEWBIE 0

void DWT_Init(void);
void delayus(uint32_t us);

#endif /* INC_DWT_DELAY_DWT_DELAY_H_ */

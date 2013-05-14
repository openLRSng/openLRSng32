#pragma once

// for roundf()
#define __USE_C99_MATH

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "stm32f10x_conf.h"
#include "core_cm3.h"
#include "printf.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

#define RADX10 (M_PI / 1800.0f)                  // 0.001745329252f

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// Chip Unique ID on F103
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

#define digitalHi(p, i)     { p->BSRR = i; }
#define digitalLo(p, i)     { p->BRR = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }

#define LEDR_GPIO GPIOA
#define LEDR_PIN  GPIO_Pin_11
#define LEDR_ON   digitalHi(LEDR_GPIO,LEDR_PIN)
#define LEDR_OFF  digitalLo(LEDR_GPIO,LEDR_PIN)
#define LEDG_GPIO GPIOA
#define LEDG_PIN  GPIO_Pin_12
#define LEDG_ON   digitalHi(LEDG_GPIO,LEDG_PIN)
#define LEDG_OFF  digitalLo(LEDG_GPIO,LEDG_PIN)
#define LEDB_GPIO GPIOA
#define LEDB_PIN  GPIO_Pin_13
#define LEDB_ON   digitalHi(LEDB_GPIO,LEDB_PIN)
#define LEDB_OFF  digitalLo(LEDB_GPIO,LEDB_PIN)

#include "drv_system.h"         // timers, delays, etc
#include "drv_uart.h"
#include "drv_pwm.h"
#include "drv_spi.h"
#include "drv_i2c.h"
#include "drv_mpu6050.h"

#pragma once
#include <stdint.h>
double trimf(double x, double a, double b, double c);
double gaussmf(double x, double ave, double sigma);
double trapmf(double x, double a, double b, double c, double d);
#define TRIMF 0x1000
#define GAUSSMF 0x2000
#define TRAPMF 0x3000
#define NO_TYPE 0x0000


/*!< STM8Lx Standard Peripheral Library old types (maintained for legacy purpose) */

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef u32 TYPE;

#ifndef __HELPER_H__
#define __HELPER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "proj.h"

#define PREC 5

#define PI           3.14159265358f
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f

/*
#ifdef CONFIG_DEBUG_PRINTF
void _printf(const char *p_format, ...);
#define _dbg(...) _printf(__VA_ARGS__)
#else
#define _dbg(...) ((void)0)
#endif
*/

float _atan2f(float y, float x);
float _sin(const float x);
float _cos(const float x);
float _sqrt(const float number);
float radians(const float x);
float sq(const float x);

uint8_t str_to_uint32(char *str, uint32_t * out, const uint8_t seek,
                      const uint8_t len, const uint32_t min, const uint32_t max);
uint8_t str_to_uint16(char *str, uint16_t * out, const uint8_t seek,
                      const uint8_t len, const uint16_t min, const uint16_t max);
uint8_t str_to_floaty_uint16(char *str, uint16_t * out, const uint8_t seek,
                      const uint8_t len, const uint8_t precision, const uint16_t min, const uint16_t max);

#ifdef __cplusplus
}
#endif

#endif
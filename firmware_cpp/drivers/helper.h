#ifndef __HELPER_H__
#define __HELPER_H__

#ifdef __cplusplus
extern "C" {
#endif

#define PREC 5

#define PI           3.14159265358f
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f

float _atan2f(float y, float x);
float _sin(const float x);
float _cos(const float x);
float _sqrt(const float number);
float radians(const float x);
float sq(const float x);

/** return a binary string for an uint16_t integer
    @param temporary buffer used to build up the string. buf[17] needs to be allocated
    @param uint16_t value
    @return pointer to the string
*/
char *_utob(char *buf, const uint16_t val);

/** return a hex string for an uint32_t integer
    @param temporary buffer used to build up the string. buf[11] needs to be allocated
    @param uint32_t value
    @return pointer to the string
*/
char *_utoh(char *buf, const uint32_t val);

/** return a decimal string for an uint32_t integer
    @param temporary buffer used to build up the string. buf[11] needs to be allocated
    @param uint32_t value
    @return pointer to the string
*/
char *_utoa(char *buf, const uint32_t val);

/** return a decimal string for an int32_t integer
    @param temporary buffer used to build up the string. buf[11] needs to be allocated
    @param int32_t value
    @return pointer to the string
*/
char *_itoa(char *buf, const int32_t val);


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

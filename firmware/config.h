#ifndef _CONFIG_H_
#define _CONFIG_H_

//#define CONFIG_DEBUG

#define LOG_LEVEL_NONE      0x0 //  9205
#define LOG_LEVEL_FATAL     0x1 //  9241
#define LOG_LEVEL_ERROR     0x2 //  9337
#define LOG_LEVEL_WARNING   0x3 //  9449
#define LOG_LEVEL_INFO      0x4 //  9756
#define LOG_LEVEL_DEBUG     0x5 // 10002
#define LOG_LEVEL_TRACE     0x6 // 10574

#define CONFIG_LOG_LEVEL    LOG_LEVEL_WARNING

// dont touch
//#define USE_WATCHDOG

#define UART0_SPEED_115200_8M

//#define USE_XT1
#define USE_XT2

#endif

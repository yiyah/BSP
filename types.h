#ifndef __TYPES_H
#define __TYPES_H
#include <limits.h>
/**
 * @brief typedef data type
 * 
 * @note  work with 32bit platform
 */
typedef unsigned char           u8;
typedef unsigned short int      u16;
typedef unsigned int            u32;

typedef signed   char           s8;
typedef signed   short          s16;
typedef signed   int            s32;

typedef float                   f32;

#define NULL                    ((void *)0)
#define TRUE                    1U
#define FALSE                   0U

#endif

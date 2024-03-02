#ifndef __UTIL_H
#define __UTIL_H

/**
// #define offsetof(TYPE, MEMBER) ((size_t) &((TYPE*)0)->MEMBER)
// #define container_of(ptr, type, member) ({          \
//         const typeof( ((type *)0)->member ) *__mptr = (const typeof( ((type *)0)->member ) *)(ptr); \
//         (type *)( (char *)__mptr - offsetof(type,member) );})

 * 
 */

#define container_of(ptr, type, member) ({                  \
        void *__mptr = (void *)(ptr);                       \
        ((type *)(__mptr - offsetof(type, member))); })
#endif

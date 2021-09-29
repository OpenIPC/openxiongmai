#ifndef RE_TYPE_H
#define RE_TYPE_H

typedef unsigned char XM_U8;
typedef unsigned short XM_U16;
typedef unsigned int XM_U32;

typedef signed char XM_S8;
typedef short XM_S16;
typedef int XM_S32;

typedef unsigned long long XM_U64;
typedef long long XM_S64;

typedef char XM_CHAR;
#define XM_VOID void

typedef enum {
  XM_FALSE = 0,
  XM_TRUE = 1,
} XM_BOOL;

#ifndef NULL
#define NULL 0L
#endif

#define XM_NULL 0L
#define XM_SUCCESS 0
#define XM_FAILURE (-1)

#endif /* RE_TYPE_H */

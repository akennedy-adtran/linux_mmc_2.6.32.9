/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/*

 vitesse_common.h  -- Vitesse common definitions

 This file is used by the Vitesse Switch/Mac API software.
 Modify it to fit your configuration.

 Copyright (c) 2003 Vitesse Semiconductor Corporation. All Rights Reserved.
 Unpublished rights reserved under the copyright laws of the United States of 
 America, other countries and international treaties. The software is provided
 without fee. Permission to use, copy, store, modify, disclose, transmit or 
 distribute the software is granted, provided that this copyright notice must 
 appear in any copy, modification, disclosure, transmission or distribution of 
 the software. Vitesse Semiconductor Corporation retains all ownership, 
 copyright, trade secret and proprietary rights in the software. THIS SOFTWARE
 HAS BEEN PROVIDED "AS IS," WITHOUT EXPRESS OR IMPLIED WARRANTY INCLUDING, 
 WITHOUT LIMITATION, IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
 PARTICULAR USE AND NON-INFRINGEMENT.
 
*/
#include <linux/types.h>

#define VSC7323

#ifndef _VITESSE_COMMON_H
#define _VITESSE_COMMON_H 1

/* ================================================================= *
 *  Basic types:
 *    uint, ulong, ushort, uchar - unsigned standard types.
 *    longlong, ulonglong - 64 bit integers.
 *    BOOL - The boolean type. false: 0, true: anything but 0.
 * ================================================================= */

//typedef unsigned long       ulong;
typedef unsigned char       uchar;

/* - longlong, ulonglong ------------------------------------------- */

/* longlong and ulonglong: 64 bit integers */
#ifdef __GNUC__
typedef long long           longlong;
typedef unsigned long long  ulonglong;
#endif /* __GNUC__ */
#ifdef _MSC_VER
typedef __int64             longlong;
typedef unsigned __int64    ulonglong;
#endif /* _MSC_VER */

/* - BOOL ---------------------------------------------------------- */

/* BOOL: The boolean type. false: 0, true: anything but 0. */
/* You may redefine it to any other type, e.g. char. */
typedef int                 BOOL;


/* ================================================================= *
 *  Custom types
 * ================================================================= */

/* Big counter type, may be 32 or 64 bits depending on the OS */
/* You may redefine it to ulong or ulonglong */
typedef ulonglong vtss_counter_t;


/* ================================================================= *
 *  Macros:
 *    VTSS_ASSERT(expr) - Call assert(expr).
 *    VTSS_NSLEEP(nsec) - Sleep at least nsec nanoseconds.
 *    VTSS_MSLEEP(msec) - Sleep at least msec milliseconds.
 *  Notes:
 *    VTSS_NSLEEP uses busy waiting, so it should only be used for
 *    very short intervals.
 *    VTSS_MSLEEP should not use busy waiting, but may do so.
 * ================================================================= */


/* - VTSS_ASSERT(expr) -------------------------------------------------- */

/* VTSS_ASSERT(expr): Call assert(). */
#if !defined(VTSS_ASSERT)
/* You may define your own VTSS_ASSERT here. */
#endif

#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 0
#endif

#if !defined(VTSS_ASSERT) && (_POSIX_C_SOURCE > 0)
#include <assert.h>
#define VTSS_ASSERT(expr) { assert(expr); }
#endif

/* - VTSS_NSLEEP(nsec) -------------------------------------------------- */
#define VTSS_NSLEEP(nsec)

/* VTSS_NSLEEP(nsec): Sleep nsec nanoseconds. Use busy waiting. */
#if !defined(VTSS_NSLEEP)
/* You may define your own VTSS_NSLEEP here. */
#endif

#if !defined(VTSS_NSLEEP) && defined(_BSD_SOURCE)
/* The function "gettimeofday" is available, so use it. */
#include <sys/time.h>
#define VTSS_NSLEEP(nsec) { \
    struct timeval tve, tv; \
    gettimeofday(&tve,NULL); \
    tve.tv_usec+=(nsec+999)/1000;\
    if (tve.tv_usec>=1000000) { tve.tv_sec+=tve.tv_usec/1000000; tve.tv_usec%=1000000; } \
    do { gettimeofday(&tv,NULL); } \
    while ( timercmp(&tv,&tve,<) ); \
}
#endif /* _BSD_SOURCE */
#if !defined(VTSS_NSLEEP) && ( (_POSIX_C_SOURCE - 0) >= 199309L )
/* The function "nanosleep" is available, so use it. */
#include <time.h>
#include <errno.h>
#define VTSS_NSLEEP(nsec) {struct timespec ts = { 0, nsec }; while (nanosleep(&ts,&ts)==-1 && errno==EINTR) ;}
#endif /* (_POSIX_C_SOURCE - 0) >= 199309L */
#if !defined(VTSS_NSLEEP) && defined(_WIN32)
#define VTSS_NSLEEP(nsec) {VOID Sleep(DWORD); Sleep((nsec+999999)/1000000);}
#endif /* _WIN32 */
/* VTSS_NSLEEP is required by the API. */
#if defined(__VTSS_LIBRARY__) && !defined(VTSS_NSLEEP)
#error Macro function VTSS_NSLEEP must be defined in vitesse_common.h.
#endif /* __VTSS_LIBRARY__ && !VTSS_NSLEEP */

/* - VTSS_MSLEEP(msec) -------------------------------------------------- */
#define VTSS_MSLEEP(msec)

/* VTSS_MSLEEP(msec): Sleep msec milliseconds. Avoid busy waiting. */
#if !defined(VTSS_MSLEEP)
/* You may define your own VTSS_MSLEEP here. */
#endif

#if !defined(VTSS_MSLEEP) && ( (_POSIX_C_SOURCE - 0) >= 199309L )
/* The function "nanosleep" is available, so use it. */
#include <time.h>
#include <errno.h>
#define VTSS_MSLEEP(msec) { \
    struct timespec ts; \
    ts.tv_sec = msec/1000; \
    ts.tv_nsec = (msec%1000)*1000000; \
    while (nanosleep(&ts,&ts)==-1 && errno==EINTR) ; \
}
#endif /* (_POSIX_C_SOURCE - 0) >= 199309L */
#if !defined(VTSS_MSLEEP) && defined(_WIN32)
#define VTSS_MSLEEP(msec) {VOID Sleep(DWORD); Sleep(msec);}
#endif /* _WIN32 */
/* VTSS_MSLEEP is required by the API. */
#if defined(__VTSS_LIBRARY__) && !defined(VTSS_MSLEEP)
#error Macro function VTSS_MSLEEP must be defined in vitesse_common.h.
#endif /* __VTSS_LIBRARY__ && !VTSS_MSLEEP */


/* ================================================================= *
 *  Debugging Macros:
 *    VTSS_E(args) - Error.
 *    VTSS_D(args) - Debug.
 *    VTSS_N(args) - Noise.
 *
 *    Usage Example: VTSS_E(("Port %d",port_no));
 * ================================================================= */

#if !defined(VTSS_E) && !defined(VTSS_D) && !defined(VTSS_N)
/* You may define your own VTSS_E, VTSS_D and VTSS_N here. */
#endif

#if !defined(VTSS_E) && !defined(VTSS_D) && !defined(VTSS_N) && defined(SWITCH_APP_TRACE)
/* Debugging for an VTSS internal application. */
#include <stdio.h>
#include <stdarg.h>

extern const char *vtss_trace_func;
extern int vtss_trace_line;
extern int vtss_trace_layer;
extern int vtss_trace_level;
void vtss_api_trace(int level, BOOL debug, const char *func, int line, char *msg);
void vtss_trace(const char *fmt, ...);

#define E(args); {vtss_trace_level=1, vtss_trace_layer=VTSS_TRACE_LAYER, vtss_trace_func=__FUNCTION__; vtss_trace_line=__LINE__; vtss_trace args;}
#define D(args); {vtss_trace_level=2, vtss_trace_layer=VTSS_TRACE_LAYER, vtss_trace_func=__FUNCTION__; vtss_trace_line=__LINE__; vtss_trace args;}
#define N(args); {vtss_trace_level=3, vtss_trace_layer=VTSS_TRACE_LAYER, vtss_trace_func=__FUNCTION__; vtss_trace_line=__LINE__; vtss_trace args;}
#endif

#if !defined(VTSS_E) && !defined(VTSS_D) && !defined(VTSS_N)
#if (_POSIX_C_SOURCE > 0)
/* Fallback to debugging using printf. */
#include <stdio.h>
#define VTSS_E(args) { printf("E:%s#%d: ",__FUNCTION__,__LINE__); printf args; printf("\n"); }
#define VTSS_D(args) { printf("D:%s#%d: ",__FUNCTION__,__LINE__); printf args; printf("\n"); }
#define VTSS_N(args) { printf("N:%s#%d: ",__FUNCTION__,__LINE__); printf args; printf("\n"); }
#else
/* Fallback to no debugging. */
#define VTSS_E(args)
#define VTSS_D(args)
#define VTSS_N(args)
#endif
#endif


/* ================================================================= *
 *  No changes should be needed below this line
 * ================================================================= */

#if !defined(MEIGS2) && !defined(MEIGS2E) && !defined(CAMPBELL) && \
    !defined(VSC7321) && !defined(VSC7323) && !defined(VSC7331) 
#warning Device MEIGS2, MEIGS2E, CAMPBELL, VSC7321, VSC7323 or VSC7331 should be defined.
#endif

/* ================================================================= *
 *  Various definitions and macros
 * ================================================================= */

/* MAKEBOOL01(value): Convert BOOL value to 0 (false) or 1 (true). */
/* Use this to ensure BOOL values returned are always 1 or 0. */
#ifndef MAKEBOOL01
#define MAKEBOOL01(value) ((value)?1:0)
#endif
#ifndef TRUE
#define TRUE MAKEBOOL01(1)
#endif
#ifndef FALSE
#define FALSE MAKEBOOL01(0)
#endif

/* - Basic defines/macros: NULL, offsetof() ------------------------ */

#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *)0)
#endif
#endif

#ifndef offsetof
#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#endif

/* - Compiler Hints ------------------------------------------------ */

#ifdef __GNUC__
/* "__attribute__ ((const))" informs the GNU C compiler that a
 * function does not change any states anywhere
 */
#define __VTSS_ATTRIB_CONST_FUNCTION__ __attribute__ ((const))
#else
#define __VTSS_ATTRIB_CONST_FUNCTION__ /* no "const" compiler hint */
#endif


/* ================================================================= *
 *  Private data and functions (used internally by VTSS library)
 * ================================================================= */
#ifdef __VTSS_LIBRARY__

/* - Compiler Hints ------------------------------------------------ */

#ifdef __GNUC__
/* "__attribute__ ((unused))" informs the GNU C compiler that a
 * variable is not used, so we don't get a warning about it
 */
#define __VTSS_ATTRIB_UNUSED_VARIABLE__ __attribute__ ((unused))
#else
#define __VTSS_ATTRIB_UNUSED_VARIABLE__ /* no "unused" compiler hint */
#endif

#endif /* __VTSS_LIBRARY__ */

#endif /* _VITESSE_COMMON_H_ */

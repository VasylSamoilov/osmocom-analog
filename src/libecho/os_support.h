/* Copyright (C) 2007 Jean-Marc Valin

   File: os_support.h
   OS abstraction layer for SpeexDSP

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are
   met:

   1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef OS_SUPPORT_H
#define OS_SUPPORT_H

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/** Speex wrapper for calloc - MUST clear memory */
static inline void *speex_alloc(int size)
{
   return calloc(size, 1);
}

/** Speex wrapper for realloc */
static inline void *speex_realloc(void *ptr, int size)
{
   return realloc(ptr, size);
}

/** Speex wrapper for free */
static inline void speex_free(void *ptr)
{
   free(ptr);
}

/** Copy n elements from src to dst */
#define SPEEX_COPY(dst, src, n) (memcpy((dst), (src), (n)*sizeof(*(dst)) + 0*((dst)-(src))))

/** Copy n elements from src to dst, allowing overlapping regions */
#define SPEEX_MOVE(dst, src, n) (memmove((dst), (src), (n)*sizeof(*(dst)) + 0*((dst)-(src))))

/** Set memory */
#define SPEEX_MEMSET(dst, c, n) (memset((dst), (c), (n)*sizeof(*(dst))))

/** Fatal error handler */
static inline void _speex_fatal(const char *str, const char *file, int line)
{
   fprintf(stderr, "Fatal (internal) error in %s, line %d: %s\n", file, line, str);
   exit(1);
}

/** Warning handler */
static inline void speex_warning(const char *str)
{
#ifndef DISABLE_WARNINGS
   fprintf(stderr, "warning: %s\n", str);
#else
   (void)str;
#endif
}

/** Warning handler with integer value */
static inline void speex_warning_int(const char *str, int val)
{
#ifndef DISABLE_WARNINGS
   fprintf(stderr, "warning: %s %d\n", str, val);
#else
   (void)str;
   (void)val;
#endif
}

/** Notification handler */
static inline void speex_notify(const char *str)
{
#ifndef DISABLE_NOTIFICATIONS
   fprintf(stderr, "notification: %s\n", str);
#else
   (void)str;
#endif
}

#define speex_fatal(str) _speex_fatal(str, __FILE__, __LINE__);
#define speex_assert(cond) {if (!(cond)) {speex_fatal("assertion failed: " #cond);}}

#endif /* OS_SUPPORT_H */


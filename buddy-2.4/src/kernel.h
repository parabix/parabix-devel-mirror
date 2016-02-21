/*========================================================================
               Copyright (C) 1996-2002 by Jorn Lind-Nielsen
                            All rights reserved

    Permission is hereby granted, without written agreement and without
    license or royalty fees, to use, reproduce, prepare derivative
    works, distribute, and display this software and its documentation
    for any purpose, provided that (1) the above copyright notice and
    the following two paragraphs appear in all copies of the source code
    and (2) redistributions, including without limitation binaries,
    reproduce these notices in the supporting documentation. Substantial
    modifications to this software may be copyrighted by their authors
    and need not follow the licensing terms described here, provided
    that the new terms are clearly indicated in all files where they apply.

    IN NO EVENT SHALL JORN LIND-NIELSEN, OR DISTRIBUTORS OF THIS
    SOFTWARE BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL,
    INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS
    SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE
    ABOVE PARTIES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    JORN LIND-NIELSEN SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING,
    BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
    FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS
    ON AN "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO
    OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR
    MODIFICATIONS.
========================================================================*/

/*************************************************************************
  $Header: /cvsroot/buddy/buddy/src/kernel.h,v 1.2 2004/07/13 20:51:49 haimcohen Exp $
  FILE:  kernel.h
  DESCR: Kernel specific definitions for BDD package
  AUTH:  Jorn Lind
  DATE:  (C) june 1997
*************************************************************************/

#ifndef _KERNEL_H
#define _KERNEL_H

/*=== Includes =========================================================*/

#include <limits.h>
#include <stdexcept>
#include "bdd.h"

/*=== BUILT-INS ==========================================================*/

#ifndef __has_builtin
#define __has_builtin(x) 0
#endif

#ifndef __GNUC_PREREQ
#if defined(__GNUC__) && defined(__GNUC_MINOR__)
  #define __GNUC_PREREQ(maj, min) ((__GNUC__ << 16) + __GNUC_MINOR__ >= ((maj) << 16) + (min))
#else
  #define __GNUC_PREREQ(maj, min) 0
#endif
#endif

#if __has_builtin(__builtin_expect) || __GNUC_PREREQ(4, 0)
#define LIKELY(EXPR) __builtin_expect((bool)(EXPR), true)
#define UNLIKELY(EXPR) __builtin_expect((bool)(EXPR), false)
#else
#define LIKELY(EXPR) (EXPR)
#define UNLIKELY(EXPR) (EXPR)
#endif

#ifndef NDEBUG
#define ALWAYS_INLINE
#elif defined _MSC_VER
#define ALWAYS_INLINE   __forceinline
#elif defined __GNUC__
#define ALWAYS_INLINE   __attribute__((always_inline)) inline
#else
#define ALWAYS_INLINE   inline
#endif

/*=== SANITY CHECKS ====================================================*/

/* Make sure we use at least 32 bit integers */
#if (INT_MAX < 0x7FFFFFFF)
#error The compiler does not support 4 byte integers!
#endif

/*=== SEMI-INTERNAL TYPES ==============================================*/

typedef struct s_BddNode /* Node table entry */
{
    unsigned int refcount : 10;
    unsigned int level : 22;
    int low;
    int high;
    unsigned hash;
    unsigned next;
} BddNode;


/*=== KERNEL VARIABLES =================================================*/

#ifdef CPLUSPLUS
extern "C" {
#endif

extern int       bddrunning;         /* Flag - package initialized */
extern int       bddnodesize;        /* Number of allocated nodes */
extern int       bddmaxnodesize;     /* Maximum allowed number of nodes */
extern int       bddmaxnodeincrease; /* Max. # of nodes used to inc. table */
extern BddNode*  bddnodes;           /* All of the bdd nodes */
extern int       bddvarnum;          /* Number of defined BDD variables */
extern int*      bddrefstack;        /* Internal node reference stack */
extern int*      bddrefstacktop;     /* Internal node reference stack top */
extern int*      bddvar2level;
extern int*      bddlevel2var;
extern int       bddreorderdisabled;
extern int       bddresized;

#ifdef CPLUSPLUS
}
#endif

/*=== KERNEL DEFINITIONS ===============================================*/

class reordering_required : public std::exception {
public:
reordering_required() = default;
virtual char const * what() const  _GLIBCXX_USE_NOEXCEPT { return nullptr; }
};

/*=== KERNEL DEFINITIONS ===============================================*/

#define MAXVAR 0x1FFFFF
#define MAXREF 0x3FF

static ALWAYS_INLINE void DECREF(const BDD i) {
    BddNode & n = bddnodes[i];
    if (LIKELY(n.refcount != 0 && n.refcount != MAXREF)) {
        n.refcount--;
    }
}

static ALWAYS_INLINE void INCREF(const BDD i) {
    BddNode & n = bddnodes[i];
    if (LIKELY(n.refcount != MAXREF)) {
        n.refcount++;
    }
}

/* Reference counting */
static ALWAYS_INLINE bool HASREF(const BDD i) {
    return (bddnodes[i].refcount != 0);
}

/* Marking BDD nodes */
#define MARKON   0x200000    /* Bit used to mark a node (1) */
#define MARKOFF  0x1FFFFF    /* - unmark */
#define MARKHIDE 0x1FFFFF

static ALWAYS_INLINE void SETMARK(BddNode * const p) {
    p->level |= MARKON;
}

static ALWAYS_INLINE void SETMARK(const BDD i) {
    SETMARK(bddnodes + i);
}

static ALWAYS_INLINE void UNMARK(BddNode * const p) {
    p->level &= MARKOFF;
}

static ALWAYS_INLINE void UNMARK(const BDD i) {
    UNMARK(bddnodes + i);
}

static ALWAYS_INLINE bool MARKED(const BddNode * const p) {
    return (p->level & MARKON) != 0;
}

static ALWAYS_INLINE bool MARKED(const BDD i) {
    return MARKED(bddnodes + i);
}




/* Hashfunctions */

#define PAIR(a,b)      ((unsigned int)((((unsigned int)a)+((unsigned int)b))*(((unsigned int)a)+((unsigned int)b)+((unsigned int)1))/((unsigned int)2)+((unsigned int)a)))
#define TRIPLE(a,b,c)  ((unsigned int)(PAIR((unsigned int)c,PAIR(a,b))))


/* Inspection of BDD nodes */
#define ISCONST(a) ((a) < 2)
#define ISNONCONST(a) ((a) >= 2)
#define ISONE(a)   ((a) == 1)
#define ISZERO(a)  ((a) == 0)

static ALWAYS_INLINE unsigned LEVEL(const BddNode * const p) {
    return p->level;
}

static ALWAYS_INLINE unsigned LEVEL(const BDD i) {
    return LEVEL(bddnodes + i);
}

static ALWAYS_INLINE int LOW(const BddNode * const p) {
    return p->low;
}

static ALWAYS_INLINE int LOW(const BDD i) {
    return LOW(bddnodes + i);
}

static ALWAYS_INLINE int HIGH(const BddNode * const p) {
    return p->high;
}

static ALWAYS_INLINE int HIGH(const BDD i) {
    return HIGH(bddnodes + i);
}

/* Stacking for garbage collector */
#define INITREF bddrefstacktop = bddrefstack

static ALWAYS_INLINE BDD PUSHREF(const BDD n) {
    *bddrefstacktop++ = n;
    return n;
}

static ALWAYS_INLINE BDD READREF(const int i) {
    return bddrefstacktop[-i];
}

static ALWAYS_INLINE void POPREF(const int i) {
    bddrefstacktop -= i;
}

#define BDDONE 1
#define BDDZERO 0

#ifndef CLOCKS_PER_SEC
/* Pass `CPPFLAGS=-DDEFAULT_CLOCK=1000' as an argument to ./configure
     to override this setting.  */
# ifndef DEFAULT_CLOCK
#  define DEFAULT_CLOCK 60
# endif
# define CLOCKS_PER_SEC DEFAULT_CLOCK
#endif

#define DEFAULTMAXNODEINC 50000

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define NEW(t,n) ( (t*)malloc(sizeof(t)*(n)) )

/*=== KERNEL PROTOTYPES ================================================*/

#ifdef CPLUSPLUS
extern "C" {
#endif

extern void   bdd_error(const unsigned code);
extern int    bdd_makenode(unsigned int, int, int);
extern int    bdd_noderesize(int);
extern void   bdd_checkreorder(void);
extern void   bdd_mark(int);
extern void   bdd_mark_upto(int, int);
extern void   bdd_markcount(int, int*);
extern void   bdd_unmark(int);
extern void   bdd_unmark_upto(int, int);
extern void   bdd_register_pair(bddPair*);
extern int   *fdddec2bin(int, int);

extern int    bdd_operator_init(int);
extern void   bdd_operator_done(void);
extern void   bdd_operator_varresize(void);
extern void   bdd_operator_reset(void);

extern void   bdd_pairs_init(void);
extern void   bdd_pairs_done(void);
extern int    bdd_pairs_resize(int,int);
extern void   bdd_pairs_vardown(int);

extern void   bdd_reorder_init(void);
extern void   bdd_reorder_done(void);
extern int    bdd_reorder_ready(void);
extern void   bdd_reorder_auto(void);
extern int    bdd_reorder_vardown(int);
extern int    bdd_reorder_varup(int);

extern void   bdd_cpp_init(void);

#ifdef CPLUSPLUS
}
#endif


/*=== INPUT VALIDATION ==================================================*/

inline void CHECK(const BDD r) {
    if (UNLIKELY(!bddrunning)) {
        bdd_error(BDD_RUNNING);
    } else if (UNLIKELY(r < 0 || r >= bddnodesize)) {
        bdd_error(BDD_ILLBDD);
    } else if (UNLIKELY(r >= 2 && LOW(r) == -1)) {
        bdd_error(BDD_ILLBDD);
    }
}

#endif /* _KERNEL_H */


/* EOF */

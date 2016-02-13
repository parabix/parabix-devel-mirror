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
  $Header: /cvsroot/buddy/buddy/src/bdd.h,v 1.1.1.1 2004/06/25 13:22:09 haimcohen Exp $
  FILE:  bdd.h
  DESCR: C,C++ User interface for the BDD package
  AUTH:  Jorn Lind
  DATE:  (C) feb 1997
*************************************************************************/

#ifndef _BDD_H
#define _BDD_H

#include <string>

/* Allow this headerfile to define C++ constructs if requested */

#include <stdio.h>

/*=== Defined operators for apply calls ================================*/

#define bddop_and       0
#define bddop_xor       1
#define bddop_or        2
#define bddop_nand      3
#define bddop_nor       4
#define bddop_imp       5
#define bddop_biimp     6
#define bddop_diff      7
#define bddop_less      8
#define bddop_invimp    9

   /* Should *not* be used in bdd_apply calls !!! */
#define bddop_not      10
#define bddop_simplify 11


/*=== User BDD types ===================================================*/

typedef int BDD;
typedef BDD bdd;

typedef struct s_bddPair
{
   BDD *result;
   int last;
   int id;
   struct s_bddPair *next;
} bddPair;


/*=== Status information ===============================================*/

/*
NAME    {* bddStat *}
SECTION {* kernel *}
SHORT   {* Status information about the bdd package *}
PROTO   {* typedef struct s_bddStat
{
   long int produced;
   int nodenum;
   int maxnodenum;
   int freenodes;
   int minfreenodes;
   int varnum;
   int cachesize;
   int gbcnum;
} bddStat;  *}
DESCR   {* The fields are \\[\baselineskip] \begin{tabular}{lp{10cm}}
  {\tt produced}     & total number of new nodes ever produced \\
  {\tt nodenum}      & currently allocated number of bdd nodes \\
  {\tt maxnodenum}   & user defined maximum number of bdd nodes \\
  {\tt freenodes}    & number of currently free nodes \\
  {\tt minfreenodes} & minimum number of nodes that should be left after a
                       garbage collection. \\
  {\tt varnum}       & number of defined bdd variables \\
  {\tt cachesize}    & number of entries in the internal caches \\
  {\tt gbcnum}       & number of garbage collections done until now
  \end{tabular} *}
ALSO    {* bdd\_stats *}
*/
typedef struct s_bddStat
{
   long int produced;
   int nodenum;
   int maxnodenum;
   int freenodes;
   int minfreenodes;
   int varnum;
   int cachesize;
   int gbcnum;
} bddStat;


/*
NAME    {* bddGbcStat *}
SECTION {* kernel *}
SHORT   {* Status information about garbage collections *}
PROTO   {* typedef struct s_bddGbcStat
{
   int nodes;
   int freenodes;
   long time;
   long sumtime;
   int num;
} bddGbcStat;  *}
DESCR   {* The fields are \\[\baselineskip] \begin{tabular}{ll}
  {\tt nodes}     & Total number of allocated nodes in the nodetable \\
  {\tt freenodes} & Number of free nodes in the nodetable \\
  {\tt time}      & Time used for garbage collection this time \\
  {\tt sumtime}   & Total time used for garbage collection \\
  {\tt num}       & number of garbage collections done until now
  \end{tabular} *}
ALSO    {* bdd\_gbc\_hook *}
*/
typedef struct s_bddGbcStat
{
   int nodes;
   int freenodes;
   long time;
   long sumtime;
   int num;
} bddGbcStat;


/*
NAME    {* bddCacheStat *}
SECTION {* kernel *}
SHORT   {* Status information about cache usage *}
PROTO   {* typedef struct s_bddCacheStat
{
   long unsigned int uniqueAccess;
   long unsigned int uniqueChain;
   long unsigned int uniqueHit;
   long unsigned int uniqueMiss;
   long unsigned int opHit;
   long unsigned int opMiss;
   long unsigned int swapCount;
} bddCacheStat; *}
DESCR   {* The fields are \\[\baselineskip] \begin{tabular}{ll}
  {\bf Name}         & {\bf Number of } \\
  uniqueAccess & accesses to the unique node table \\
  uniqueChain  & iterations through the cache chains in the unique node table\\
  uniqueHit    & entries actually found in the the unique node table \\
  uniqueMiss   & entries not found in the the unique node table \\
  opHit        & entries found in the operator caches \\
  opMiss       & entries not found in the operator caches \\
  swapCount    & number of variable swaps in reordering \\
\end{tabular} *}
ALSO    {* bdd\_cachestats *}
*/
typedef struct s_bddCacheStat
{
   long unsigned int uniqueAccess;
   long unsigned int uniqueChain;
   long unsigned int uniqueHit;
   long unsigned int uniqueMiss;
   long unsigned int opHit;
   long unsigned int opMiss;
   long unsigned int swapCount;
} bddCacheStat;

/*=== BDD interface prototypes =========================================*/

/*
NAME    {* bdd\_relprod *}
SECTION {* operator *}
SHORT   {* relational product *}
PROTO   {* #define bdd_relprod(a,b,var) bdd_appex(a,b,bddop_and,var) *}
DESCR   {* Calculates the relational product of {\tt a} and {\tt b} as
           {\tt a AND b} with the variables in {\tt var} quantified out
	   afterwards. *}
RETURN  {* The relational product or {\tt bddfalse} on errors. *}
ALSO    {* bdd\_appex *}
*/
#define bdd_relprod(a,b,var) bdd_appex((a),(b),bddop_and,(var))


  /* In file "kernel.c" */

#ifdef CPLUSPLUS
extern "C" {
#endif

typedef void (*bddinthandler)(int);
typedef void (*bddgbchandler)(int,bddGbcStat*);
typedef void (*bdd2inthandler)(int,int);
typedef int  (*bddsizehandler)(void);
typedef void (*bddfilehandler)(FILE *, int);
typedef void (*bddallsathandler)(char*, int);
   
extern bddinthandler  bdd_error_hook(bddinthandler);
extern bddgbchandler  bdd_gbc_hook(bddgbchandler);
extern bdd2inthandler bdd_resize_hook(bdd2inthandler);
extern bddinthandler  bdd_reorder_hook(bddinthandler);
extern bddfilehandler bdd_file_hook(bddfilehandler);
   
extern int      bdd_init(int, int);
extern void     bdd_done(void);
extern int      bdd_setvarnum(int);
extern int      bdd_extvarnum(int);
extern int      bdd_isrunning(void);
extern int      bdd_setmaxnodenum(int);
extern int      bdd_setmaxincrease(int);
extern int      bdd_setminfreenodes(int);
extern int      bdd_getnodenum(void);
extern int      bdd_getallocnum(void);
extern void     bdd_stats(bddStat *);
extern void     bdd_cachestats(bddCacheStat *);
extern void     bdd_fprintstat(FILE *);
extern void     bdd_printstat(void);
extern void     bdd_default_gbchandler(int, bddGbcStat *);
extern void     bdd_default_errhandler(int);
extern std::string bdd_errstring(const int);
extern void     bdd_clear_error(void);
extern int      bdd_varnum(void);
extern BDD      bdd_ithvar(int);
extern BDD      bdd_nithvar(int);
extern int      bdd_var(BDD);
extern BDD      bdd_low(BDD);
extern BDD      bdd_high(BDD);
extern int      bdd_varlevel(int);
extern BDD      bdd_addref(BDD);
extern BDD      bdd_addref(BDD, unsigned int);
extern BDD      bdd_delref(BDD);
extern BDD      bdd_recursive_deref(BDD root);
extern void     bdd_gbc(void);
extern int      bdd_scanset(BDD, int**, int*);
extern BDD      bdd_makeset(int *, int);
extern bddPair* bdd_newpair(void);
extern int      bdd_setpair(bddPair*, int, int);
extern int      bdd_setpairs(bddPair*, int*, int*, int);
extern int      bdd_setbddpair(bddPair*, int, BDD);
extern int      bdd_setbddpairs(bddPair*, int*, BDD*, int);
extern void     bdd_resetpair(bddPair *);
extern void     bdd_freepair(bddPair*);

  /* In bddop.c */

extern int      bdd_setcacheratio(int);
extern BDD      bdd_buildcube(int, int, BDD *);
extern BDD      bdd_ibuildcube(int, int, int *);
extern BDD      bdd_not(BDD);
extern BDD      bdd_apply(BDD, BDD, int);
extern BDD      bdd_ite(BDD, BDD, BDD);
extern BDD      bdd_restrict(BDD, BDD);
extern BDD      bdd_constrain(BDD, BDD);
extern BDD      bdd_replace(BDD, bddPair*);
extern BDD      bdd_compose(BDD, BDD, BDD);
extern BDD      bdd_veccompose(BDD, bddPair*);
extern BDD      bdd_simplify(BDD, BDD);
extern BDD      bdd_exist(BDD, BDD);
extern BDD      bdd_forall(BDD, BDD);
extern BDD      bdd_unique(BDD, BDD);
extern BDD      bdd_appex(BDD, BDD, int, BDD);
extern BDD      bdd_appall(BDD, BDD, int, BDD);
extern BDD      bdd_appuni(BDD, BDD, int, BDD);
extern BDD      bdd_support(BDD);
extern BDD      bdd_satone(BDD);
extern BDD      bdd_satoneset(BDD, BDD, BDD);
extern BDD      bdd_fullsatone(BDD);
extern void     bdd_allsat(BDD r, bddallsathandler handler);
extern double   bdd_satcount(BDD);
extern double   bdd_satcountset(BDD, BDD);
extern double   bdd_satcountln(BDD);
extern double   bdd_satcountlnset(BDD, BDD);
extern int      bdd_nodecount(BDD);
extern int      bdd_anodecount(BDD *, int);
extern int*     bdd_varprofile(BDD);
extern double   bdd_pathcount(BDD);

   
/* In file "bddio.c" */

extern void     bdd_printall(void);
extern void     bdd_fprintall(FILE *);
extern void     bdd_fprinttable(FILE *, BDD);
extern void     bdd_printtable(BDD);
extern void     bdd_fprintset(FILE *, BDD);
extern void     bdd_printset(BDD);
extern int      bdd_fnprintdot(char *, BDD);
extern void     bdd_fprintdot(FILE *, BDD);
extern void     bdd_printdot(BDD);
extern int      bdd_fnsave(char *, BDD);
extern int      bdd_save(FILE *, BDD);
extern int      bdd_fnload(char *, BDD *);
extern int      bdd_load(FILE *ifile, BDD *);

/* In file reorder.c */

extern int      bdd_swapvar(int v1, int v2);
extern void     bdd_default_reohandler(int);
extern void     bdd_reorder(int);
extern int      bdd_reorder_gain(void);
extern bddsizehandler bdd_reorder_probe(bddsizehandler);
extern void     bdd_clrvarblocks(void);
extern int      bdd_addvarblock(BDD, int);
extern int      bdd_intaddvarblock(int, int, int);
extern void     bdd_varblockall(void);
extern bddfilehandler bdd_blockfile_hook(bddfilehandler);
extern int      bdd_autoreorder(int);
extern int      bdd_autoreorder_times(int, int);
extern int      bdd_var2level(int);
extern int      bdd_level2var(int);
extern int      bdd_getreorder_times(void);
extern int      bdd_getreorder_method(void);
extern void     bdd_enable_reorder(void);
extern void     bdd_disable_reorder(void);
extern int      bdd_reorder_verbose(int);
extern void     bdd_setvarorder(int *);
extern void     bdd_printorder(void);
extern void     bdd_fprintorder(FILE *);

#ifdef CPLUSPLUS
}
#endif


/*=== BDD constants ====================================================*/

static inline BDD bdd_one(void) {
   return 1;
}

static inline BDD bdd_zero(void) {
   return 0;
}

static inline BDD bdd_constant(const BDD var) {
    return var == 0 || var == 1;
}

/*=== BDD helpers ====================================================*/

static inline BDD bdd_and(BDD l, BDD r) {
    return bdd_apply(l, r, bddop_and);
}

static inline BDD bdd_or(BDD l, BDD r) {
    return bdd_apply(l, r, bddop_or);
}

static inline BDD bdd_xor(BDD l, BDD r) {
    return bdd_apply(l, r, bddop_xor);
}

static inline BDD bdd_nor(BDD l, BDD r) {
    return bdd_apply(l, r, bddop_nor);
}

static inline BDD bdd_imp(BDD l, BDD r) {
    return bdd_apply(l, r, bddop_imp);
}

static inline BDD bdd_biimp(BDD l, BDD r) {
    return bdd_apply(l, r, bddop_biimp);
}

/*=== Reordering algorithms ============================================*/

#define BDD_REORDER_NONE     0
#define BDD_REORDER_WIN2     1
#define BDD_REORDER_WIN2ITE  2
#define BDD_REORDER_SIFT     3
#define BDD_REORDER_SIFTITE  4
#define BDD_REORDER_WIN3     5
#define BDD_REORDER_WIN3ITE  6
#define BDD_REORDER_RANDOM   7

#define BDD_REORDER_FREE     0
#define BDD_REORDER_FIXED    1


/*=== Error codes ======================================================*/

#define BDD_MEMORY (-1)   /* Out of memory */
#define BDD_VAR (-2)      /* Unknown variable */
#define BDD_RANGE (-3)    /* Variable value out of range (not in domain) */
#define BDD_DEREF (-4)    /* Removing external reference to unknown node */
#define BDD_RUNNING (-5)  /* Called bdd_init() twice whithout bdd_done() */
#define BDD_FILE (-6)     /* Some file operation failed */
#define BDD_FORMAT (-7)   /* Incorrect file format */
#define BDD_ORDER (-8)    /* Vars. not in order for vector based functions */
#define BDD_BREAK (-9)    /* User called break */
#define BDD_VARNUM (-10)  /* Different number of vars. for vector pair */
#define BDD_NODES (-11)   /* Tried to set max. number of nodes to be fewer */
                          /* than there already has been allocated */
#define BDD_OP (-12)      /* Unknown operator */
#define BDD_VARSET (-13)  /* Illegal variable set */
#define BDD_VARBLK (-14)  /* Bad variable block operation */
#define BDD_DECVNUM (-15) /* Trying to decrease the number of variables */
#define BDD_REPLACE (-16) /* Replacing to already existing variables */
#define BDD_NODENUM (-17) /* Number of nodes reached user defined maximum */
#define BDD_ILLBDD (-18)  /* Illegal bdd argument */
#define BDD_SIZE (-19)    /* Illegal size argument */

#define BVEC_SIZE (-20)    /* Mismatch in bitvector size */
#define BVEC_SHIFT (-21)   /* Illegal shift-left/right parameter */
#define BVEC_DIVZERO (-22) /* Division by zero */

#define BDD_ERRNUM 24

#ifdef CPLUSPLUS
#include <iostream>

/*=== Iostream printing ================================================*/

class bdd_ioformat
{
 public:
   bdd_ioformat(int f) { format=f; }
 private:
   bdd_ioformat(void)  { }
   int format;
   static int curformat;

   friend std::ostream &operator<<(std::ostream &, const bdd_ioformat &);
   friend std::ostream &operator<<(std::ostream &, const bdd &);
};

std::ostream &operator<<(std::ostream &, const bdd &);
std::ostream &operator<<(std::ostream &, const bdd_ioformat &);

extern bdd_ioformat bddset;
extern bdd_ioformat bddtable;
extern bdd_ioformat bdddot;
extern bdd_ioformat bddall;
extern bdd_ioformat fddset;

typedef void (*bddstrmhandler)(std::ostream &, int);

extern bddstrmhandler bdd_strm_hook(bddstrmhandler);

#endif /* CPLUSPLUS */

#endif /* _BDD_H */

/* EOF */

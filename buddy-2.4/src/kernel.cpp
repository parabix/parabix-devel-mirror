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
  $Header: /cvsroot/buddy/buddy/src/kernel.c,v 1.2 2004/07/13 21:04:36 haimcohen Exp $
  FILE:  kernel.c
  DESCR: implements the bdd kernel functions.
  AUTH:  Jorn Lind
  DATE:  (C) june 1997

  WARNING: Do not use pointers to nodes across makenode calls,
           as makenode may resize/move the nodetable.

*************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <assert.h>
#include <stdexcept>

#include "kernel.h"
#include "cache.h"
#include "prime.h"

/*************************************************************************
  Various definitions and global variables
*************************************************************************/

/*=== INTERNAL DEFINITIONS =============================================*/

/* Min. number of nodes (%) that has to be left after a garbage collect
   unless a resize should be done. */
static int minfreenodes=20;


/*=== GLOBAL KERNEL VARIABLES ==========================================*/

int          bddrunning;            /* Flag - package initialized */
int          bddnodesize;           /* Number of allocated nodes */
int          bddmaxnodesize;        /* Maximum allowed number of nodes */
int          bddmaxnodeincrease;    /* Max. # of nodes used to inc. table */
BddNode*     bddnodes;          /* All of the bdd nodes */
int          bddfreepos;        /* First free node */
int          bddfreenum;        /* Number of free nodes */
long int     bddproduced;       /* Number of new nodes ever produced */
int          bddvarnum;         /* Number of defined BDD variables */
int*         bddrefstack;       /* Internal node reference stack */
int*         bddrefstacktop;    /* Internal node reference stack top */
int*         bddvar2level;      /* Variable -> level table */
int*         bddlevel2var;      /* Level -> variable table */
int          bddresized;        /* Flag indicating a resize of the nodetable */

/*=== PRIVATE KERNEL VARIABLES =========================================*/

static BDD*     bddvarset;             /* Set of defined BDD variables */
static int      gbcollectnum;          /* Number of garbage collections */
static int      cachesize;             /* Size of the operator caches */
static long int gbcclock;              /* Clock ticks used in GBC */
static int      usednodes_nextreorder; /* When to do reorder next time */
static bddinthandler  err_handler;     /* Error handler */
static bdd2inthandler resize_handler;  /* Node-table-resize handler */


/*=== OTHER INTERNAL DEFINITIONS =======================================*/

#define NODEHASH(lvl,l,h) (TRIPLE(lvl,l,h) % bddnodesize)


/*************************************************************************
  BDD misc. user operations
*************************************************************************/

/*
NAME   {* bdd\_init *}
SECTION {* kernel *}
SHORT  {* initializes the BDD package *}
PROTO  {* int bdd_init(int nodesize, int cachesize) *}
DESCR  {* This function initiates the bdd package and {\em must} be called
          before any bdd operations are done. The argument {\tt nodesize}
      is the initial number of nodes in the nodetable and {\tt cachesize}
      is the fixed size of the internal caches. Typical values for
      {\tt nodesize} are 10000 nodes for small test examples and up to
      1000000 nodes for large examples. A cache size of 10000 seems to
      work good even for large examples, but lesser values should do it
      for smaller examples.

      The number of cache entries can also be set to depend on the size
      of the nodetable using a call to {\tt bdd\_setcacheratio}.

      The initial number of nodes is not critical for any bdd operation
      as the table will be resized whenever there are to few nodes left
      after a garbage collection. But it does have some impact on the
      efficency of the operations. *}
RETURN {* If no errors occur then 0 is returned, otherwise
          a negative error code. *}
ALSO   {* bdd\_done, bdd\_resize\_hook *}
*/
int bdd_init(int initnodesize, int cs) {

    if (bddrunning)
        bdd_error(BDD_RUNNING);

    bddnodesize = bdd_prime_gte(initnodesize);

    if ((bddnodes=(BddNode*)malloc(sizeof(BddNode)*bddnodesize)) == nullptr) {
        bdd_error(BDD_MEMORY);
    }

    bddresized = 0;

    for (unsigned i = 0 ; i != 2 ; ++i) {
        BddNode & n = bddnodes[i];
        n.refcount = MAXREF;
        n.level = 0;
        n.low = i;
        n.high = i;
        n.hash = 0;
        n.next = i + 1;
    }
    for (unsigned i = 2; i != bddnodesize ; ++i) {
        BddNode & n = bddnodes[i];
        n.refcount = 0;
        n.level = 0;
        n.low = -1;
        n.high = -1;
        n.hash = 0;
        n.next = i + 1;
    }
    bddnodes[bddnodesize - 1].next = 0;

    const auto err = bdd_operator_init(cs);
    if (err < 0) {
        bdd_done();
        return err;
    }

    bddfreepos = 2;
    bddfreenum = bddnodesize-2;
    bddrunning = 1;
    bddvarnum = 0;
    gbcollectnum = 0;
    gbcclock = 0;
    cachesize = cs;
    usednodes_nextreorder = bddnodesize;
    bddmaxnodeincrease = DEFAULTMAXNODEINC;

    bdd_resize_hook(nullptr);
    bdd_pairs_init();
    bdd_reorder_init();

    return 0;
}


/*
NAME    {* bdd\_done*}
SECTION {* kernel *}
SHORT {* resets the bdd package *}
PROTO {* void bdd_done(void) *}
DESCR {* This function frees all memory used by the bdd package and resets
         the package to it's initial state.*}
ALSO  {* bdd\_init *}
*/
void bdd_done(void)
{
    /*sanitycheck(); FIXME */
    // bdd_fdd_done();
    bdd_reorder_done();
    bdd_pairs_done();

    free(bddnodes);
    free(bddrefstack);
    free(bddvarset);
    free(bddvar2level);
    free(bddlevel2var);

    bddnodes = nullptr;
    bddrefstack = nullptr;
    bddvarset = nullptr;

    bdd_operator_done();

    bddrunning = 0;
    bddnodesize = 0;
    bddmaxnodesize = 0;
    bddvarnum = 0;
    bddproduced = 0;

    err_handler = nullptr;
    resize_handler = nullptr;
}


/*
NAME    {* bdd\_setvarnum *}
SECTION {* kernel *}
SHORT   {* set the number of used bdd variables *}
PROTO   {* int bdd_setvarnum(int num) *}
DESCR   {* This function is used to define the number of variables used in
           the bdd package. It may be called more than one time, but only
       to increase the number of variables. The argument
       {\tt num} is the number of variables to use. *}
RETURN  {* Zero on succes, otherwise a negative error code. *}
ALSO    {* bdd\_ithvar, bdd\_varnum, bdd\_extvarnum *}
*/
int bdd_setvarnum(int num)
{
    int bdv;
    int oldbddvarnum = bddvarnum;

    bdd_disable_reorder();

    if (num < 1  ||  num > MAXVAR)
    {
        bdd_error(BDD_RANGE);
        return bdd_zero();
    }

    if (num < bddvarnum)
        bdd_error(BDD_DECVNUM);
    if (num == bddvarnum)
        return 0;

    if (bddvarset == nullptr)
    {
        if ((bddvarset=(BDD*)malloc(sizeof(BDD)*num*2)) == nullptr)
            bdd_error(BDD_MEMORY);
        if ((bddlevel2var=(int*)malloc(sizeof(int)*(num+1))) == nullptr)
        {
            free(bddvarset);
            bdd_error(BDD_MEMORY);
        }
        if ((bddvar2level=(int*)malloc(sizeof(int)*(num+1))) == nullptr)
        {
            free(bddvarset);
            free(bddlevel2var);
            bdd_error(BDD_MEMORY);
        }
    }
    else
    {
        if ((bddvarset=(BDD*)realloc(bddvarset,sizeof(BDD)*num*2)) == nullptr)
            bdd_error(BDD_MEMORY);
        if ((bddlevel2var=(int*)realloc(bddlevel2var,sizeof(int)*(num+1))) == nullptr)
        {
            free(bddvarset);
            bdd_error(BDD_MEMORY);
        }
        if ((bddvar2level=(int*)realloc(bddvar2level,sizeof(int)*(num+1))) == nullptr)
        {
            free(bddvarset);
            free(bddlevel2var);
            bdd_error(BDD_MEMORY);
        }
    }

    if (bddrefstack != nullptr)
        free(bddrefstack);
    bddrefstack = bddrefstacktop = (int*)malloc(sizeof(int)*(num*2+4));

    for(bdv=bddvarnum ; bddvarnum < num; bddvarnum++)
    {
        bddvarset[bddvarnum*2] = PUSHREF( bdd_makenode(bddvarnum, 0, 1) );
        bddvarset[bddvarnum*2+1] = bdd_makenode(bddvarnum, 1, 0);
        POPREF(1);

        bddnodes[bddvarset[bddvarnum*2]].refcount = MAXREF;
        bddnodes[bddvarset[bddvarnum*2+1]].refcount = MAXREF;
        bddlevel2var[bddvarnum] = bddvarnum;
        bddvar2level[bddvarnum] = bddvarnum;
    }

    bddnodes[0].level = num;
    bddnodes[1].level = num;
    bddvar2level[num] = num;
    bddlevel2var[num] = num;

    bdd_pairs_resize(oldbddvarnum, bddvarnum);
    bdd_operator_varresize();

    bdd_enable_reorder();

    return 0;
}


/*
NAME    {* bdd\_extvarnum *}
SECTION {* kernel *}
SHORT   {* add extra BDD variables *}
PROTO   {* int bdd_extvarnum(int num) *}
DESCR   {* Extends the current number of allocated BDD variables with
           {\tt num} extra variables. *}
RETURN  {* The old number of allocated variables or a negative error code. *}
ALSO    {* bdd\_setvarnum, bdd\_ithvar, bdd\_nithvar *}
*/
int bdd_extvarnum(int num)
{
    int start = bddvarnum;

    if (num < 0  ||  num > 0x3FFFFFFF)
        bdd_error(BDD_RANGE);

    bdd_setvarnum(bddvarnum+num);
    return start;
}

/*
NAME  {* bdd\_resize\_hook  *}
SECTION {* kernel *}
SHORT {* set a handler for nodetable resizes *}
PROTO {* bdd2inthandler bdd_resize_hook(bdd2inthandler handler) *}
DESCR {* Whenever it is impossible to get enough free nodes by a garbage
         collection then the node table is resized and a test is done to see
     if a handler is supllied by the user for this event. If so then
     it is called with {\tt oldsize} being the old nodetable size and
     {\tt newsize} being the new nodetable size.

     This function sets the handler to be {\tt handler}. If a {\tt nullptr}
     argument is supplied then no calls are made when a table resize
     is done. No default handler is supplied.

     Any handler should be defined like this:
     \begin{verbatim}
void my_resize_handler(int oldsize, int newsize)
{
   ...
}
\end{verbatim} *}
RETURN {* The previous handler *}
ALSO  {* bdd\_gbc\_hook, bdd\_reorder\_hook, bdd\_setminfreenodes  *}
*/
bdd2inthandler bdd_resize_hook(bdd2inthandler handler)
{
    bdd2inthandler tmp = handler;
    resize_handler = handler;
    return tmp;
}


/*
NAME    {* bdd\_setmaxincrease *}
SECTION {* kernel *}
SHORT   {* set max. number of nodes used to increase node table *}
PROTO   {* int bdd_setmaxincrease(int size) *}
DESCR   {* The node table is expanded by doubling the size of the table
           when no more free nodes can be found, but a maximum for the
       number of new nodes added can be set with {\tt bdd\_maxincrease}
       to {\tt size} nodes. The default is 50000 nodes (1 Mb). *}
RETURN  {* The old threshold on succes, otherwise a negative error code. *}
ALSO    {* bdd\_setmaxnodenum, bdd\_setminfreenodes *}
*/
int bdd_setmaxincrease(int size)
{
    int old = bddmaxnodeincrease;

    if (size < 0)
        bdd_error(BDD_SIZE);

    bddmaxnodeincrease = size;
    return old;
}

/*
NAME    {* bdd\_setmaxnodenum *}
SECTION {* kernel *}
SHORT {* set the maximum available number of bdd nodes *}
PROTO {* int bdd_setmaxnodenum(int size) *}
DESCR {* This function sets the maximal number of bdd nodes the package may
         allocate before it gives up a bdd operation. The
     argument {\tt size} is the absolute maximal number of nodes there
     may be allocated for the nodetable. Any attempt to allocate more
     nodes results in the constant false being returned and the error
     handler being called until some nodes are deallocated.
     A value of 0 is interpreted as an unlimited amount.
     It is {\em not} possible to specify
     fewer nodes than there has already been allocated. *}
RETURN {* The old threshold on succes, otherwise a negative error code. *}
ALSO   {* bdd\_setmaxincrease, bdd\_setminfreenodes *}
*/
int bdd_setmaxnodenum(int size)
{
    if (size > bddnodesize || size == 0) {
        int old = bddmaxnodesize;
        bddmaxnodesize = size;
        return old;
    }

    bdd_error(BDD_NODES);
    return bddmaxnodesize;
}


/*
NAME    {* bdd\_setminfreenodes *}
SECTION {* kernel *}
SHORT   {* set min. no. of nodes to be reclaimed after GBC. *}
PROTO   {* int bdd_setminfreenodes(int n) *}
DESCR   {* Whenever a garbage collection is executed the number of free
           nodes left are checked to see if a resize of the node table is
       required. If $X = (\mathit{bddfreenum}*100)/\mathit{maxnum}$
       is less than or
       equal to {\tt n} then a resize is initiated. The range of
       {\tt X} is of course $0\ldots 100$ and has some influence
       on how fast the package is. A low number means harder attempts
       to avoid resizing and saves space, and a high number reduces
       the time used in garbage collections. The default value is
       20. *}
RETURN  {* The old threshold on succes, otherwise a negative error code. *}
ALSO    {* bdd\_setmaxnodenum, bdd\_setmaxincrease *}
*/
int bdd_setminfreenodes(int mf)
{
    int old = minfreenodes;

    if (mf<0 || mf>100)
        bdd_error(BDD_RANGE);

    minfreenodes = mf;
    return old;
}


/*
NAME    {* bdd\_getnodenum *}
SECTION {* kernel *}
SHORT   {* get the number of active nodes in use *}
PROTO   {* int bdd_getnodenum(void) *}
DESCR   {* Returns the number of nodes in the nodetable that are
           currently in use. Note that dead nodes that have not been
       reclaimed yet
       by a garbage collection are counted as active. *}
RETURN  {* The number of nodes. *}
ALSO    {* bdd\_getallocnum, bdd\_setmaxnodenum *}
*/
int bdd_getnodenum(void) {
    return bddnodesize - bddfreenum;
}


/*
NAME    {* bdd\_getallocnum *}
SECTION {* kernel *}
SHORT   {* get the number of allocated nodes *}
PROTO   {* int bdd_getallocnum(void) *}
DESCR   {* Returns the number of nodes currently allocated. This includes
           both dead and active nodes. *}
RETURN  {* The number of nodes. *}
ALSO    {* bdd\_getnodenum, bdd\_setmaxnodenum *}
*/
int bdd_getallocnum(void) {
    return bddnodesize;
}


/*
NAME    {* bdd\_isrunning *}
SECTION {* kernel *}
SHORT   {* test whether the package is started or not *}
PROTO   {* void bdd_isrunning(void) *}
DESCR   {* This function tests the internal state of the package and returns
          a status. *}
RETURN  {* 1 (true) if the package has been started, otherwise 0. *}
ALSO    {* bdd\_init, bdd\_done *}
*/
int bdd_isrunning(void) {
    return bddrunning;
}

/*************************************************************************
  Error handler
*************************************************************************/

void bdd_error(const unsigned code) {
    if (code >= BDD_ERRNUM) {
        throw std::runtime_error("Unknown error " + std::to_string(code));
    } else {
        static std::string errorstrings[BDD_ERRNUM] = {
            "Out of memory", "Unknown variable", "Value out of range",
            "Unknown BDD root dereferenced", "bdd_init() called twice",
            "File operation failed", "Incorrect file format",
            "Variables not in ascending order", "User called break",
            "Mismatch in size of variable sets",
            "Cannot allocate fewer nodes than already in use",
            "Unknown operator", "Illegal variable set",
            "Bad variable block operation",
            "Trying to decrease the number of variables",
            "Trying to replace with variables already in the bdd",
            "Number of nodes reached user defined maximum",
            "Unknown BDD - was not in node table",
            "Bad size argument",
            "Mismatch in bitvector size",
            "Illegal shift-left/right parameter",
            "Division by zero"
        };
        throw std::runtime_error(errorstrings[code]);
    }
}

/*************************************************************************
  BDD primitives
*************************************************************************/

/*
NAME    {* bdd\_ithvar *}
SECTION {* kernel *}
SHORT   {* returns a bdd representing the I'th variable *}
PROTO   {* BDD bdd_ithvar(int var) *}
DESCR   {* This function is used to get a bdd representing the I'th
           variable (one node with the childs true and false). The requested
       variable must be in the range define by {\tt
       bdd\_setvarnum} starting with 0 being the first. For ease
       of use then the bdd returned from {\tt bdd\_ithvar} does
       not have to be referenced counted with a call to {\tt
       bdd\_addref}. The initial variable order is defined by the
       the index {\tt var} that also defines the position in the
       variable order -- variables with lower indecies are before
       those with higher indecies. *}
RETURN  {* The I'th variable on succes, otherwise the constant false bdd *}
ALSO {* bdd\_setvarnum, bdd\_nithvar, bdd_true(), bdd_false() *} */
BDD bdd_ithvar(int var) {
    if (UNLIKELY(var < 0 || var >= bddvarnum)) {
        bdd_error(BDD_VAR);
    }
    return bddvarset[var * 2];
}


/*
NAME    {* bdd\_nithvar *}
SECTION {* kernel *}
SHORT   {* returns a bdd representing the negation of the I'th variable *}
PROTO   {* BDD bdd_nithvar(int var) *}
DESCR   {* This function is used to get a bdd representing the negation of
           the I'th variable (one node with the childs false and true).
       The requested variable must be in the range define by
       {\tt bdd\_setvarnum} starting with 0 being the first. For ease of
       use then the bdd returned from {\tt bdd\_nithvar} does not have
       to be referenced counted with a call to {\tt bdd\_addref}. *}
RETURN  {* The negated I'th variable on succes, otherwise the constant false bdd *}	   
ALSO    {* bdd\_setvarnum, bdd\_ithvar, bdd_true(), bdd_false() *}
*/
BDD bdd_nithvar(int var) {
    if (UNLIKELY(var < 0 || var >= bddvarnum)) {
        bdd_error(BDD_VAR);
    }
    return bddvarset[var*2+1];
}


/*
NAME    {* bdd\_varnum *}
SECTION {* kernel *}
SHORT   {* returns the number of defined variables *}
PROTO   {* int bdd_varnum(void) *}
DESCR   {* This function returns the number of variables defined by
           a call to {\tt bdd\_setvarnum}.*}
RETURN  {* The number of defined variables *}
ALSO    {* bdd\_setvarnum, bdd\_ithvar *}
*/
int bdd_varnum(void) {
    return bddvarnum;
}


/*
NAME    {* bdd\_var *}
SECTION {* info *}
SHORT   {* gets the variable labeling the bdd *}
PROTO   {* int bdd_var(BDD r) *}
DESCR   {* Gets the variable labeling the bdd {\tt r}. *}
RETURN  {* The variable number. *}
*/
int bdd_var(BDD root) {
    CHECK(root);
    if (ISCONST(root)) {
        bdd_error(BDD_ILLBDD);
    }
    return (bddlevel2var[LEVEL(root)]);
}


/*
NAME    {* bdd\_low *}
SECTION {* info *}
SHORT   {* gets the false branch of a bdd  *}
PROTO   {* BDD bdd_low(BDD r) *}
DESCR   {* Gets the false branch of the bdd {\tt r}.  *}
RETURN  {* The bdd of the false branch *}
ALSO    {* bdd\_high *}
*/
BDD bdd_low(BDD root) {
    CHECK(root);
    if (ISCONST(root)) {
        bdd_error(BDD_ILLBDD);
    }
    return (LOW(root));
}


/*
NAME    {* bdd\_high *}
SECTION {* info *}
SHORT   {* gets the true branch of a bdd  *}
PROTO   {* BDD bdd_high(BDD r) *}
DESCR   {* Gets the true branch of the bdd {\tt r}.  *}
RETURN  {* The bdd of the true branch *}
ALSO    {* bdd\_low *}
*/
BDD bdd_high(BDD root) {
    CHECK(root);
    if (ISCONST(root)) {
        bdd_error(BDD_ILLBDD);
    }
    return (HIGH(root));
}

/*************************************************************************
  Garbage collection and node referencing
*************************************************************************/

static void bdd_gbc_rehash(void)
{
    int n;

    bddfreepos = 0;
    bddfreenum = 0;

    for (n=bddnodesize-1 ; n>=2 ; n--)
    {
        BddNode *node = &bddnodes[n];

        if (LOW(node) != -1)
        {
            unsigned int hash;

            hash = NODEHASH(LEVEL(node), LOW(node), HIGH(node));
            node->next = bddnodes[hash].hash;
            bddnodes[hash].hash = n;
        }
        else
        {
            node->next = bddfreepos;
            bddfreepos = n;
            bddfreenum++;
        }
    }
}


void bdd_gbc(void)
{
    int *r;
    int n;

    for (r=bddrefstack ; r<bddrefstacktop ; r++)
        bdd_mark(*r);

    for (n=0 ; n<bddnodesize ; n++)
    {
        if (bddnodes[n].refcount > 0)
            bdd_mark(n);
        bddnodes[n].hash = 0;
    }

    bddfreepos = 0;
    bddfreenum = 0;

    for (n=bddnodesize-1 ; n>=2 ; n--)
    {
        BddNode *node = &bddnodes[n];

        if (MARKED(node) && LOW(node) != -1) {
            UNMARK(node);
            unsigned int hash = NODEHASH(LEVEL(node), LOW(node), HIGH(node));
            node->next = bddnodes[hash].hash;
            bddnodes[hash].hash = n;
        }
        else
        {
            node->low = -1;
            node->next = bddfreepos;
            bddfreepos = n;
            bddfreenum++;
        }
    }

    bdd_operator_reset();

    gbcollectnum++;

}

/*
NAME    {* bdd\_addref *}
SECTION {* kernel *}
SHORT   {* increases the reference count on a node *}
PROTO   {* BDD bdd_addref(BDD r) *}
DESCR   {* Reference counting is done on externaly referenced nodes only
           and the count for a specific node {\tt r} can and must be
       increased using this function to avoid loosing the node in the next
       garbage collection. *}
ALSO    {* bdd\_delref *}
RETURN  {* The BDD node {\tt r}. *}
*/
BDD bdd_addref(BDD root) {
    assert (bddrunning);
    if (ISCONST(root)) {
        return root;
    }
    if (LOW(root) == -1 || root >= bddnodesize) {
        bdd_error(BDD_ILLBDD);
    }

    INCREF(root);
    return root;
}

/*
NAME    {* bdd\_addref *}
SECTION {* kernel *}
SHORT   {* increases the reference count on a node by n *}
PROTO   {* BDD bdd_addref(BDD r) *}
DESCR   {* Reference counting is done on externaly referenced nodes only
           and the count for a specific node {\tt r} can and must be
       increased using this function to avoid loosing the node in the next
       garbage collection. *}
ALSO    {* bdd\_delref *}
RETURN  {* The BDD node {\tt r}. *}
*/
BDD bdd_addref(BDD root, unsigned int n) {
    assert (bddrunning);
    if (ISCONST(root)) {
        return root;
    }
    if (LOW(root) == -1 || root >= bddnodesize) {
        bdd_error(BDD_ILLBDD);
    }
    bddnodes[root].refcount = std::min<unsigned int>(n + bddnodes[root].refcount, MAXREF);
    return root;
}

/*
NAME    {* bdd\_delref *}
SECTION {* kernel *}
SHORT   {* decreases the reference count on a node *}
PROTO   {* BDD bdd_delref(BDD r) *}
DESCR   {* Reference counting is done on externaly referenced nodes only
           and the count for a specific node {\tt r} can and must be
       decreased using this function to make it possible to reclaim the
       node in the next garbage collection. *}
ALSO    {* bdd\_addref *}
RETURN  {* The BDD node {\tt r}. *}
*/
BDD bdd_delref(BDD root) {
    assert (bddrunning);
    if (ISCONST(root)) {
        return root;
    }
    if (LOW(root) == -1 || root >= bddnodesize) {
        bdd_error(BDD_ILLBDD);
    }
    /* if the following line is present, fails there much earlier */
    if (!HASREF(root)) bdd_error(BDD_BREAK); /* distinctive */

    DECREF(root);
    return root;
}

/*
NAME    {* bdd\_bdd_recursive\_deref *}
SECTION {* kernel *}
SHORT   {* recursively decreases the reference count on a node *}
PROTO   {* BDD bdd_delref(BDD r) *}
DESCR   {* Reference counting is done on externaly referenced nodes only
           and the count for a specific node {\tt r} can and must be
       decreased using this function to make it possible to reclaim the
       node in the next garbage collection. *}
ALSO    {* bdd\_addref *}
RETURN  {* The BDD node {\tt r}. *}
*/
BDD bdd_recursive_deref(BDD root) {
    assert (bddrunning);
    if (ISCONST(root)) {
        return root;
    }
    if (root >= bddnodesize)
        bdd_error(BDD_ILLBDD);
    BddNode * node = bddnodes + root;
    if (node->low == -1)
        bdd_error(BDD_ILLBDD);
    if (node->low == -1 || node->refcount == 0) {
        return root;
    }
    if (--node->refcount == 0) {
        bdd_recursive_deref(node->low);
        bdd_recursive_deref(node->high);
    }
    return root;
}


/*=== RECURSIVE MARK / UNMARK ==========================================*/

void bdd_mark(int i)
{
    BddNode *node;

    if (i < 2)
        return;

    node = &bddnodes[i];
    if (MARKED(node) || LOW(node) == -1)
        return;

    SETMARK(node);
    bdd_mark(LOW(node));
    bdd_mark(HIGH(node));
}


void bdd_mark_upto(int i, int level)
{
    BddNode *node = &bddnodes[i];

    if (i < 2)
        return;

    if (MARKED(node) || LOW(node) == -1)
        return;

    if (LEVEL(node) > level)
        return;

    SETMARK(node);
    bdd_mark_upto(LOW(node), level);
    bdd_mark_upto(HIGH(node), level);
}


void bdd_markcount(int i, int *cou)
{
    BddNode *node;

    if (i < 2)
        return;

    node = &bddnodes[i];
    if (MARKED(node)  ||  LOW(node) == -1)
        return;

    SETMARK(node);
    *cou += 1;

    bdd_markcount(LOW(node), cou);
    bdd_markcount(HIGH(node), cou);
}


void bdd_unmark(int i)
{
    if (i < 2)
        return;
    BddNode * node = &bddnodes[i];
    if (MARKED(node) && LOW(node) >= 0) {
        UNMARK(node);
        bdd_unmark(LOW(node));
        bdd_unmark(HIGH(node));
    }
}


void bdd_unmark_upto(int i, int level)
{
    if (i < 2)
        return;
    BddNode * node = &bddnodes[i];
    if (MARKED(node)) {
        UNMARK(node);
        if (LEVEL(node) <= level) {
            bdd_unmark_upto(LOW(node), level);
            bdd_unmark_upto(HIGH(node), level);
        }
    }
}


/*************************************************************************
  Unique node table functions
*************************************************************************/

int bdd_makenode(unsigned int level, int low, int high) {

    unsigned int hash;
    int res;

    assert (low >= 0);
    assert (high >= 0);

    /* check whether childs are equal */
    if (UNLIKELY(low == high)) {
        return low;
    }

    /* Try to find an existing node of this kind */
    hash = NODEHASH(level, low, high);
    res = bddnodes[hash].hash;

    while (res) {
        if (LEVEL(res) == level && LOW(res) == low && HIGH(res) == high) {
            return res;
        }
        res = bddnodes[res].next;
    }

    /* No existing node -> build one */

    /* Any free nodes to use ? */
    if (bddfreepos == 0) {

        /* Try to allocate more nodes */
        bdd_gbc();

        if ((bddnodesize - bddfreenum) >= usednodes_nextreorder && bdd_reorder_ready()) {
            throw reordering_required();
        }

        if ((bddfreenum*100) / bddnodesize <= minfreenodes) {
            bdd_noderesize(1);
            hash = NODEHASH(level, low, high);
        }

        /* Panic if that is not possible */
        if (bddfreepos == 0) {
            bdd_error(BDD_NODENUM);
        }
    }

    /* Build new node */
    res = bddfreepos;
    bddfreepos = bddnodes[bddfreepos].next;
    bddfreenum--;
    bddproduced++;

    BddNode * node = bddnodes + res;
    node->level = level;
    node->low = low;
    node->high = high;

    /* Insert node */
    node->next = bddnodes[hash].hash;
    bddnodes[hash].hash = res;

    return res;
}


int bdd_noderesize(int doRehash)
{
    const int oldsize = bddnodesize;

    if (bddnodesize >= bddmaxnodesize  &&  bddmaxnodesize > 0)
        return -1;

    bddnodesize = bddnodesize << 1;

    if (bddnodesize > oldsize + bddmaxnodeincrease)
        bddnodesize = oldsize + bddmaxnodeincrease;

    if (bddnodesize > bddmaxnodesize  &&  bddmaxnodesize > 0)
        bddnodesize = bddmaxnodesize;

    bddnodesize = bdd_prime_lte(bddnodesize);

    if (resize_handler != nullptr)
        resize_handler(oldsize, bddnodesize);

    BddNode * newnodes = (BddNode*)realloc(bddnodes, sizeof(BddNode)*bddnodesize);
    if (newnodes == nullptr)
        bdd_error(BDD_MEMORY);
    bddnodes = newnodes;

    if (doRehash)
        for (unsigned n=0 ; n<oldsize ; n++)
            bddnodes[n].hash = 0;

    for (unsigned n=oldsize ; n<bddnodesize ; n++) {
        bddnodes[n].refcount = 0;
        bddnodes[n].level = 0;
        bddnodes[n].low = -1;
        bddnodes[n].hash = 0;
        bddnodes[n].next = n + 1;
    }
    bddnodes[bddnodesize-1].next = bddfreepos;
    bddfreepos = oldsize;
    bddfreenum += bddnodesize - oldsize;

    if (doRehash)
        bdd_gbc_rehash();

    bddresized = 1;

    return 0;
}


void bdd_checkreorder(void)
{
    bdd_reorder_auto();

    /* Do not reorder before twice as many nodes have been used */
    usednodes_nextreorder = 2 * (bddnodesize - bddfreenum);

    /* And if very little was gained this time (< 20%) then wait until
     * even more nodes (upto twice as many again) have been used */
    const auto gain = bdd_reorder_gain();
    if (gain < 20) {
        usednodes_nextreorder += (usednodes_nextreorder * (20 - gain)) / 20;
    }
}


/*************************************************************************
  Variable sets
*************************************************************************/

/*
NAME    {* bdd\_scanset *}
SECTION {* kernel *}
SHORT   {* returns an integer representation of a variable set *}
PROTO   {* int bdd_scanset(BDD r, int **v, int *n) *}
DESCR   {* Scans a variable set {\tt r} and copies the stored variables into
           an integer array of variable numbers. The argument {\tt v} is
       the address of an integer pointer where the array is stored and
       {\tt n} is a pointer to an integer where the number of elements
       are stored. It is the users responsibility to make sure the
       array is deallocated by a call to {\tt free(v)}. The numbers
       returned are guaranteed to be in ascending order. *}
ALSO    {* bdd\_makeset *}
RETURN  {* Zero on success, otherwise a negative error code. *}
*/
int bdd_scanset(BDD r, int **varset, int *varnum)
{
    int n, num;

    CHECK(r);
    if (r < 2)
    {
        *varnum = 0;
        *varset = nullptr;
        return 0;
    }

    for (n=r, num=0 ; n > 1 ; n=HIGH(n))
        num++;

    if (((*varset) = (int *)malloc(sizeof(int)*num)) == nullptr)
        bdd_error(BDD_MEMORY);

    for (n=r, num=0 ; n > 1 ; n=HIGH(n))
        (*varset)[num++] = bddlevel2var[LEVEL(n)];

    *varnum = num;

    return 0;
}


/*
NAME    {* bdd\_makeset *}
SECTION {* kernel *}
SHORT   {* builds a BDD variable set from an integer array *}
PROTO   {* BDD bdd_makeset(int *v, int n) *}
DESCR   {* Reads a set of variable numbers from the integer array {\tt v}
           which must hold exactly {\tt n} integers and then builds a BDD
       representing the variable set.

       The BDD variable set is represented as the conjunction of
       all the variables in their positive form and may just as
       well be made that way by the user. The user should keep a
       reference to the returned BDD instead of building it every
       time the set is needed. *}
ALSO    {* bdd\_scanset *}
RETURN {* A BDD variable set. *} */
BDD bdd_makeset(int *varset, int varnum)
{
    int v, res=1;

    for (v=varnum-1 ; v>=0 ; v--)
    {
        BDD tmp;
        bdd_addref(res);
        tmp = bdd_apply(res, bdd_ithvar(varset[v]), bddop_and);
        bdd_delref(res);
        res = tmp;
    }

    return res;
}


/* EOF */

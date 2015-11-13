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
  $Header: /cvsroot/buddy/buddy/src/prime.c,v 1.1.1.1 2004/06/25 13:22:51 haimcohen Exp $
  FILE:  prime.c
  DESCR: Prime number calculations
  AUTH:  Jorn Lind
  DATE:  (C) feb 2001
*************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include "prime.h"

#define Random(i) ( (rand() % (i)) + 1 )
#define isEven(src) (!((src) & 0x1))
#define hasFactor(src,n) ( (((src)!=(n)) && ((src)%(n) == 0)) )
#define BitIsSet(src,b) ( ((src) & (1<<(b))) != 0 )

#define CHECKTIMES 20

static unsigned int mulmod(const uint64_t a, const uint64_t b, const uint64_t c) {
    return (a * b) % c;
}

/*************************************************************************
  Miller Rabin check
*************************************************************************/

static unsigned int numberOfBits(unsigned int src)
{
    unsigned int b;

    if (src == 0)
        return 0;

    for (b=(sizeof(unsigned int)*8)-1 ; b>0 ; --b)
        if (BitIsSet(src,b))
            return b+1;

    return 1;
}



static int isWitness(unsigned int witness, unsigned int src)
{
    unsigned int bitNum = numberOfBits(src-1)-1;
    unsigned int d = 1;
    int i;

    for (i=bitNum ; i>=0 ; --i)
    {
        unsigned int x = d;

        d = mulmod(d, d, src);

        if (d == 1  &&  x != 1  &&  x != src-1)
            return 1;

        if (BitIsSet(src-1,i))
            d = mulmod(d, witness, src);
    }

    return d != 1;
}


static int isMillerRabinPrime(unsigned int src)
{
    int n;

    for (n=0 ; n<CHECKTIMES ; ++n)
    {
        unsigned int witness = Random(src-1);

        if (isWitness(witness,src))
            return 0;
    }

    return 1;
}


/*************************************************************************
  Basic prime searching stuff
*************************************************************************/

static int hasEasyFactors(unsigned int src)
{
    return hasFactor(src, 3)
            || hasFactor(src, 5)
            || hasFactor(src, 7)
            || hasFactor(src, 11)
            || hasFactor(src, 13);
}


static int isPrime(unsigned int src)
{
    if (hasEasyFactors(src))
        return 0;

    return isMillerRabinPrime(src);
}


/*************************************************************************
  External interface
*************************************************************************/

unsigned int bdd_prime_gte(unsigned int src)
{
    if (isEven(src))
        ++src;

    while (!isPrime(src))
        src += 2;

    return src;
}


unsigned int bdd_prime_lte(unsigned int src)
{
    if (isEven(src))
        --src;

    while (!isPrime(src))
        src -= 2;

    return src;
}


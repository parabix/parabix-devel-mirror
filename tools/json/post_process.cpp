#include "post_process.h"

#include <cassert>
#include <cstdlib>
#include <string>
#include <llvm/Support/Compiler.h>
#include <llvm/Support/ErrorHandling.h>

void postproc_validateObjectsAndArrays(const uint8_t * ptr, uint64_t position) {
    printf("%ld %c\n", position, *ptr);
}
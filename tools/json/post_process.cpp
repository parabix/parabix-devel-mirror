#include "post_process.h"

#include <cassert>
#include <cstdlib>
#include <string>
#include <llvm/Support/Compiler.h>
#include <llvm/Support/ErrorHandling.h>

void postproc_validateObjectsAndArrays(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * /*lineEnd*/, uint64_t lineNum, uint64_t position) {
    printf("%ld %c %ld\n", position, *ptr, lineNum);
}
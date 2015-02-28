#ifndef BUFFER_HPP
#define BUFFER_HPP
#include "bitblock.hpp"

///////////////////////////////////////////////////////////////////////////
//
// Statically Allocated Parallel Data Buffer Management
//
// Parallel data buffers with optional copyback/padding segments.
//
// |COPYBACK BLOCKS|WORKING SEGMENT BLOCKS|PADDING BLOCKS|
//
// WARNING: Statically allocated bitblock arrays must remain in scope.
//
///////////////////////////////////////////////////////////////////////////

#define ALLOC_STATIC_ALIGNED_BYTE_BUFFER(base_ptr, buffer_size) \
    ATTRIBUTE_SIMD_ALIGN BitBlock aligned_##base_ptr[buffer_size/sizeof(BitBlock)]; \
    assert_bitblock_align((void*)aligned_##base_ptr);\
    base_ptr = (char *)&aligned_##base_ptr[0];\

#define ALLOC_STATIC_ALIGNED_BITBLOCK_BUFFER(base_ptr, buffer_size) \
    ATTRIBUTE_SIMD_ALIGN BitBlock aligned_##base_ptr[buffer_size/sizeof(BitBlock)/8]; \
    assert_bitblock_align((void*)aligned_##base_ptr);\
    base_ptr = &aligned_##base_ptr[0];\

// TODO - Unbake macro buffer constants. - BUFFER_SIZE

#define ALLOC_STATIC_ALIGNED_BYTE_BUFFER_WITH_COPYBACK(copyback_ptr, base_ptr) \
    ATTRIBUTE_SIMD_ALIGN BitBlock aligned_##base_ptr[BUFFER_SIZE/sizeof(BitBlock)]; \
    assert_bitblock_align((void*)aligned_##base_ptr);\
    copyback_ptr = (char *)aligned_##base_ptr; \
    memset(copyback_ptr,0,COPYBACK_SIZE); \
    base_ptr = &copyback_ptr[COPYBACK_SIZE]; \

#define ALLOC_STATIC_ALIGNED_BITBLOCK_BUFFER_WITH_COPYBACK(copyback_ptr, base_ptr) \
    ATTRIBUTE_SIMD_ALIGN BitBlock aligned_##base_ptr[BUFFER_SIZE/sizeof(BitBlock)/8]; \
    assert_bitblock_align((void*)aligned_##base_ptr);\
    copyback_ptr = (BitBlock *) aligned_##base_ptr; \
    memset(copyback_ptr,0,COPYBACK_SIZE/8); \
    base_ptr = &copyback_ptr[COPYBACK_BLOCKS]; \

#define COPY_BACK_BYTE_BUFFER(copyback_ptr, base_ptr) \
do {	void * dest = copyback_ptr;\
    void * src = (char *)base_ptr + (SEGMENT_SIZE - COPYBACK_SIZE);\
    assert_bitblock_align(src);\
    assert_bitblock_align(dest);\
    memmove(dest, src, COPYBACK_SIZE);\
} while(0)

#define COPY_BACK_BITBLOCK_BUFFER(copyback_ptr, base_ptr) \
do {	void * dest = copyback_ptr;\
    void * src = (char *)base_ptr + ((SEGMENT_SIZE - COPYBACK_SIZE)/8);\
    assert_bitblock_align(src);\
    assert_bitblock_align(dest);\
    memmove(dest, src, COPYBACK_SIZE/8);\
} while(0)

#endif // BUFFER_HPP

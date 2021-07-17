#ifndef SEARCHABLE_BUFFER_H
#define SEARCHABLE_BUFFER_H

#include <llvm/ADT/StringRef.h>
#include <util/aligned_allocator.h>
#include <vector>

namespace grep {

class SearchableBuffer  {

    enum : unsigned {
        MAX_SIMD_WIDTH_SUPPORTED   = 512U
        , DEFAULT_INITIAL_CAPACITY = 4096U
    };

    template <typename T>
    using BufferAllocator = AlignedAllocator<T, (MAX_SIMD_WIDTH_SUPPORTED / (sizeof(T) * 8))>;

    template <typename T>
    using BufferType = std::vector<T, BufferAllocator<T>>;

public:

    void append(const llvm::StringRef candidate) {
        mInternalBuffer.reserve(candidate.size() + 1);
        mInternalBuffer.insert(mInternalBuffer.end(), candidate.begin(), candidate.end());
        mInternalBuffer.push_back(0);
        mEntries++;
    }

    void reset() {
        mInternalBuffer.clear();
        mEntries = 0;
    }

    size_t getCandidateCount() const {return mEntries; }

    const char * data() const { return mInternalBuffer.data(); }

    size_t size() const { return mInternalBuffer.size(); }

    SearchableBuffer(const unsigned initialCapacity = DEFAULT_INITIAL_CAPACITY)
    : mEntries(0) {
        mInternalBuffer.reserve(initialCapacity);
    }

private:
    size_t           mEntries;
    BufferType<char> mInternalBuffer;
};

}

#endif // SEARCHABLE_BUFFER_H

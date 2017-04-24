/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <cassert>
#include <string>

/**
 * Only works on little-endian architectures for now.
 * Does not support multi-frame files (streams) (optional in the spec).
 * Checksums are always ignored.
 */
class LZ4FrameDecoder {
public:
    LZ4FrameDecoder(const std::string & filename);

    size_t getBlocksStart() const {
        assert(mValid && "Invalid LZ4 frame.");
        return mBlocksStart;
    }

    size_t getBlocksLength() const {
        assert(mValid && "Invalid LZ4 frame.");
        return mBlocksLength;
    }

    bool hasBlockChecksum() const {
        assert(mValid && "Invalid LZ4 frame.");
        return mHasBlockChecksum;
    }

    bool isValid() const {
        return mValid;
    }

private:
    bool mValid = false;
    size_t mFilesize;
    size_t mBlocksStart;
    size_t mBlocksLength;
    size_t mFDLength;
    bool mHasContentChecksum;
    bool mHasBlockChecksum;

    bool decodeFrameDescriptor(std::ifstream & f);
};

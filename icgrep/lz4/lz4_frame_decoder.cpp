/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <iostream>
#include <fstream>

#include <lz4/lz4_frame_decoder.h>

namespace {


// Little-endian.
const uint32_t magicNumber = 0x184D2204;
}

LZ4FrameDecoder::LZ4FrameDecoder() {

}

LZ4FrameDecoder::LZ4FrameDecoder(const std::string & filename) {
    init(filename);
}

void LZ4FrameDecoder::init(const std::string &filename) {
    const size_t minFilesize = getMinFileSize();

    std::ifstream f(filename, std::ios::binary | std::ios::ate);
    if (f.fail()) {
        return;
    }
    mFilesize = f.tellg();
    if (mFilesize < minFilesize) {
        return;
    }

    // Verify magic number (assuming little-endian).
    f.seekg(0);
    uint32_t magic;
    f.read(reinterpret_cast<char*>(&magic), 4);
    if (magic != magicNumber) {
        return;
    }

    // Decode FD to get its length and whether checksum is used.
    if (!decodeFrameDescriptor(f)) {
        return;
    }

    mBlocksStart = 4 + mFDLength;       // MagicNb & FD
    long long blocksEnd = mFilesize - endMarkSize() - (mHasContentChecksum ? contentChecksumSize() : 0);      // EndMark & checksum
    if (blocksEnd > 0 && mBlocksStart <= static_cast<size_t>(blocksEnd)) {
        mBlocksLength = blocksEnd - mBlocksStart;
        mValid = true;
    }
}

bool LZ4FrameDecoder::decodeFrameDescriptor(std::ifstream & f) {
    const size_t minFilesize = getMinFileSize();

    char flag, blockDescriptor, headerChecksum;
    f.get(flag);
    f.get(blockDescriptor);
    mHasContentChecksum = (flag >> 2) & 1;
    bool hasContentSize = (flag >> 3) & 1;
    mHasBlockChecksum = (flag >> 4) & 1;

    // Version number.
    if ((flag >> 6) != 1) {
        return false;
    }

    if (mFilesize < minFilesize +
            (mHasContentChecksum ? contentChecksumSize() : 0) +
            (hasContentSize ? 8 : 0)
       ) {
        return false;
    }

    if (hasContentSize) {
        mFDLength = 1 + 1 + 8 + 1;      //  flag, BD, content size, HC
        // Skip content size.
        f.seekg(8, std::ios::cur);
    } else {
        mFDLength = 3;
    }
    f.get(headerChecksum);
    return true;
}

size_t LZ4FrameDecoder::endMarkSize() const {
    return 4;
}

size_t LZ4FrameDecoder::contentChecksumSize() const {
    return 4;
}


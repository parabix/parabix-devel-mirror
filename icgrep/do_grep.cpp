/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "toolchain.h"
#include "do_grep.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <stdint.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdexcept>
#include <cctype>


#include <llvm/Support/raw_os_ostream.h>

// mmap system
#ifdef USE_BOOST_MMAP
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
using namespace boost::iostreams;
using namespace boost::filesystem;
#else
#include <sys/mman.h>
#endif
#include <fcntl.h>


#define BUFFER_SEGMENTS 15
#define BUFFER_SIZE (BUFFER_SEGMENTS * SEGMENT_SIZE)

//
// Write matched lines from a buffer to an output file, given segment
// scanners for line ends and matches (where matches are a subset of line ends).
// The buffer pointer must point to the first byte of the segment
// corresponding to the scanner indexes.   The first_line_start is the
// start position of the first line relative to the buffer start position.
// It must be zero or negative;  if negative, the buffer must permit negative
// indexing so that the lineup to the buffer start position can also be printed.
// The start position of the final line in the processed segment is returned.
//


bool GrepExecutor::finalLineIsUnterminated() const {
    if (mFileSize == 0) return false;
    unsigned char end_byte = static_cast<unsigned char>(mFileBuffer[mFileSize-1]);
    // LF through CR are line break characters
    if ((end_byte >= 0xA) && (end_byte <= 0xD)) return false;
    // Other line breaks require at least two bytes.
    if (mFileSize == 1) return true;
    // NEL
    unsigned char penult_byte = static_cast<unsigned char>(mFileBuffer[mFileSize-2]);
    if ((end_byte == 0x85) && (penult_byte == 0xC2)) return false;
    if (mFileSize == 2) return true;
    // LS and PS
    if ((end_byte < 0xA8) || (end_byte > 0xA9)) return true;
    return (static_cast<unsigned char>(mFileBuffer[mFileSize-3]) != 0xE2) || (penult_byte != 0x80);
}

void GrepExecutor::doGrep(const std::string & fileName) {

    mFileName = fileName;

#ifdef USE_BOOST_MMAP
    const path file(mFileName);
    if (exists(file)) {
        if (is_directory(file)) {
            return;
        }
    } else {
        std::cerr << "Error: cannot open " << mFileName << " for processing. Skipped.\n";
        return;
    }

    mFileSize = file_size(file);
    mapped_file mFile;
    if (mFileSize == 0) {
        mFileBuffer = nullptr;
    }
    else {
        try {
            mFile.open(mFileName, mapped_file::priv, mFileSize, 0);
        } catch (std::ios_base::failure e) {
            std::cerr << "Error: Boost mmap of " << mFileName << ": " << e.what() << std::endl;
            return;
        }
        mFileBuffer = mFile.data();
    }
#else
    struct stat infile_sb;
    const int fdSrc = open(mFileName.c_str(), O_RDONLY);
    if (fdSrc == -1) {
        std::cerr << "Error: cannot open " << mFileName << " for processing. Skipped.\n";
        return;
    }
    if (fstat(fdSrc, &infile_sb) == -1) {
        std::cerr << "Error: cannot stat " << mFileName << " for processing. Skipped.\n";
        close (fdSrc);
        return;
    }
    if (S_ISDIR(infile_sb.st_mode)) {
        close (fdSrc);
        return;
    }
    mFileSize = infile_sb.st_size;
    if (mFileSize == 0) {
        mFileBuffer = nullptr;
    }
    else {
        mFileBuffer = (char *) mmap(NULL, mFileSize, PROT_READ, MAP_PRIVATE, fdSrc, 0);
        if (mFileBuffer == MAP_FAILED) {
            if (errno ==  ENOMEM) {
                std::cerr << "Error:  mmap of " << mFileName << " failed: out of memory\n";
                close (fdSrc);
            }
            else {
                std::cerr << "Error: mmap of " << mFileName << " failed with errno " << errno << ". Skipped.\n";
                close (fdSrc);
            }
            return;
        }
    }
#endif

    llvm::raw_os_ostream out(std::cout);

    uint64_t finalLineUnterminated = 0;
    if(finalLineIsUnterminated())
        finalLineUnterminated = 1;

    mMainFcn(mFileBuffer, mFileSize, fileName.c_str(), finalLineUnterminated);

    PrintTotalCount();
    
#ifdef USE_BOOST_MMAP
    mFile.close();
#else
    munmap((void *)mFileBuffer, mFileSize);
    close(fdSrc);
#endif   
}

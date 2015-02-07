/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "icgrep.h"
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

#include "include/simd-lib/carryQ.hpp"
#include "include/simd-lib/pabloSupport.hpp"
#include "include/simd-lib/s2p.hpp"
#include "include/simd-lib/buffer.hpp"

// mmap system
#include <sys/mman.h>
#include <fcntl.h>


#define BUFFER_SEGMENTS 15
#define BUFFER_SIZE (BUFFER_SEGMENTS * SEGMENT_SIZE)

#define BitBlock_declare(name)  BitBlock name

#define ubitblock_declare(name, n) \
  ubitblock name[n];\
  do {int i;\
      for (i = 0; i < n; i++) name[i]._128 = simd<1>::constant<0>();\
     }\
  while (0)

BitBlock EOF_mask = simd<1>::constant<1>();

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

ssize_t GrepExecutor::write_matches(char * buffer, ssize_t first_line_start) {

  ssize_t line_start = first_line_start;
  size_t match_pos;
  size_t line_end;
  while (match_scanner.has_next()) {
    match_pos = match_scanner.scan_to_next();
    // If we found a match, it must be at a line end.
    line_end = LF_scanner.scan_to_next();
    while (line_end < match_pos) {
      line_start = line_end + 1;
      line_no++;
      line_end = LF_scanner.scan_to_next();
    }
    if (mShowFileNameOption) {
      std::cout << mFileName;
    }
    if (mShowLineNumberingOption) {
      std::cout << line_no << ":";
    }
    if ((buffer[line_start] == 0xA) && (line_start != line_end)) {
        // The LF of a CRLF.  Really the end of the last line.  
        line_start++;
    }
    unsigned char end_byte = (unsigned char) buffer[line_end];
    if (mNormalizeLineBreaksOption) {
        if (end_byte <= 0xD) {
            // Line terminated with LF, VT, FF or CR.  
            std::cout.write(&buffer[line_start], line_end - line_start);
            std::cout << std::endl;
        }
        else if (end_byte == 0x85) {
            // Line terminated with NEL, on the second byte.  
            std::cout.write(&buffer[line_start], line_end - line_start - 1);
            std::cout << std::endl;
        }
        else  {
            // Line terminated with PS or LS, on the third byte.
            std::cout.write(&buffer[line_start], line_end - line_start - 2);
            std::cout << std::endl;
        }
    }
    else {
        // Check for line_end on first byte of CRLF;  note that to safely
        // access past line_end, even at the end of buffer, we require the
        // mmap_sentinel_bytes >= 1.
        if (end_byte == 0x0D) {
            if (buffer[line_end + 1] == 0x0A) {
                line_end++;
            }
        }
        std::cout.write(&buffer[line_start], line_end - line_start + 1);
    }
    line_start = line_end + 1;
    line_no++;

  }
  while(LF_scanner.has_next()) {
    line_end = LF_scanner.scan_to_next();
    line_start = line_end+1;
    line_no++;
  }
  return line_start;
}



void GrepExecutor::doGrep(const std::string infilename) {

    struct Basis_bits basis_bits;
    struct Output output;
    BitBlock match_vector;
    BitBlock carry_q[mCarries];
    BitBlock advance_q[mAdvances];
    
    
    mFileName = infilename + ":";
    
    size_t match_count = 0;
    size_t blk = 0;
    size_t block_base  = 0;
    size_t block_pos   = 0;
    size_t chars_avail = 0;
    ssize_t line_start = 0;
    line_no = 1;

    match_vector = simd<1>::constant<0>();
    memset (carry_q, 0, sizeof(BitBlock) * mCarries);
    memset (advance_q, 0, sizeof(BitBlock) * mAdvances);
    
    int fdSrc;
    struct stat infile_sb;
    char * infile_buffer;
    fdSrc = open(infilename.c_str(), O_RDONLY);
    if (fdSrc == -1) {
        std::cerr << "Error: cannot open " << infilename << " for processing. Skipped.\n";
        return;
    }
    if (fstat(fdSrc, &infile_sb) == -1) {
        std::cerr << "Error: cannot stat " << infilename << " for processing. Skipped.\n";
        return;
    }
    if (S_ISDIR(infile_sb.st_mode)) {
        // Silently ignore directories.
        // std::cerr << "Error: " << infilename << " is a directory. Skipped.\n";
        return;
    }
    mFileSize = infile_sb.st_size;
    // Set 2 sentinel bytes, 1 for possible addition of LF for unterminated last line, 
    // 1 guard byte.
    const size_t mmap_sentinel_bytes = 2;  
    infile_buffer = (char *) mmap(NULL, mFileSize + mmap_sentinel_bytes, PROT_READ, MAP_PRIVATE, fdSrc, 0);
    if (infile_buffer == MAP_FAILED) {
        std::cerr << "Error: mmap of " << infilename << " failed. Skipped.\n";
        return;
    }
    char * buffer_ptr;
    size_t segment = 0;
    size_t segment_base = 0;
    chars_avail = mFileSize;
    
//////////////////////////////////////////////////////////////////////////////////////////
// Full Segments
//////////////////////////////////////////////////////////////////////////////////////////

    while (chars_avail >= SEGMENT_SIZE) {

        segment_base = segment * SEGMENT_SIZE;
        LF_scanner.init();
        match_scanner.init();

        for (blk = 0; blk < SEGMENT_BLOCKS; blk++) {
            block_base = blk*BLOCK_SIZE + segment_base;
            s2p_do_block((BytePack *) &infile_buffer[block_base], basis_bits);
            mProcessBlockFcn(basis_bits, carry_q, advance_q, output);

            LF_scanner.load_block(output.LF, blk);
            match_scanner.load_block(output.matches, blk);
            if (mCountOnlyOption){
                if (bitblock::any(output.matches))
                {
                    if (bitblock::any(simd_and(match_vector, output.matches))){
                        match_count += bitblock::popcount(match_vector);
                        match_vector = output.matches;
                    }
                    else
                    {
                        match_vector = simd_or(match_vector, output.matches);
                    }
                }
            }
        }

        buffer_ptr = &infile_buffer[segment_base];

        if (!mCountOnlyOption) {
          line_start = write_matches(buffer_ptr, line_start);
        }
        segment++;
        line_start -= SEGMENT_SIZE;  /* Will be negative offset for use within next segment. */
        chars_avail -= SEGMENT_SIZE;
    }

//////////////////////////////////////////////////////////////////////////////////////////
// For the Final Partial Segment.
//////////////////////////////////////////////////////////////////////////////////////////

    segment_base = segment * SEGMENT_SIZE;
    int remaining = chars_avail;
        

    LF_scanner.init();
    match_scanner.init();

    /* Full Blocks */
    blk = 0;
    while (remaining >= BLOCK_SIZE) {
        block_base = block_pos + segment_base;
        s2p_do_block((BytePack *) &infile_buffer[block_base], basis_bits);
        mProcessBlockFcn(basis_bits, carry_q, advance_q, output);

        LF_scanner.load_block(output.LF, blk);
        match_scanner.load_block(output.matches, blk);
        if (mCountOnlyOption)
        {
            if (bitblock::any(output.matches))
            {
                if (bitblock::any(simd_and(match_vector, output.matches)))
                {
                    match_count += bitblock::popcount(match_vector);
                    match_vector = output.matches;
                }
                else
                {
                    match_vector = simd_or(match_vector, output.matches);
                }
            }
        }

        block_pos += BLOCK_SIZE;
        remaining -= BLOCK_SIZE;
        blk++;
    }
    block_base = block_pos;
    //fprintf(stderr, "Remaining = %i\n", remaining);

    //For the last partial block, or for any carry.
    
    
    EOF_mask = bitblock::srl(simd<1>::constant<1>(), convert(BLOCK_SIZE-remaining));
    block_base = block_pos + segment_base;
    s2p_do_final_block((BytePack *) &infile_buffer[block_base], basis_bits, EOF_mask);
    mProcessBlockFcn(basis_bits, carry_q, advance_q, output);

    if (mCountOnlyOption)
    {
        match_count += bitblock::popcount(match_vector);
        if (bitblock::any(output.matches))
        {
            match_count += bitblock::popcount(output.matches);
        }
        if (mShowFileNameOption) {
            std::cout << mFileName;
        }
        std::cout << match_count << std::endl;
    }
    else
    {
        LF_scanner.load_block(output.LF, blk);
        match_scanner.load_block(output.matches, blk);
        blk++;
        for (int i = blk; i < SEGMENT_BLOCKS; i++)
        {
            LF_scanner.load_block(simd<1>::constant<0>(), i);
            match_scanner.load_block(simd<1>::constant<0>(), i);
        }
        buffer_ptr = &infile_buffer[segment_base];
        line_start = write_matches(buffer_ptr, line_start);
    }
    
    munmap((void *) infile_buffer, mFileSize + mmap_sentinel_bytes);
    close(fdSrc);
    
}

/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "icgrep.h"
#include "utf_encoding.h"
#include "compiler.h"

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

#include "include/simd-lib/bitblock.hpp"
#include "include/simd-lib/carryQ.hpp"
#include "include/simd-lib/pabloSupport.hpp"
#include "include/simd-lib/s2p.hpp"
#include "include/simd-lib/buffer.hpp"
#include "include/simd-lib/bitblock_iterator.hpp"
#include "include/simd-lib/transpose.hpp"

#include "hrtime.h"

// mmap system
#include <sys/mman.h>
#include <fcntl.h>

#if (BLOCK_SIZE == 128)
#define SEGMENT_BLOCKS 7
#endif

#if (BLOCK_SIZE == 256)
#define SEGMENT_BLOCKS 15
#endif

#define SEGMENT_SIZE (BLOCK_SIZE * SEGMENT_BLOCKS)

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

struct Output {
    BitBlock matches;
    BitBlock LF;
};

using namespace std;

typedef void (*process_block_fcn)(const Basis_bits &basis_bits, BitBlock carry_q[], BitBlock advance_q[], Output &output);


void do_process(char * infile_buffer, size_t infile_size, FILE *outfile, int count_only_option, int carry_count, int advance_count, process_block_fcn process_block);


BitBlock get_category(Basis_bits &basis_bits, const char* category);

int main(int argc, char *argv[])
{
    char * inregex, * fileregex, * infilename, * outfilename;
    FILE *infile, *outfile, *regexfile;

    int fdSrc;
    struct stat infile_sb;
    char * infile_buffer;

    int opt_code;
    bool count_only_option = 0;
    bool print_version_option = 0;
    bool regex_from_file_option = 0;
    bool ascii_only_option = 0;
    bool compile_time_option = 0;
    bool enable_multiplexing = 0;
    bool print_usage = 0;

    unsigned long long cycles = 0;


    long lSize = 0;

    size_t result;

    while ((opt_code = getopt(argc, argv, "cvftam")) != -1)
    {
        switch (opt_code)
        {
        case 'c':
            count_only_option = 1;
            break;
        case 'v':
            print_version_option = 1;
            break;
        case 'f':
            regex_from_file_option = 1;
            break;
        case 't':
            compile_time_option = 1;
            break;
        case 'a':
            ascii_only_option = 1;
            break;
        case 'm':
            enable_multiplexing = 1;
            break;
        case '?':
            break;
        default:
            printf ("Invalid option: %c\n", opt_code);
            print_usage = 1;
        }
    }

    if (optind >= argc)
    {
        printf ("Too few arguments\n");
        print_usage = 1;
    }

    if (print_usage) {
        printf("Usage: %s [-a] [-c] [-f] [-m] [-t] [-v] <regex|regexfile> <inputfile> [<outputfile>]\n", argv[0]);
        exit(-1);
    }

    inregex = argv[optind++];
    if (inregex == 0)
    {
        fprintf(stderr, "Error: cannot read the regular expression.\n");
        exit(-1);
    }

    if (regex_from_file_option)
    {
        regexfile = fopen(inregex, "rb");
        if (!regexfile){
            fprintf(stderr, "Error: cannot open %s for processing.\n", inregex);
            exit(-1);
        }

        fseek (regexfile , 0 , SEEK_END);
        lSize = ftell (regexfile);
        rewind (regexfile);

        fileregex = (char*) malloc (sizeof(char)*lSize);
        if (fileregex == NULL) {fputs ("Memory error",stderr); exit (2);}

        result = fread (fileregex, 1, lSize, regexfile);
        if (result != lSize) {fputs ("Reading error",stderr); exit (3);}
        fclose(regexfile);

        if (fileregex[lSize - 1] == '\n') fileregex[lSize - 1] = '\0';
    }

    infilename = argv[optind++];

    if (optind >= argc) {
        outfile = stdout;
    }
    else {
        outfilename = argv[optind++];
        if (optind != argc)
        {
            printf("Too many arguments\n");
            printf("Usage: %s [-a] [-c] [-f] [-m] [-t] [-v] <regex|regexfile> <inputfile> [<outputfile>]\n", argv[0]);
            exit(-1);
        }
        outfile = fopen(outfilename, "wb");
        if (!outfile)
        {
            fprintf(stderr, "Error: cannot open %s for writing.\n", outfilename);
            exit(-1);
        }
    }

    fdSrc = open(infilename, O_RDONLY);
    if (fdSrc == -1) {
        fprintf(stderr, "Error: cannot open %s for processing.\n", infilename);
        exit(-1);
    }
    if (fstat(fdSrc, &infile_sb) == -1) {
        fprintf(stderr, "Error: cannot stat %s for processing.\n", infilename);
        exit(-1);
    }
    if (infile_sb.st_size == 0) {
        if (count_only_option) fprintf(outfile, "Matching Lines: %d\n", 0);
        exit(0);
    }
    infile_buffer = (char *) mmap(NULL, infile_sb.st_size, PROT_READ, MAP_PRIVATE, fdSrc, 0);
    if (infile_buffer == MAP_FAILED) {
        fprintf(stderr, "Error: mmap of %s failure.\n", infilename);
        exit(-1);
    }

    if (print_version_option)
    {
        fprintf(outfile, "Parabix icgrep implementation: August 2014\n");
    }

    Encoding encoding(ascii_only_option ? Encoding::Type::ASCII : Encoding::Type::UTF_8, 8);
    if (compile_time_option)
    {
        cycles = get_hrcycles();
    }
    const auto llvm_codegen = icgrep::compile(encoding, (regex_from_file_option ? fileregex : inregex), compile_time_option, enable_multiplexing);

    if (compile_time_option)
    {
        cycles = get_hrcycles() - cycles;
        std::cout << "Total compile time - cycles:       " << cycles << std::endl;
    }

    if (llvm_codegen.process_block_fptr != 0)
    {
        void (*FP)(const Basis_bits &basis_bits, BitBlock carry_q[], BitBlock advance_q[], Output &output) = (void (*)(const Basis_bits &basis_bits, BitBlock carry_q[], BitBlock advance_q[], Output &output))(void*)llvm_codegen.process_block_fptr;
        do_process(infile_buffer, infile_sb.st_size, outfile, count_only_option, llvm_codegen.carry_q_size, llvm_codegen.advance_q_size, FP);
    }

    munmap((void *) infile_buffer, infile_sb.st_size);
    close(fdSrc);
    fclose(outfile);
    if (regex_from_file_option) free(fileregex);

    return 0;
}

#if (BLOCK_SIZE == 256)
typedef BitStreamScanner<BitBlock, uint64_t, uint64_t, SEGMENT_BLOCKS> ScannerT;
#endif

#if (BLOCK_SIZE == 128)
typedef BitStreamScanner<BitBlock, uint32_t, uint32_t, SEGMENT_BLOCKS> ScannerT;
#endif

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

ssize_t write_matches(FILE * outfile, ScannerT line_scanner, ScannerT match_scanner, char * buffer, ssize_t first_line_start) {

  ssize_t line_start = first_line_start;
  size_t match_pos;
  size_t line_end;
  while (match_scanner.has_next()) {
    match_pos = match_scanner.scan_to_next();
    // If we found a match, it must be at a line end.
    line_end = line_scanner.scan_to_next();
    while (line_end < match_pos) {
      line_start = line_end + 1;
      line_end = line_scanner.scan_to_next();
    }
    fwrite(&buffer[line_start], 1, line_end - line_start + 1, outfile);
    line_start = line_end + 1;

  }
  while(line_scanner.has_next()) {
    line_end = line_scanner.scan_to_next();
    line_start = line_end+1;
  }
  return line_start;
}



void do_process(char * infile_buffer, size_t infile_size, FILE *outfile, int count_only_option, int carry_count, int advance_count, process_block_fcn process_block) {

    struct Basis_bits basis_bits;
    struct Output output;
    BitBlock match_vector;
    BitBlock carry_q[carry_count];
    BitBlock advance_q[advance_count];
    int match_count=0;
    int blk = 0;
    int block_base  = 0;
    int block_pos   = 0;
    int buffer_pos  = 0;
    int chars_avail = 0;
    int chars_read  = 0;
    int line_start = 0;
    int line_end = 0;
    int match_pos = 0;
    int line_no = 0;

    ScannerT LF_scanner;
    ScannerT match_scanner;

    match_vector = simd<1>::constant<0>();
    memset (carry_q, 0, sizeof(BitBlock) * carry_count);
    memset (advance_q, 0, sizeof(BitBlock) * advance_count);

    char * buffer_ptr;
    int segment = 0;
    int segment_base = 0;
    chars_avail = infile_size;

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
            process_block(basis_bits, carry_q, advance_q, output);

            LF_scanner.load_block(output.LF, blk);
            match_scanner.load_block(output.matches, blk);
            if (count_only_option){
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

        if (!count_only_option) {
          line_start = write_matches(outfile, LF_scanner, match_scanner, buffer_ptr, line_start);
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
    //fprintf(outfile, "Remaining = %i\n", remaining);
        block_base = block_pos + segment_base;
        s2p_do_block((BytePack *) &infile_buffer[block_base], basis_bits);
        process_block(basis_bits, carry_q, advance_q, output);

        LF_scanner.load_block(output.LF, blk);
        match_scanner.load_block(output.matches, blk);
        if (count_only_option)
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
    process_block(basis_bits, carry_q, advance_q, output);

    if (count_only_option)
    {
        match_count += bitblock::popcount(match_vector);
        if (bitblock::any(output.matches))
        {
            match_count += bitblock::popcount(output.matches);
        }
        fprintf(outfile, "Matching Lines:%d\n", match_count);
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
        line_start = write_matches(outfile, LF_scanner, match_scanner, buffer_ptr, line_start);
    }

    buffer_pos += chars_avail;
}




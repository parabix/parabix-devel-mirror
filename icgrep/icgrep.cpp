/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "icgrep.h"

#include "utf_encoding.h"
#include "re_compiler.h"

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

#include <simd-lib/bitblock.hpp>
#include <simd-lib/carryQ.hpp>
#include <simd-lib/pabloSupport.hpp>
#include <simd-lib/s2p.hpp>
#include <simd-lib/buffer.hpp>
#include <simd-lib/bitblock_iterator.hpp>

// mmap system
#include <sys/mman.h>
#include <fcntl.h>


#define SEGMENT_BLOCKS 15
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

#include <simd-lib/transpose.hpp>

using namespace std;

typedef void (*process_block_fcn)(const Basis_bits &basis_bits, BitBlock carry_q[], Output &output);


#define USE_MMAP
#ifndef USE_MMAP
void do_process(FILE *infile, FILE *outfile, int count_only_option, int carry_count, process_block_fcn process_block);
#endif
#ifdef USE_MMAP
void do_process(char * infile_buffer, size_t infile_size, FILE *outfile, int count_only_option, int carry_count, process_block_fcn process_block);
#endif

BitBlock get_category(Basis_bits &basis_bits, const char* category);

int main(int argc, char *argv[])
{
    char * inregex, * fileregex, * infilename, * outfilename;
    FILE *infile, *outfile, *regexfile;

#ifdef USE_MMAP
    int fdSrc;
    struct stat infile_sb;
    char * infile_buffer;
#endif
    
    int opt_code;
    int count_only_option = 0;
    int print_version_option = 0;
    int regex_from_file_option = 0;
    int ascii_only_option = 0;

    int compile_time_option = 0;

    unsigned long long cycles = 0;
    double timer = 0;

    long lSize = 0;

    size_t result;

    while ((opt_code = getopt(argc, argv, "cvfta")) != -1)
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
        case '?':
            break;
        default:
            printf ("Invalid option: %c\n", opt_code);
            printf("Usage: %s [-c] [-v] [-f] [-t] [-a] <regex|regexfile> <inputfile> [<outputfile>]\n", argv[0]);
                    exit(-1);
        }
    }

    if (optind >= argc)
    {
        printf ("Too few arguments\n");
        printf("Usage: %s [-c] [-v] [-f] [-t] [-a] <regex|regexfile> <inputfile> [<outputfile>]\n", argv[0]);
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
#ifndef USE_MMAP
    infile = fopen(infilename, "rb");
    if (!infile) {
        fprintf(stderr, "Error: cannot open %s for processing.\n", infilename);
        exit(-1);
    }
#endif
#ifdef USE_MMAP
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
        if (count_only_option) fprintf(outfile, "Matching Lines%d\n", 0);
        exit(0);
    }
    infile_buffer = (char *) mmap(NULL, infile_sb.st_size, PROT_READ, MAP_PRIVATE, fdSrc, 0);
    if (infile_buffer == MAP_FAILED) {
        fprintf(stderr, "Error: mmap of %s failure.\n", infilename);
        exit(-1);
    }
#endif

    if (optind >= argc) outfile = stdout;
    else
    {
        outfilename = argv[optind++];
        if (optind != argc)
        {
            printf ("Too many arguments\n");
            printf("Usage: %s [-c] [-v] [-f] [-t] [-a] <regex|regexfile> <inputfile> [<outputfile>]\n", argv[0]);
            exit(-1);
        }
        outfile = fopen(outfilename, "wb");
        if (!outfile)
        {
            fprintf(stderr, "Error: cannot open %s for writing.\n", outfilename);
            exit(-1);
        }
    }

    if (print_version_option)
    {
        fprintf(outfile, "Parabix icgrep implementation: August 2014\n");
    }

    UTF_Encoding encoding;
    encoding.setName("UTF-8");
    encoding.setBits(8);
    encoding.setMask(0xFF);

    RE_Compiler* re_compiler = new RE_Compiler();
    if (compile_time_option)
    {
        cycles = get_hrcycles();
        timer = getElapsedTime();
    }
    LLVM_Gen_RetVal llvm_codegen = re_compiler->compile(compile_time_option,
                                                        ascii_only_option,
                                                        "basis_bits.bit_",
                                                        "temp",
                                                        encoding ,
                                                        (regex_from_file_option ? fileregex : inregex));

    if (compile_time_option)
    {
        cycles = get_hrcycles() - cycles;
        timer = getElapsedTime() - timer;
        std::cout << "Total compile time - cycles:       " << cycles << std::endl;
        std::cout << "Total compile time - milliseconds: " << timer << std::endl;
    }

    if (llvm_codegen.process_block_fptr != 0)
    {
        void (*FP)(const Basis_bits &basis_bits, BitBlock carry_q[], Output &output) = (void (*)(const Basis_bits &basis_bits, BitBlock carry_q[], Output &output))(void*)llvm_codegen.process_block_fptr;
#ifndef USE_MMAP
        do_process(infile, outfile, count_only_option, llvm_codegen.carry_q_size, FP);
#endif
#ifdef USE_MMAP
        do_process(infile_buffer, infile_sb.st_size, outfile, count_only_option, llvm_codegen.carry_q_size, FP);
#endif
    }

    delete re_compiler;
#ifndef USE_MMAP
    fclose(infile);
#endif
#ifdef USE_MMAP
    munmap((void *) infile_buffer, infile_sb.st_size);
    close(fdSrc);
#endif
    fclose(outfile);
    if (regex_from_file_option) free(fileregex);

    return 0;
}

#ifndef USE_MMAP
void do_process(FILE *infile, FILE *outfile, int count_only_option, int carry_count, process_block_fcn process_block) {
#endif
#ifdef USE_MMAP
void do_process(char * infile_buffer, size_t infile_size, FILE *outfile, int count_only_option, int carry_count, process_block_fcn process_block) {
#endif

    struct Basis_bits basis_bits;
    struct Output output;

    BitBlock carry_q[carry_count];
    memset (carry_q, 0, sizeof(BitBlock) * carry_count);

    BitBlock match_vector = simd<1>::constant<0>();
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

    BitStreamScanner<BitBlock, uint64_t, uint64_t, SEGMENT_BLOCKS> LF_scanner;
    BitStreamScanner<BitBlock, uint64_t, uint64_t, SEGMENT_BLOCKS> match_scanner;
        
    
#ifndef USE_MMAP
    ATTRIBUTE_SIMD_ALIGN char src_buffer[SEGMENT_SIZE];
    
    chars_read = fread((void *)&src_buffer[0], 1, SEGMENT_SIZE, infile);
    chars_avail = chars_read;
    if (chars_avail >= SEGMENT_SIZE) chars_avail = SEGMENT_SIZE;
#endif
#ifdef USE_MMAP    
    int segment = 0;
    int segment_base = 0;
    chars_avail = infile_size;
    
#endif
//////////////////////////////////////////////////////////////////////////////////////////
// Full Segments
//////////////////////////////////////////////////////////////////////////////////////////

    
    
    while (chars_avail >= SEGMENT_SIZE) {

#ifdef USE_MMAP
	segment_base = segment * SEGMENT_SIZE;
#endif
        LF_scanner.init();
        match_scanner.init();

        for (blk = 0; blk < SEGMENT_BLOCKS; blk++) {
#ifndef USE_MMAP
            block_base = blk*BLOCK_SIZE;
            s2p_do_block((BytePack *) &src_buffer[block_base], basis_bits);
#endif
#ifdef USE_MMAP
            block_base = blk*BLOCK_SIZE + segment_base;
            s2p_do_block((BytePack *) &infile_buffer[block_base], basis_bits);    
#endif
            process_block(basis_bits, carry_q, output);

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

#ifndef USE_MMAP
        int copy_back_pos = 0;


        if (LF_scanner.count() > 0) {
            copy_back_pos = LF_scanner.get_final_pos() + 1;
            memset (carry_q, 0, sizeof(BitBlock) * carry_count);
        }
        else {
            copy_back_pos =  SEGMENT_SIZE;
        }

        int  copy_back_size = SEGMENT_SIZE - copy_back_pos;
#endif
#ifdef USE_MMAP
    
#endif

        if (!count_only_option) {
#ifndef USE_MMAP
            line_start = 0;
#endif

            while (match_scanner.has_next()) {
                match_pos = match_scanner.scan_to_next();
                line_end = LF_scanner.scan_to_next();
                while (line_end < match_pos) {
                    line_start = line_end+1;
                    line_no++;
                    line_end = LF_scanner.scan_to_next();
                }
#ifndef USE_MMAP
                fwrite(&src_buffer[line_start], 1, line_end - line_start + 1, outfile);
#endif
#ifdef USE_MMAP
                fwrite(&infile_buffer[segment_base + line_start], 1, line_end - line_start + 1, outfile);
  
#endif

                line_start = line_end+1;
                line_no++;
            }
            while (LF_scanner.has_next()) {
                line_end = LF_scanner.scan_to_next();
                line_start = line_end+1;
                line_no++;
            }

        }
#ifndef USE_MMAP
        memmove(&src_buffer[0], &src_buffer[copy_back_pos], copy_back_size);

	//Do another read.
        chars_read = fread(&src_buffer[copy_back_size], 1, copy_back_pos, infile);
        chars_avail = chars_read + copy_back_size;
        if (chars_avail >= SEGMENT_SIZE) chars_avail = SEGMENT_SIZE;
        buffer_pos += chars_avail;
#endif
#ifdef USE_MMAP
	segment++;
	line_start -= SEGMENT_SIZE;  /* Will be negative offset for use within next segment. */
	chars_avail -= SEGMENT_SIZE;
    
#endif
    }


//////////////////////////////////////////////////////////////////////////////////////////
// For the Final Partial Segment.
//////////////////////////////////////////////////////////////////////////////////////////

#ifdef USE_MMAP
    segment_base = segment * SEGMENT_SIZE;
#endif
    int remaining = chars_avail;

    LF_scanner.init();
    match_scanner.init();

    /* Full Blocks */
    blk = 0;
    while (remaining >= BLOCK_SIZE) {
    //fprintf(outfile, "Remaining = %i\n", remaining);
#ifndef USE_MMAP
        block_base = block_pos;
        s2p_do_block((BytePack *) &src_buffer[block_base], basis_bits);
#endif
#ifdef USE_MMAP
        block_base = block_pos + segment_base;
        s2p_do_block((BytePack *) &infile_buffer[block_base], basis_bits);    
#endif
        process_block(basis_bits, carry_q, output);

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
#ifndef USE_MMAP
     block_base = block_pos;
     s2p_do_final_block((BytePack *) &src_buffer[block_base], basis_bits, EOF_mask);
#endif
#ifdef USE_MMAP
     block_base = block_pos + segment_base;
     s2p_do_final_block((BytePack *) &infile_buffer[block_base], basis_bits, EOF_mask);    
#endif
    process_block(basis_bits, carry_q, output);

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
#ifndef USE_MMAP
        line_start = 0;
#endif
        while (match_scanner.has_next())
        {
            match_pos = match_scanner.scan_to_next();
            line_end = LF_scanner.scan_to_next();
            while(line_end < match_pos)
            {
                line_start = line_end + 1;
                line_no++;
                line_end = LF_scanner.scan_to_next();
            }
#ifndef USE_MMAP
            fwrite(&src_buffer[line_start], 1, line_end - line_start + 1, outfile);
#endif
#ifdef USE_MMAP
            fwrite(&infile_buffer[segment_base + line_start], 1, line_end - line_start + 1, outfile);
  
#endif
            line_start = line_end + 1;
            line_no++;
        }
        while(LF_scanner.has_next())
        {
            line_end = LF_scanner.scan_to_next();
            line_start = line_end+1;
            line_no++;
        }
    }

    buffer_pos += chars_avail;
}




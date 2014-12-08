/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "icgrep.h"
#include "utf_encoding.h"
#include "compiler.h"
#include "pablo/pablo_compiler.h"
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


    if (print_version_option)
    {
        fprintf(outfile, "Parabix icgrep implementation: August 2014\n");
    }

    Encoding encoding(ascii_only_option ? Encoding::Type::ASCII : Encoding::Type::UTF_8, 8);
    const auto llvm_codegen = icgrep::compile(encoding, (regex_from_file_option ? fileregex : inregex), false, enable_multiplexing);

    if (llvm_codegen.process_block_fptr != 0)
      
    {
        void (*FP)(const Basis_bits &basis_bits, BitBlock carry_q[], BitBlock advance_q[], Output &output) = 
           (void (*)(const Basis_bits &basis_bits, BitBlock carry_q[], BitBlock advance_q[], Output &output))(void*)llvm_codegen.process_block_fptr;
        GrepExecutor grepEngine = GrepExecutor(llvm_codegen.carry_q_size, llvm_codegen.advance_q_size, FP);
	grepEngine.setCountOnlyOption(count_only_option);
	grepEngine.doGrep(infilename);
    }

    if (regex_from_file_option) free(fileregex);

    return 0;
}


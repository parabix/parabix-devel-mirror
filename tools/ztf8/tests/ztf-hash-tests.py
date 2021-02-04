#!/usr/bin/env python3

import sys
import os
import subprocess
import optparse
import codecs
import binascii
import uniseg.wordbreak
import compress_words
import decompress_words
import edelta_compression
import edelta_decompression

# --------------------------------------------------------------------

CmpAlgorithmList = [
    compress_words.Compressor(),
    edelta_compression.Compressor(),
]

DecmpAlgorithmList = [
    decompress_words.Decompressor(),
    edelta_decompression.Decompressor(),
]

# --------------------------------------------------------------------

if __name__ == '__main__':
    StubDirName = 'stubs'
    option_parser = optparse.OptionParser(
        usage='python %prog [options] <grep_executable>', version='1.0')
    option_parser.add_option('-c', '--compress',
                             dest='compress', type='string')
    option_parser.add_option('-d', '--decompress',
                             dest='decompress', type='string')
    options, args = option_parser.parse_args(sys.argv[1:])

    # Create a file to hold the generated encoded data.
    outputFilename = 'output'

    if options.decompress:
        fileName = options.decompress
        with open(fileName, 'rb') as infile:
            text = infile.read()
    else:
        fileName = options.compress
        with open(fileName, 'rt') as infile:
            text = infile.read()

    if options.decompress:
        for algorithm in DecmpAlgorithmList:
            decompressedData = algorithm.Decompress(text)
            print(decompressedData.decode('utf-8'))
    else:
        # segment the input as per unicode text segmentation rules
        word_list = []
        boundary_positions = []
        for word in uniseg.wordbreak.words(text):
            word_list.append(word)
        # a vector representing the positions of word boundaries - future use?
        #for index in uniseg.wordbreak.word_boundaries(text):
        #    boundary_positions.append(index)
        for algorithm in CmpAlgorithmList:
            compressedData = algorithm.Compress(word_list)
        # write compressed data to output.z file
        output_file = open(outputFilename+"."+algorithm.name+".z", "wb")
        output_file.write(compressedData)
        output_file.close()
        #print(compressedData, 'compressedData')
        #print(compressedData.decode('utf-8', 'ignore'))

#!/usr/bin/env python3

import sys
import os
import subprocess
import optparse
import codecs
import binascii
import uniseg.wordbreak
import unicodedata
import nltk
import compress_words
import decompress_words
import edelta_compression
import edelta_decompression
import length_group_phrase_cpm
# --------------------------------------------------------------------

CmpAlgorithmList = [
    # compress_words.Compressor(),
    length_group_phrase_cpm.LGPhraseCompressor(),
    # edelta_compression.Compressor(),
]

DecmpAlgorithmList = [
    decompress_words.Decompressor(),
    # edelta_decompression.Decompressor(),
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
    option_parser.add_option('-p', '--print-plaintext',
                             dest='plaintext', type='string')
    option_parser.add_option('-w', '--print-word-codeword',
                             dest='wordCodeword', type='string')
    options, args = option_parser.parse_args(sys.argv[1:])

    # Create a file to hold the generated encoded data.
    outputFilename = 'output'
    decompressFileName = 'original'
    printPlainText = False
    printWordCodeword = False

    if options.decompress:
        fileName = options.decompress
        with open(fileName, 'rb') as infile:
            text = infile.read()
    else:
        if options.plaintext:
            printPlainText = True
        if options.wordCodeword:
            printWordCodeword = True
        fileName = options.compress

    if options.decompress:
        for algorithm in DecmpAlgorithmList:
            decompressedData = algorithm.Decompress(text)
            print(decompressedData)
        output_file = open(decompressFileName, "wb")
        #decompressedData = decompressedData.decode('utf-8')
        output_file.write(decompressedData)
        output_file.close()
    else:
        # segment the input as per unicode text segmentation rules
        word_list = []
        boundary_positions = []
        # word_list = nltk.word_tokenize(text)
        # print(word_list, 'word_list')
        # a vector representing the positions of word boundaries - future use?
        # for index in uniseg.wordbreak.word_boundaries(text):
        #    boundary_positions.append(index)
        for algorithm in CmpAlgorithmList:
            # compressedData = algorithm.CompressPhrase(
            #    fileName, 4, printPlainText, printWordCodeword)
            compressedData = algorithm.LengthGroupPhraseCompression(
                fileName, 4, printPlainText, printWordCodeword)
        # write compressed data to output.z file
        output_file = open(outputFilename+"."+algorithm.name+".z", "wb")
        output_file.write(compressedData)
        output_file.close()
        # print(compressedData, 'compressedData')
        # print(compressedData.decode('utf-8', 'ignore'))

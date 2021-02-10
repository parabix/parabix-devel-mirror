#!/usr/bin/env python3
import codecs
import binascii
# https://medium.com/analytics-vidhya/python-dictionaries-are-ordered-now-but-how-and-why-5d5a40ee327f
# dict in Python 3.7 is ordered by default, passing the use of OrderedDict for now
from collections import OrderedDict


class Compressor:
    def __init__(self):
        self.name = "edelta"
        self.hashTable = OrderedDict()
        self.encodePrefix = 192  # b'\xC0'
        self.encodeSuffix = 0    # b'\x00'
        self.compressed = bytearray(b'')
        self.expandSym = 218   # b'\xDA'
        self.words = []
        self.expandWord = ""

    def Name(self):
        return 'words'

    def expandSymbol(self, prevPos, currIndex):
        expandWord = ""
        maxLen = len(self.words)
        while(currIndex < maxLen and
              self.words[prevPos] == self.words[currIndex]):
            expandWord += self.words[currIndex]
            currIndex += 1
            prevPos += 1
        return expandWord, currIndex

    # for every occurrence of a symbol which is already encoded, try to find the longest match
    # of the sequence of symbols which are already encoded and encode the complete sequence with
    # a single encoded unit. This even helps encoding symbols of RLE format.

    # TODO: we can restrict the lookahead range for any symbol for faster compression.
    def Compress(self, words):
        self.words = words
        index = 0

        while index < len(words):
            # print(index, 'index')
            word = words[index]
            index += 1
            self.expandWord = word
            # if encoded hash for current "word" to be compressed is already calculated,
            # check for the longest sequence of symbols starting with current "word" already encoded
            #print(word, 'word')
            hash_pos = self.hashTable.get(word, None)
            if hash_pos:
                hashVal = hash_pos[0]
                # positions of all the previous occurrences of current "word"
                posArray = hash_pos[1]
                for pos in posArray:
                    expandedWord, newIndex = self.expandSymbol(
                        pos, index-1)
                    if len(expandedWord) > len(self.expandWord):
                        self.expandWord = expandedWord
                #print(self.expandWord, 'self.expandWord')
                # store the hash of the expandedWord and replace the symbol sequence
                # with hashVal in the compressed data
                if len(self.expandWord) > 6:
                    expandHash = self.hashTable.get(self.expandWord, None)
                    if not expandHash:
                        extendedHash = bytearray(b'')
                        extendedHash.append(self.expandSym)
                        extendedHash.append(len(self.expandWord))
                        lastSym = self.hashTable.get(
                            self.words[newIndex-1], None)
                        extendedHash.extend(lastSym[0])
                        self.compressed += hashVal
                        self.compressed += extendedHash
                        # store the symbol sequence, its hash and
                        # index of the word which was the start point of expansion
                        # DO NOT ADD EXTENED HASH IN HASHTABLE!
                        # self.hashTable[self.expandWord] = [
                        #    extendedHash, [index-1]]

                        # comment temporarily to east out decompression!
                        # self.hashTable[word][1].append(index-1)
                        index = newIndex
                    else:
                        self.compressed += hashVal
                        index = newIndex
                else:
                    # if there is no compression achieved by expanded hash, compress the symbol sequence
                    # as plain text and update the index to next symbol to be compressed
                    self.compressed += bytearray(word, 'utf-8')
            else:
                self.encodeSuffix += 1
                if self.encodeSuffix == 255:
                    self.encodePrefix += 1
                    self.encodeSuffix = 0
                encodedWord = bytearray(b'')
                encodedWord.append(self.encodePrefix)
                encodedWord.append(self.encodeSuffix)
                # index-1 corresponds to the array index of the word in the list of words
                self.hashTable[word] = [encodedWord, [index-1]]
                self.compressed += bytearray(word, 'utf-8')
        return self.compressed

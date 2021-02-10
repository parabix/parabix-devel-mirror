#!/usr/bin/env python3
from collections import OrderedDict
import codecs
import binascii
import uniseg.wordbreak

class Decompressor:
    def __init__(self):
        self.hashTable = OrderedDict()
        self.decodePrefix = 192  # b'\xC0'
        self.decodeSuffix = 0    # b'\x00'
        self.decompressed = bytearray(b'')
        self.expandSym = 218   # b'\xDA'
        self.words = []
        self.textBytes = ""
        self.word_list = OrderedDict()
        self.wordIndex = OrderedDict()
        self.wordPos = 0
        self.decodedWordsLen = 0

    def Name(self):
        return 'text'

    def Decompress(self, textBytes):
        self.textBytes = textBytes
        # extract all the plaintext symbols in the compressed text to create the hashTable
        # which is used in decoding the encoded symbols
        index = 0
        # add error check to confirm any valid UTF-8 sequences are not ignored
        # because they consist of code unit > 0xC0 (192)
        while index < len(textBytes):
            if textBytes[index] < 192:
                plaintext = bytearray(b'')
                while index < len(textBytes) and textBytes[index] < 192:
                    plaintext.append(textBytes[index])
                    index += 1
                words = self._Segment(plaintext)

                newWords = []
                for word in words:
                    if word not in self.word_list:
                        self.word_list[word] = self.wordPos
                        newWords.append(word)
                    self.wordPos += len(word)
                # create a hashTable incrementally similar to the compression hashTable with keys and values reversed
                self._HashCode(newWords)
            # check for symbol sequence codeword -> codeword '0xDA' length codeword
            elif textBytes[index] >= 192 and index+5 < len(textBytes):
                symLen1 = self._getCodeWordLen(index)
                index += 2
                if textBytes[index] == self.expandSym and textBytes[index+2] >= 192:
                    symLen2 = self._getCodeWordLen(index+2)
                    self.decodedWordsLen += (textBytes[index+1] -
                                             (symLen1+symLen2+2))
                    index += 4
                self.wordPos = index
            # just a single codeword encoded
            else:
                symLen = self._getCodeWordLen(index)
                index += 2
                self.wordPos = index

        # refer to the hashTable to decode the encoded symbols
        self._Decode(textBytes)
        return self.decompressed

    def _getCodeWordLen(self, index):
        codeWord = bytearray(b'')
        codeWord.append(self.textBytes[index])
        codeWord.append(self.textBytes[index+1])
        codeWordLen = self.hashTable.get(bytes(codeWord))
        self.decodedWordsLen += (len(codeWordLen[0])-2)
        return len(codeWordLen[0])

    def _Segment(self, plaintext):
        word_list = []
        for word in uniseg.wordbreak.words(plaintext.decode('utf-8')):
            word_list.append(word)
        return word_list

    def _HashCode(self, word_list):
        index = 0
        while index < (len(word_list)):
            word = word_list[index]
            self.wordPos += len(word)
            self.decodeSuffix += 1
            if self.decodeSuffix == 255:
                self.decodePrefix += 1
                self.decodeSuffix = 0
            encodedWord = bytearray(b'')
            encodedWord.append(self.decodePrefix)
            encodedWord.append(self.decodeSuffix)
            # print(encodedWord, 'encodedWord')
            self.hashTable[bytes(encodedWord)] = [
                word, self.decodedWordsLen]
            index += 1

    def _Decode(self, textBytes):
        index = 0
        word = bytearray(b'')
        while index < len(textBytes):
            if textBytes[index] >= 192:
                # if a plaintext sequence is already accumulated when a codeword is encountered,
                # add the plaintext to decompressed text and proceed with decoding the codeword/ codeword sequence
                if word and word[0] < 192:
                    self.decompressed += bytearray(word)
                    word = bytearray(b'')
                word.append(textBytes[index])
                word.append(textBytes[index+1])
                index += 2
                decmp = self.hashTable.get(bytes(word))
                if decmp:
                    self.decompressed += bytearray(decmp[0], 'utf-8')
                    # upon decoding current codeword, check if the following is a sequence of codewords
                    # assumes that one symbol with prefix '0xDA' between 2 consecutive codewords is length
                    # of symbols to be decompressed
                    if (index+1 < len(textBytes) and textBytes[index] == self.expandSym):
                        index = self.checkExtendedCodeword(
                            index+2, word, textBytes[index+1])
                    word = bytearray(b'')
            else:
                word.append(textBytes[index])
                index += 1
        if word:
            self.decompressed += bytearray(word)

    def checkExtendedCodeword(self, index, firstCodeword, seqLength):
        if index+1 < len(self.textBytes) and self.textBytes[index] >= 192:
            lastCodeword = bytearray(b'')
            lastCodeword.append(self.textBytes[index])
            lastCodeword.append(self.textBytes[index+1])
            #print(firstCodeword, 'firstCodeword')
            #print(lastCodeword, 'lastCodeword')
            firstSym = self.hashTable.get(bytes(firstCodeword))
            lastSym = self.hashTable.get(bytes(lastCodeword))
            #print(firstSym, 'firstSym')
            #print(lastSym, 'lastSym')
            lenToBeCopied = seqLength - len(firstSym[0])
            if firstCodeword == lastCodeword and (lenToBeCopied % len(firstSym[0])) == 0:
                firstSym = self.hashTable.get(bytes(firstCodeword))
                seqLength -= len(firstSym[0])
                while seqLength > 0:
                    self.decompressed += bytearray(firstSym[0], 'utf-8')
                    seqLength -= len(firstSym[0])
            else:
                if firstSym:
                    bytesDecoded = firstSym[1]
                    firstSymIndex = self.word_list.get(firstSym[0])
                    firstSymPos = firstSymIndex+bytesDecoded
                    # print(bytesDecoded, 'bytesDecoded')
                    seqLength -= len(firstSym[0])
                    firstSymPos += len(firstSym[0])
                start = firstSymPos
                self.decompressed.extend(
                    self.decompressed[start: start+seqLength])
            return index+2
        return index

# TODO: Find, insert, delete and replace symbol/ symbol sequences in compressed text without decompressing

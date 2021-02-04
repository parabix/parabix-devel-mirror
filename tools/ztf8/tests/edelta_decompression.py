#!/usr/bin/env python3
from collections import OrderedDict
import codecs
import binascii
import uniseg.wordbreak

# 1. segment all the plain text words
# 2. generate code-word for the plain text symbols
# 3. parse the encoded symbols to locate the direct mapping of codewords OR
# if prefix 0xDA is found, capture the length of symbol sequence encoded.
# referring to all the positions of the prefix symbol (seen before 0xDA), try to find the length of the
# symbols of specified length to be replaced in the decompressed text (needs more info to find the exact
# symbol sequence to be replaced in the place of codeword. Perhaps include the index position of the start
# of symbol sequence while encoding.


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
        #print(self.textBytes, ' self.textBytes')
        #print(len(self.textBytes), ' len - self.textBytes')
        # extract all the plaintext symbols in the compressed text to create the hashTable
        # which is used in decoding the encoded symbols
        index = 0
        # add error check to confirm any valid UTF-8 sequences are not ignored
        # because they consist of code unit > 0xC0 (192)
        while index < len(textBytes):
            # print(textBytes[index], "textBytes[index]")
            if textBytes[index] < 192:
                plaintext = bytearray(b'')
                while index < len(textBytes) and textBytes[index] < 192:
                    plaintext.append(textBytes[index])
                    index += 1
                words = self._Segment(plaintext)
                # print(plaintext, 'plaintext')
                # print(words, 'words')
                for word in words:
                    if word not in self.word_list:
                        self.word_list[word] = [self.wordPos, None]
                    self.wordPos += len(word)
            # check for symbol sequence codeword -> codeword length codeword
            elif textBytes[index] >= 192 and index+4 < len(textBytes):
                index += 2
                if textBytes[index] < 256 and textBytes[index+1] >= 192:
                    index += 3
                self.wordPos = index
            # just a single codeword encoded
            else:
                index += 2
                self.wordPos = index

        # create a hashTable similar to the compression hashTable with keys and values reversed
        # print(word_list.keys(), 'word_list.keys()')
        self._HashCode(list(self.word_list.keys()))
        # refer to the hashTable to decode the encoded symbols
        self._Decode(textBytes)
        for items in self.hashTable.items():
            print(items[0], items[1])
        for items in self.word_list.items():
            print(items[0], items[1])
        return self.decompressed

    def _Segment(self, plaintext):
        word_list = []
        for word in uniseg.wordbreak.words(plaintext.decode('utf-8')):
            word_list.append(word)
        return word_list

    def _HashCode(self, word_list):
        # print(len(plaintextIndex), 'plaintextIndex')
        index = 0
        while index < (len(word_list)):
            # if word_list[index] != "\n" and index+1 < len(word_list) and len(word_list[index+1]) == 1:
            #    word = word_list[index] + word_list[index+1]
            #    index += 2
            # else:
            word = word_list[index]
            if not self.hashTable.get(word):
                self.decodeSuffix += 1
                if self.decodeSuffix == 255:
                    self.decodePrefix += 1
                    self.decodeSuffix = 0
                encodedWord = bytearray(b'')
                encodedWord.append(self.decodePrefix)
                encodedWord.append(self.decodeSuffix)
                # print(encodedWord, 'encodedWord')
                self.hashTable[bytes(encodedWord)] = [word]
            index += 1

    def _Decode(self, textBytes):
        index = 0
        word = bytearray(b'')
        while index < len(textBytes):
            # if textBytes[index] == "\n":
            #    self.decompressed += bytearray("\n", 'utf-8')
            #    index += 1
            if textBytes[index] >= 192:
                # if a plaintext sequence is already accumulated when a codeword is encountered,
                # add the plaintext to decompressed text and proceed with decoding the codeword/ codeword sequence
                if word and word[0] < 192:
                    self.decompressed += bytearray(word)
                    word = bytearray(b'')
                word.append(textBytes[index])
                word.append(textBytes[index+1])
                #print(word, 'word')
                index += 2
                decmp = self.hashTable.get(bytes(word))
                if decmp:
                    print(decmp[0], 'decmp')
                    self.decompressed += bytearray(decmp[0], 'utf-8')
                    #self.decompressed += bytearray(decmp[0], 'utf-8')
                    # upon decoding current codeword, check if the following is a sequence of codewords
                    if (index+1 < len(textBytes) and textBytes[index+1] >= 192):
                        index = self.checkExtendedCodeword(
                            index+1, word, textBytes[index], None)
                        print(index, 'index updated')
                word = bytearray(b'')
            else:
                if self.word_list.get(word.decode('utf-8')):
                    #print(word.decode('utf-8'), 'word.decode(utf-8)')
                    self.word_list[word.decode(
                        'utf-8')][1] = self.decodedWordsLen
                    self.decompressed += bytearray(word)
                    word = bytearray(b'')
                word.append(textBytes[index])
                index += 1
        if word:
            self.decompressed += bytearray(word)

    # def checkExtendedCodeword(self, index of second codeword (if a seq of symbols encoded), why firstCodeword?, seqLength to be copied starting from
    # the first codeword until last codeword (actually not needed! as we are just considering the first index and copy the following characters of certain length)
    # lastCodeWord was encoded assuming we'll be copying the consecutive decoded symbols from the hashTable, and if we were to keep track
    # of all the occurrences of the decoded firstCodeword and lookahead from those positions to find our current encoded symbol sequence
    def checkExtendedCodeword(self, index, firstCodeword, seqLength, firstSymPos):
        print(index, 'index')
        copyFromDecompressed = False
        print(self.textBytes[index], 'self.textBytes[index]')
        if index+1 < len(self.textBytes) and self.textBytes[index] >= 192:
            print(firstCodeword, 'firstCodeword')
            print(seqLength, 'seqLength')
            # yes we have symbol sequence encoded that need to be expanded
            lastCodeword = bytearray(b'')
            lastCodeword.append(self.textBytes[index])
            lastCodeword.append(self.textBytes[index+1])

            firstSym = self.hashTable.get(bytes(firstCodeword))
            #self.decompressed += bytearray(firstSym[0], 'utf-8')
            self.decodedWordsLen += len(firstSym[0])-2
            print(self.decodedWordsLen, 'self.decodedWordsLen - after')

            if firstCodeword == lastCodeword:
                firstSym = self.hashTable.get(bytes(firstCodeword))
                seqLength -= len(firstSym[0])
                while seqLength > 0:
                    self.decompressed += bytearray(firstSym[0], 'utf-8')
                    seqLength -= len(firstSym[0])
            else:
                self.decodedWordsLen -= 3
                firstSym = self.hashTable.get(bytes(firstCodeword))
                print(firstSym, 'firstSym')
                if firstSym:
                    if firstSymPos is None:
                        firstSymIndex = self.word_list.get(firstSym[0])
                        firstSymPos = firstSymIndex[0]
                    else:
                        copyFromDecompressed = True
                    print(firstSym[0], 'firstSym[0]')
                    print(firstSymPos, ' firstSymPos 1')
                    # self.decompressed += bytearray(firstSym[0], 'utf-8')
                    seqLength -= len(firstSym[0])
                    firstSymPos += len(firstSym[0])
                    print(firstSymPos, ' firstSymPos 2')
                    print(seqLength, 'seqLength 2')
                # firstCodewordPfx = firstCodeword[0]
                # firstCodewordSfx = firstCodeword[1]
                print(self.decompressed, 'self.decompressed 1')
                start = firstSymPos
                toCopy = seqLength
                print(self.textBytes[firstSymPos:firstSymPos+seqLength],
                      'self.textBytes[firstSymPos:firstSymPos+seqLength]')
                print(self.textBytes, 'self.textBytes')
                print(firstSymPos+seqLength, 'firstSymPos+seqLength')
                bytesCopied = 0
                while start < firstSymPos+seqLength:
                    print(start, 'start')
                    if self.textBytes[start] >= 192 and start+2 < len(self.textBytes):
                        print((start-firstSymPos), '(firstSymPos-start)')
                        # print(self.word_list.get(
                        #    firstSym[0]), 'firstSym before')
                        # update the symPos to the latest occurrnece position in the decompressed text
                        #self.word_list[firstSym[0]] += self.decodedWordsLen
                        #firstSymPos += self.decodedWordsLen
                        print(firstSymIndex[1],
                              'firstSymIndex')
                        recWord = bytearray(b'')
                        recWord.append(self.textBytes[start])
                        recWord.append(self.textBytes[start+1])
                        # start = self.checkExtendedCodeword(
                        #    start+3, recWord, seqLength-(start-firstSymPos), firstSymPos+(start-firstSymPos))
                        print(firstSymPos+firstSymIndex[1],
                              'start copying from here')
                        print(toCopy,
                              'toCopy to be copied')
                        self.copySymSeq(
                            firstSymPos+firstSymIndex[1]+bytesCopied, toCopy)
                        # print(
                        #    self.decompressed[firstSymPos+firstSymIndex[1]+bytesCopied:firstSymPos+firstSymIndex[1]+bytesCopied+toCopy], 'self.decompressed 2')
                        start += 5
                        seqLength -= toCopy
                    else:
                        # if copyFromDecompressed:
                        self.decompressed.append(self.textBytes[start])
                        # else:
                        #    self.decompressed.append(self.textBytes[start])
                        start += 1
                        self.decodedWordsLen += 1
                        toCopy -= 1
                        bytesCopied += 1
                print(self.decodedWordsLen, 'self.decodedWordsLen - after')
                # self.decompressed.extend(
                #    self.textBytes[firstSymPos:firstSymPos+seqLength])
                print(self.decompressed, 'self.decompressed 3')
                # while seqLength > 0:
                #    print(bytearray(
                #        self.textBytes[firstSymPos]).decode('utf-8'),
                #        'self.textBytes[firstSymPos]')
                #      firstCodewordSfx += 1
                #      firstCodeword[1] = (firstCodeword[1] + 1) % 255
                #       if firstCodeword[1] == 255:
                #            firstCodeword[0] += 1
                # firstSym = self.hashTable.get(bytes(firstCodeword))
                # if firstSym:
                #    self.decompressed.append(self.textBytes[firstSymPos])
                #    print(self.decompressed.decode(
                #        'utf-8'), 'self.decompressed')
                #    seqLength -= 1
            return index+2
        return index

    def copySymSeq(self, startPos, len):
        self.decompressed.extend(self.decompressed[startPos: startPos+len])
# TODO: Find, insert, delete and replace symbol/ symbol sequences in compressed text without decompressing

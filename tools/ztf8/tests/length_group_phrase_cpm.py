import codecs
import binascii
import copy
import itertools
import nltk
import uniseg.wordbreak
import compress_words
import decompress_words


class LGPhraseCompressor:
    def __init__(self):
        self.name = "length-group-phrase-compression"
        self.wordsLen = 0
        self.words = []
        self.freqPhraseToCodewordHashTableList = [{}, {}, {}, {}, {}]
        self.lengthwisePhraseList = [{}, {}, {}, {}, {}]
        self.allPhrases = [{}, {}, {}, {}, {}]
        self.freqPhrases = {}
        # C0-C7, C8-CF, D0-DF, E0-EF, F0-FF
        self.prefixList = [192, 200, 208, 224, 240]
        self.compressed = bytearray(b'')
        self.cmp = bytearray(b'')
        self.bitmix = [[4, 1, 7, 2, 0, 6, 5, 3], [1, 2, 4, 5, 0, 3, 6, 7], [
            0, 5, 4, 6, 2, 3, 7, 1], [7, 1, 6, 4, 3, 0, 5, 2], [3, 2, 5, 4, 7, 6, 0, 1]]
        self.allHashVals = {}
        self.compressedPhraseStartIdx = {}
        self.plainTextPhraseStartIdx = {}
        self.noSuffixCollisions = [0, 0, 0, 0, 0]
        self.printPlainText = False
        self.printWordCodeword = False
        self.hideHashVal = False
        self.absIdx = 0

    def Name(self):
        return self.name

    def LengthGroupPhraseCompression(self, fileName, numWords, printPlainText, printWordCodeword):
        self.printPlainText = printPlainText
        self.printWordCodeword = printWordCodeword
        if self.printPlainText or self.printWordCodeword:
            self.hideHashVal = True
        with open(fileName, 'rt') as infile:
            while True:
                segment = infile.read(1000000)
                if not segment:
                    break
                words = compress_words.Compressor().readSegment(segment)
                self.wordsLen += len(words)
                self.words = words
                self.cmp = self.idPhraseCompress(numWords, words)
                for phraseLen in range(numWords-1, 0, -1):
                    self.absIdx = 0
                    self.cmp = self.idSubPhraseCompress(0, phraseLen, self.cmp)
                self.absIdx = 0
                self.compressed += self.cmp
        # self.printSummary()
        return self.compressed

    def printSummary(self):
        print(self.wordsLen, 'words length')
        for collisions, i in enumerate(self.noSuffixCollisions):
            print('collisions: ', i, ' --> ',
                  collisions)
        print("freqPhraseToCodewordHashTableList table size")
        for h in self.freqPhraseToCodewordHashTableList:
            print(len(h))
            for i in h.items():
                print(i)
        print("All hashVals")
        for h in self.allHashVals.items():
            print(h)
        print("self.compressedPhraseStartIdx")
        for h in self.compressedPhraseStartIdx:
            print(h)
        # print(self.compressed, 'self.compressed')

    def idPhraseCompress(self, phraseLen, words):
        compress = bytearray(b'')
        if not words:
            return compress
        # print(words, 'words')
        wordsLen = len(words)
        index = 0
        while index < wordsLen:
            if index+(phraseLen-1) < wordsLen:
                phrase, singleByteSyms = compress_words.Compressor(
                ).getPhrase(words, index, phraseLen, wordsLen)
                # print(phrase, '->check phrase:',
                #      phraseLen, '-> start index=', self.absIdx)
            else:
                for w in words[index::]:
                    compress += bytearray(w, 'utf-8')
                    self.absIdx += 1
                break
            if phrase is None:
                for w in words[index::]:
                    compress += bytearray(w, 'utf-8')
                    self.absIdx += 1
                break
            wLen = len(phrase)
            hashVal = None
            hashTablePos = -1
            hashVal, hashTablePos, startIndex, numSymInPhrase = self.getHashValfromTable(
                wLen, phrase)
            overlappingPhrase = False
            if hashVal:
                overlappingPhrase = self.checkOverlappingPhrase(
                    self.absIdx, startIndex, singleByteSyms+phraseLen, phrase)
                if overlappingPhrase:
                    compress += bytearray(words[index], 'utf-8')
                    index += 1
                    self.absIdx += 1
                    #compress += bytearray(phrase, 'utf-8')
                    #index += singleByteSyms+phraseLen
                    #self.absIdx += singleByteSyms+phraseLen
                else:
                    start = 0
                    while start < (singleByteSyms + phraseLen):
                        self.compressedPhraseStartIdx[self.absIdx +
                                                      start] = numSymInPhrase
                        start += 1
                    #print(hashVal, 'hashVal 1')
                    if self.printPlainText:
                        compress += bytearray(phrase, 'utf-8')
                    if self.printWordCodeword:
                        compress += bytearray(str(hashVal), 'utf-8')
                    if not self.hideHashVal:
                        compress += hashVal
                    index += (singleByteSyms + phraseLen)
                    self.absIdx += (singleByteSyms + phraseLen)
            else:
                if wLen < 3:
                    compress += bytearray(phrase, 'utf-8')
                    index += (singleByteSyms + phraseLen)
                    self.absIdx += (singleByteSyms + phraseLen)
                elif 3 <= wLen <= 32:
                    encodedWord, isOverlapping = self.checkPhraseFreq(
                        wLen, hashTablePos, phrase, phraseLen+singleByteSyms, self.absIdx)
                    if encodedWord:
                        #print(encodedWord, 'hashVal 2')
                        if self.printPlainText:
                            compress += bytearray(phrase, 'utf-8')
                        if self.printWordCodeword:
                            compress += bytearray(str(encodedWord),
                                                  'utf-8')
                        if not self.hideHashVal:
                            compress += encodedWord
                        index += (singleByteSyms + phraseLen)
                        self.absIdx += (singleByteSyms + phraseLen)
                    # elif isOverlapping:
                    #    compress += bytearray(phrase, 'utf-8')
                    #    index += singleByteSyms+phraseLen
                    #    self.absIdx += singleByteSyms+phraseLen
                    else:
                        compress += bytearray(words[index], 'utf-8')
                        index += 1
                        self.absIdx += 1
                elif wLen > 32:
                    compress += bytearray(words[index], 'utf-8')
                    index += 1
                    self.absIdx += 1
        return compress

    def checkOverlappingPhrase(self, absWordIdx, curIndex, numSymInPhrase, phrase):
        # print("checkOverlapping:", phrase,
        #      curIndex, self.absIdx, numSymInPhrase)
        overlappingPhrase = False
        # only True for smaller phrases
        for numOfWords in range(0, numSymInPhrase):
            if self.absIdx+numOfWords in self.compressedPhraseStartIdx or self.absIdx+numOfWords in self.plainTextPhraseStartIdx:
                overlappingPhrase = True
                return overlappingPhrase

        # check for phrases with same number of words
        # first occurrence should not be compressed.
        for numOfWords in range(0, numSymInPhrase):
            if self.absIdx+numOfWords in self.plainTextPhraseStartIdx:
                overlappingPhrase = True
                return overlappingPhrase

        for numOfWords in range(0, numSymInPhrase):
            if curIndex+numOfWords in self.compressedPhraseStartIdx:
                overlappingPhrase = True
        return overlappingPhrase

    def idSubPhraseCompress(self, startIdx, phraseLen, compressed):
        # print(compressed, 'compressed')
        absIdx = 0
        cwWords = 0
        index = startIdx
        subCompressed = bytearray(b'')
        # add error check to confirm any valid UTF-8 sequences are not ignored
        # because they consist of code unit > 0xC0 (192)
        length = len(compressed)
        plaintext = bytearray(b'')
        while index < length:
            # print(index, cwWords, 'index+cwWords')
            if compressed[index] < 192:
                plaintext = bytearray(b'')
                while index < length and compressed[index] < 192:
                    plaintext.append(compressed[index])
                    index += 1
                # print(plaintext, 'plaintext')
            else:
                codeWord = bytearray(b'')
                cwPfx = 0
                cwHex = ""
                if compressed[index] >= 192 and compressed[index] <= 223 and index+1 < length:
                    if compressed[index+1] >= 128:
                        plaintext.append(compressed[index])
                        plaintext.append(compressed[index+1])
                    else:
                        codeWord.append(compressed[index])
                        codeWord.append(compressed[index+1])
                        cwHex += str(hex(compressed[index]))[2:]
                        cwHex += str(hex(compressed[index+1]))[2:]
                        cwPfx = compressed[index]
                    index += 2
                elif compressed[index] >= 224 and compressed[index] <= 239 and index+2 < length:
                    if compressed[index+1] >= 128 and compressed[index+2] >= 128:
                        plaintext.append(compressed[index])
                        plaintext.append(compressed[index+1])
                        plaintext.append(compressed[index+2])
                    else:
                        codeWord.append(compressed[index])
                        codeWord.append(compressed[index+1])
                        codeWord.append(compressed[index+2])
                        cwHex += str(hex(compressed[index]))[2:]
                        cwHex += str(hex(compressed[index+1]))[2:]
                        cwHex += str(hex(compressed[index+2]))[2:]
                        cwPfx = compressed[index]
                    index += 3
                elif compressed[index] >= 240 and compressed[index] <= 247 and index+3 < length:
                    if compressed[index+1] >= 128 and compressed[index+2] >= 128 and compressed[index+3] >= 128:
                        plaintext.append(compressed[index])
                        plaintext.append(compressed[index+1])
                        plaintext.append(compressed[index+2])
                        plaintext.append(compressed[index+3])
                    else:
                        codeWord.append(compressed[index])
                        codeWord.append(compressed[index+1])
                        codeWord.append(compressed[index+2])
                        codeWord.append(compressed[index+3])
                        cwHex += str(hex(compressed[index]))[2:]
                        cwHex += str(hex(compressed[index+1]))[2:]
                        cwHex += str(hex(compressed[index+2]))[2:]
                        cwHex += str(hex(compressed[index+3]))[2:]
                        cwPfx = compressed[index]
                    index += 4
                else:
                    codeWord.append(compressed[index])
                    codeWord.append(compressed[index+1])
                    codeWord.append(compressed[index+2])
                    codeWord.append(compressed[index+3])
                    cwHex += str(hex(compressed[index]))[2:]
                    cwHex += str(hex(compressed[index+1]))[2:]
                    cwHex += str(hex(compressed[index+2]))[2:]
                    cwHex += str(hex(compressed[index+3]))[2:]
                    cwPfx = compressed[index]
                    index += 4
                if codeWord:
                    cwLen, hashTablePos = decompress_words.Decompressor()._codeWordLenRange(cwPfx)
                    strCodeWord = str(cwLen) + cwHex
                    # print(strCodeWord, 'strCodeWord')
                    if strCodeWord in self.allHashVals:
                        cwwords = compress_words.Compressor().readSegment(
                            self.allHashVals[strCodeWord][0])
                        cwWords = len(cwwords)
                    # get the codeword str from bytearray codeWord and figure out the number of words in encoded phrase to determine
                    # the cwWords (subtract num of codeword bytes) needed to keep track of absIdx
                    words = compress_words.Compressor().readSegment(plaintext.decode('utf-8'))
                    plaintext = bytearray(b'')
                    subCompressed += self.idPhraseCompress(phraseLen, words)
                    # print(self.allHashVals[strCodeWord][0], 'self.allHashVals[strCodeWord][0]', cwWords,
                    #      'cwWords', self.absIdx, 'absIdx')
                    self.absIdx += cwWords
                    subCompressed += codeWord
        if plaintext:
            words = compress_words.Compressor().readSegment(plaintext.decode('utf-8'))
            plaintext = bytearray(b'')
            subCompressed += self.idPhraseCompress(phraseLen, words)
        return subCompressed

    def getHashValfromTable(self, wLen, word):
        hashVal = None
        hashTablePos = -1
        startIdx = -1
        numSymInPhrase = -1
        if wLen == 3:
            hashTablePos = 0
            hashVal, startIdx, numSymInPhrase = self.freqPhraseToCodewordHashTableList[0].get(
                word, [None, None, None])
        elif wLen == 4:
            hashTablePos = 1
            hashVal, startIdx, numSymInPhrase = self.freqPhraseToCodewordHashTableList[1].get(
                word, [None, None, None])
        elif wLen > 4 and wLen <= 8:
            hashTablePos = 2
            hashVal, startIdx, numSymInPhrase = self.freqPhraseToCodewordHashTableList[2].get(
                word, [None, None, None])
        elif wLen > 8 and wLen <= 16:
            hashTablePos = 3
            hashVal, startIdx, numSymInPhrase = self.freqPhraseToCodewordHashTableList[3].get(
                word, [None, None, None])
        elif wLen > 16 and wLen <= 32:
            hashTablePos = 4
            hashVal, startIdx, numSymInPhrase = self.freqPhraseToCodewordHashTableList[4].get(
                word, [None, None, None])
        return hashVal, hashTablePos, startIdx, numSymInPhrase

    def getCodeword(self, word, numSymsInPhrase):
        wLen = len(word)
        wordLen, encodedSuffix = compress_words.Compressor().getHashVal(word,
                                                                        numSymsInPhrase)
        encodedPrefix = 0
        sfxIdx = 0
        # while encodedPrefix == 0:
        encodedPrefix = compress_words.Compressor(
        ).getPrefix(encodedSuffix[0], wLen)
        # sfxIdx += 1
        if encodedPrefix <= 247:
            # switch just one of the suffix bytes(first one) to ASCII value
            encodedSuffix[0] = encodedSuffix[0] % 128
        encodedWord = bytearray(b'')
        encodedWord.append(encodedPrefix)
        finalSuffix = ""
        for suffix in encodedSuffix:
            encodedWord.append(suffix)
            finalSuffix += str(hex(suffix)[2:])
        codeWordHex = str(hex(encodedPrefix)[2:]) + finalSuffix
        # codeWordHex = str(hex(encodedPrefix)) + finalSuffix
        codeword = str(wLen)+codeWordHex
        # print(encodedWord, 'encodedWord')
        return encodedWord, codeword

    def checkPhraseFreq(self, wLen, hashTablePos, curPhrase, numSymsInPhrase, absIdx):
        encodedWord, codeword = self.getCodeword(curPhrase, numSymsInPhrase)
        phrases = self.allHashVals.get(codeword, None)
        if phrases:
            for phrase in phrases:
                if phrase == curPhrase:
                    # check if the word identified is in the overlapping range of its last occurrence
                    phraseSeenAtIdx, isCompressed = self.lengthwisePhraseList[hashTablePos].get(
                        curPhrase, None)
                    if (phraseSeenAtIdx and self.absIdx < phraseSeenAtIdx+numSymsInPhrase) or isCompressed:
                        return None, True
                    overlappingPhrase = self.checkOverlappingPhrase(
                        self.absIdx, phraseSeenAtIdx, numSymsInPhrase, curPhrase)
                    if overlappingPhrase:
                        return None, True
                    self.freqPhraseToCodewordHashTableList[hashTablePos][curPhrase] = [
                        encodedWord, phraseSeenAtIdx, numSymsInPhrase]
                    start = 0
                    while start < numSymsInPhrase:
                        self.plainTextPhraseStartIdx[phraseSeenAtIdx +
                                                     start] = numSymsInPhrase
                        self.compressedPhraseStartIdx[self.absIdx +
                                                      start] = numSymsInPhrase
                        start += 1
                    # changing isCompressed value
                    self.lengthwisePhraseList[hashTablePos][curPhrase] = [
                        self.absIdx, 1]
                    return encodedWord, False
                elif len(phrase) == len(curPhrase):
                    # if not the first phrase in segment with codeword C, phrase is not compressed
                    self.noSuffixCollisions[hashTablePos] += 1
                    return None, False
                # UNUSED:
                # branch helps keeping track of all phrases with same codeword
                # (not just the first phrase with codeword C)
                else:
                    phrases.append(curPhrase)
                    self.allHashVals[codeword] = phrases
                    self.lengthwisePhraseList[hashTablePos][curPhrase] = [
                        self.absIdx, 0]  # [absIdx,notCompressed]
        else:
            self.allHashVals[codeword] = [curPhrase]
            self.lengthwisePhraseList[hashTablePos][curPhrase] = [
                self.absIdx, 0]
        return None, False

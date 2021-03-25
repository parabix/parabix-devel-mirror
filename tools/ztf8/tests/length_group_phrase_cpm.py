import codecs
import binascii
import copy
import itertools
import nltk
import uniseg.wordbreak
import compress_words


class LGPhraseCompressor:
    def __init__(self):
        self.name = "length-group-phrase-compression"
        self.wordsLen = 0
        self.words = []
        self.codewordHashTableList = [{}, {}, {}, {}, {}]
        self.lengthwisePhrasesList = [{}, {}, {}, {}, {}]
        self.prefixList = [192, 196, 200, 208, 248]
        self.compressed = bytearray(b'')
        self.cmp = bytearray(b'')
        self.bitmix = [[4, 1, 7, 2, 0, 6, 5, 3], [1, 2, 4, 5, 0, 3, 6, 7], [
            0, 5, 4, 6, 2, 3, 7, 1], [7, 1, 6, 4, 3, 0, 5, 2], [3, 2, 5, 4, 7, 6, 0, 1]]
        self.hashVals = {}
        self.collisionsCWLenExceeded = 0
        self.noSuffixCollisions = 0
        self.printPlainText = False
        self.printWordCodeword = False
        self.hideHashVal = False

    def Name(self):
        return self.name

    def LengthGroupPhraseCompression(self, fileName, numWords, printPlainText, printWordCodeword):
        self.printPlainText = printPlainText
        self.printWordCodeword = printWordCodeword
        if self.printPlainText or self.printWordCodeword:
            self.hideHashVal = True
        with open(fileName, 'rt') as infile:
            while True:
                #self.collisionsCWLenExceeded = 0
                #self.noSuffixCollisions = 0
                segment = infile.read(1000000)
                if not segment:
                    break
                words = compress_words.Compressor().readSegment(segment)
                self.words = words
                # first compress the segment for all the possible 4 word phrases.
                # Once done, only then move ahead with compressing 3 word phrases which could not be
                # compressed as 4 word phrases and continue till 1 word phrase/ plaintext retains.
                compressed = self.idPhraseCompress(numWords, words)
                # Now, if we give words to compress as phrase of 3 words, we first check the hashtable for a 4 word phrase to be present.
                # If present, we know that it's already compressed as a 4 word phrase and skip those 4 words.
                # Else, we take first 3 words of the phrase and make an entry in the phrase table. Repeat the same for every phrase of 4 words.
                # At any point, we find a phrase of 3 words repeated in the phrase table, create an entry for it and add it in the hash table.
                # Repeat the same by reducing the number of words in phrases till 1.
                for phraseLen in range(numWords-1, 0, -1):
                    self.cmp = self.idSubPhraseCompress(
                        phraseLen, compressed)
                self.compressed += self.cmp

        print(len(words), 'words length')
        # print('collisions: collisionsCWLenExceeded -> ',
        #      self.collisionsCWLenExceeded)
        print('collisions: noSuffixCollisions -> ',
              self.noSuffixCollisions)
        print("codewordHashTableList table size")
        for h in self.codewordHashTableList:
            print(len(h))
            # for i in h.items():
            #    print(i)
        print("lengthwisePhrasesList table size")
        for h in self.lengthwisePhrasesList:
            print(len(h))
        return self.compressed

    def idPhraseCompress(self, phraseLen, words):
        wordsLen = len(words)
        index = 0
        compress = bytearray(b'')
        while index < wordsLen:
            # remove this check?
            if index+(phraseLen-1) < wordsLen:
                phrase, singleByteSyms = compress_words.Compressor().getPhrase(
                    words, index, phraseLen, wordsLen)
            else:
                for w in words[index::]:
                    compress += bytearray(w, 'utf-8')
                break
            wLen = len(phrase)
            hashVal = None
            hashTablePos = -1
            hashVal, hashTablePos, startIdx = self.getHashValfromTable(
                wLen, phrase)
            if hashVal:
                if self.printPlainText:
                    compress += bytearray(phrase, 'utf-8')
                if self.printWordCodeword:
                    #compressed += bytearray(phrase+'-41>', 'utf-8')
                    compress += bytearray(str(hashVal), 'utf-8')
                if not self.hideHashVal:
                    compress += hashVal
                index += (singleByteSyms + phraseLen)
            else:
                if wLen < 3:
                    compress += bytearray(phrase, 'utf-8')
                    index += (singleByteSyms + phraseLen)
                elif 3 <= wLen <= 32:
                    encodedWord = self.checkPhraseFreq(
                        wLen, hashTablePos, phrase, phraseLen+singleByteSyms, None, None, index)
                    if encodedWord is not None:
                        if self.printPlainText:
                            compress += bytearray(phrase, 'utf-8')
                        if self.printWordCodeword:
                            compress += bytearray(str(encodedWord), 'utf-8')
                        if not self.hideHashVal:
                            compress += encodedWord
                        index += (singleByteSyms + phraseLen)
                    else:
                        compress += bytearray(words[index], 'utf-8')
                        index += 1
                elif wLen > 32:
                    compress += bytearray(words[index], 'utf-8')
                    index += 1
        return compress

    def idSubPhraseCompress(self, phraseLen, compressed):
        #print(compressed, 'compressed')
        index = 0
        subCompressed = bytearray(b'')
        # add error check to confirm any valid UTF-8 sequences are not ignored
        # because they consist of code unit > 0xC0 (192)
        length = len(compressed)
        plaintext = bytearray(b'')
        while index < length:
            if compressed[index] < 192:
                plaintext = bytearray(b'')
                while index < length and compressed[index] < 192:
                    plaintext.append(compressed[index])
                    index += 1
                #print(plaintext, 'plaintext')
            else:
                codeWord = bytearray(b'')
                if compressed[index] >= 192 and compressed[index] <= 223 and index+1 < length:
                    # ambiguous!!!!!!
                    # if compressed[index+1] >= 128:
                    #    plaintext.append(compressed[index])
                    #    plaintext.append(compressed[index+1])
                    # else:
                    codeWord.append(compressed[index])
                    codeWord.append(compressed[index+1])
                    index += 2
                elif compressed[index] >= 224 and compressed[index] <= 239 and index+2 < length:
                    if compressed[index+1] >= 128 and compressed[index+2] >= 128:
                        plaintext.append(compressed[index])
                        plaintext.append(compressed[index+1])
                        plaintext.append(compressed[index+2])
                        index += 3
                    else:
                        codeWord.append(compressed[index])
                        codeWord.append(compressed[index+1])
                        index += 2
                elif compressed[index] >= 240 and compressed[index] <= 247 and index+3 < length:
                    if compressed[index+1] >= 128 and compressed[index+2] >= 128 and compressed[index+3] >= 128:
                        plaintext.append(compressed[index])
                        plaintext.append(compressed[index+1])
                        plaintext.append(compressed[index+2])
                        plaintext.append(compressed[index+3])
                        index += 4
                    else:
                        codeWord.append(compressed[index])
                        codeWord.append(compressed[index+1])
                        index += 2
                else:
                    codeWord.append(compressed[index])
                    codeWord.append(compressed[index+1])
                    index += 2
                if codeWord:
                    #print(codeWord, 'codeWord')
                    #print(plaintext, 'plaintext')
                    words = compress_words.Compressor().readSegment(plaintext.decode('utf-8'))
                    plaintext = bytearray(b'')
                    subCompressed += self.idPhraseCompress(phraseLen, words)
                    subCompressed += codeWord
        if plaintext:
            words = compress_words.Compressor().readSegment(plaintext.decode('utf-8'))
            plaintext = bytearray(b'')
            subCompressed += self.idPhraseCompress(phraseLen, words)
        return subCompressed

    def getHashValfromTable(self, wLen, word):
        hashVal = None
        hashTablePos = -1
        startIdx = 1
        if wLen == 3:
            hashTablePos = 0
            hashVal, startIdx = self.codewordHashTableList[0].get(
                word, [None, None])
        elif wLen == 4:
            hashTablePos = 1
            hashVal, startIdx = self.codewordHashTableList[1].get(
                word, [None, None])
        elif wLen > 4 and wLen <= 8:
            hashTablePos = 2
            hashVal, startIdx = self.codewordHashTableList[2].get(
                word, [None, None])
        elif wLen > 8 and wLen <= 16:
            hashTablePos = 3
            hashVal, startIdx = self.codewordHashTableList[3].get(
                word, [None, None])
        elif wLen > 16 and wLen <= 32:
            hashTablePos = 4
            hashVal, startIdx = self.codewordHashTableList[4].get(
                word, [None, None])
        return hashVal, hashTablePos, startIdx

    def getCodeword(self, word, numSymsInPhrase):
        wLen = len(word)
        wordLen, encodedSuffix = compress_words.Compressor().getHashVal(word,
                                                                        numSymsInPhrase)
        #print(word, 'word --> len:', wLen)
        encodedPrefix = compress_words.Compressor().getPrefix(encodedSuffix, wLen)
        #print(encodedPrefix, 'encodedPrefix')
        encodedWord = bytearray(b'')
        encodedWord.append(encodedPrefix)
        encodedWord.append(encodedSuffix)
        codeWordHex = str(hex((encodedPrefix << 8) | encodedSuffix))
        codeword = str(wLen)+codeWordHex[2:]
        return encodedWord, codeword

    def checkPhraseFreq(self, wLen, hashTablePos, word, numSymsInPhrase, encodedWord, codeword, startIdx):
        phraseSeenAtIdx = self.lengthwisePhrasesList[hashTablePos].get(
            word, None)
        if phraseSeenAtIdx:
            if not codeword:
                encodedWord, codeword = self.getCodeword(word, numSymsInPhrase)
            if self.hashVals.get(codeword, None):
                self.noSuffixCollisions += 1
                return None
                # ignore generating additional suffix byte for the ease of decompression
                wordLen, encodedSuffix = compress_words.Compressor().getHashVal(
                    word, numSymsInPhrase+1)
                if encodedSuffix == 0:
                    self.noSuffixCollisions += 1
                    return None
                encodedWord.append(encodedSuffix)
                sfxHex = str(hex(encodedSuffix))
                codeword += sfxHex[2:]
                if len(codeword) > 8 or len(encodedWord) >= wLen:
                    self.collisionsCWLenExceeded += 1
                    return None
                self.checkPhraseFreq(
                    wLen, hashTablePos, word, numSymsInPhrase+1, encodedWord, codeword)
            else:
                if startIdx < phraseSeenAtIdx+numSymsInPhrase:
                    #print(phraseSeenAtIdx, 'phraseSeenAtIdx')
                    #print(startIdx, 'startIdx')
                    #print(wLen, ' wLen->', word, 'overlapping word')
                    return None
                self.codewordHashTableList[hashTablePos][word] = [
                    encodedWord, startIdx]
                self.hashVals[codeword] = word  # +str(numWords)
                return encodedWord
        else:
            self.lengthwisePhrasesList[hashTablePos][word] = startIdx
            #print(hashTablePos, ' word added-->', word)
            return None

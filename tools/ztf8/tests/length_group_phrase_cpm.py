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
        self.freqPhraseToCodewordHashTableList = [{}, {}, {}, {}, {}]
        self.lengthwisePhraseList = [{}, {}, {}, {}, {}]
        # C0-C7, C8-CF, D0-DF, E0-EF, F0-FF
        self.prefixList = [192, 200, 208, 224, 240]
        self.compressed = bytearray(b'')
        self.cmp = bytearray(b'')
        self.bitmix = [[4, 1, 7, 2, 0, 6, 5, 3], [1, 2, 4, 5, 0, 3, 6, 7], [
            0, 5, 4, 6, 2, 3, 7, 1], [7, 1, 6, 4, 3, 0, 5, 2], [3, 2, 5, 4, 7, 6, 0, 1]]
        self.allHashVals = {}
        self.noSuffixCollisions = [0, 0, 0, 0, 0]
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
                segment = infile.read(1000000)
                if not segment:
                    break
                words = compress_words.Compressor().readSegment(segment)
                self.wordsLen += len(words)
                self.words = words
                # first compress the segment for all the possible 4 word phrases.
                # Once done, only then move ahead with compressing 3 word phrases which could not be
                # compressed as 4 word phrases and continue till 1 word phrase/ plaintext retains.
                self.cmp = self.idPhraseCompress(numWords, words)
                #print(self.cmp, 'compressed')
                # Now, if we give words to compress as phrase of 3 words, we first check the hashtable for a 4 word phrase to be present.
                # If present, we know that it's already compressed as a 4 word phrase and skip those 4 words.
                # Else, we take first 3 words of the phrase and make an entry in the phrase table. Repeat the same for every phrase of 4 words.
                # At any point, we find a phrase of 3 words repeated in the phrase table, create an entry for it and add it in the hash table.
                # Repeat the same by reducing the number of words in phrases till 1.
                for phraseLen in range(numWords-1, 0, -1):
                    self.cmp = self.idSubPhraseCompress(
                        phraseLen, self.cmp)
                    #print(self.cmp, 'self.cmp')
                self.compressed += self.cmp

        print(self.wordsLen, 'words length')
        # print('collisions: collisionsCWLenExceeded -> ',
        #      self.collisionsCWLenExceeded)
        for collisions, i in enumerate(self.noSuffixCollisions):
            print('collisions: ', i, ' --> ',
                  collisions)
        print("freqPhraseToCodewordHashTableList table size")
        for h in self.freqPhraseToCodewordHashTableList:
            print(len(h))
            # for i in h.items():
            #    print(i)
        print("lengthwisePhraseList table size")
        for h in self.lengthwisePhraseList:
            # print(len(h))
            #print("All hashVals")
            # for h in self.allHashVals.items():
            #    print(h)
            #print(self.compressed, 'self.compressed')
        return self.compressed

    def idPhraseCompress(self, phraseLen, words):
        compress = bytearray(b'')
        if not words:
            return compress
        #print(words, 'words')
        wordsLen = len(words)
        index = 0
        while index < wordsLen:
            # remove this check?
            if index+(phraseLen-1) < wordsLen:
                phrase, singleByteSyms = compress_words.Compressor().getPhrase(
                    words, index, phraseLen, wordsLen)
            else:
                for w in words[index::]:
                    compress += bytearray(w, 'utf-8')
                break
            if phrase is None:
                for w in words[index::]:
                    compress += bytearray(w, 'utf-8')
                break
            wLen = len(phrase)
            hashVal = None
            hashTablePos = -1
            hashVal, hashTablePos, startIdx = self.getHashValfromTable(
                wLen, phrase)
            if hashVal:  # and index < (startIdx+phraseLen+singleByteSyms):
                if self.printPlainText:
                    compress += bytearray(phrase, 'utf-8')
                if self.printWordCodeword:
                    # compressed += bytearray(phrase+'-41>', 'utf-8')
                    compress += bytearray(str(hashVal), 'utf-8')
                if not self.hideHashVal:
                    compress += hashVal
                index += (singleByteSyms + phraseLen)
            else:
                if wLen < 3:
                    compress += bytearray(phrase, 'utf-8')
                    index += (singleByteSyms + phraseLen)
                elif 3 <= wLen <= 32:
                    #print(phrase, '-->phrase')
                    encodedWord = self.checkPhraseFreq(
                        wLen, hashTablePos, phrase, phraseLen+singleByteSyms, None, index)
                    if encodedWord:
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
                # print(plaintext, 'plaintext')
            else:
                codeWord = bytearray(b'')
                if compressed[index] >= 192 and compressed[index] <= 223 and index+1 < length:
                    if compressed[index+1] >= 128:
                        plaintext.append(compressed[index])
                        plaintext.append(compressed[index+1])
                    else:
                        codeWord.append(compressed[index])
                        codeWord.append(compressed[index+1])
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
                    index += 4
                else:
                    codeWord.append(compressed[index])
                    codeWord.append(compressed[index+1])
                    codeWord.append(compressed[index+2])
                    codeWord.append(compressed[index+3])
                    index += 4
                if codeWord:
                    # print(codeWord, 'codeWord')
                    # print(plaintext, 'plaintext')
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
            hashVal, startIdx = self.freqPhraseToCodewordHashTableList[0].get(
                word, [None, None])
        elif wLen == 4:
            hashTablePos = 1
            hashVal, startIdx = self.freqPhraseToCodewordHashTableList[1].get(
                word, [None, None])
        elif wLen > 4 and wLen <= 8:
            hashTablePos = 2
            hashVal, startIdx = self.freqPhraseToCodewordHashTableList[2].get(
                word, [None, None])
        elif wLen > 8 and wLen <= 16:
            hashTablePos = 3
            hashVal, startIdx = self.freqPhraseToCodewordHashTableList[3].get(
                word, [None, None])
        elif wLen > 16 and wLen <= 32:
            hashTablePos = 4
            hashVal, startIdx = self.freqPhraseToCodewordHashTableList[4].get(
                word, [None, None])
        return hashVal, hashTablePos, startIdx

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
        codeword = str(wLen)+codeWordHex
        # print(encodedWord, 'encodedWord')
        return encodedWord, codeword

    # generate codeword for all phrases, if a phrase is repeated previously, use the codeword corresponding to the phrase in compressing
    # (only if the codeword doesn't conflict with another phrase previously seen in the input => needed for decompression to easily identify the phrase corresponding to a codeword)

# generate codeword for every phrase and save in hash table
# if codeword conflict found, pass the phrase compression

    def checkPhraseFreq(self, wLen, hashTablePos, word, numSymsInPhrase, encodedWord, startIdx):
        encodedWord, codeword = self.getCodeword(word, numSymsInPhrase)
        # print(numSymsInPhrase, 'numSymsInPhrase')
        # print(word, '-->', encodedWord)
        phrases = self.allHashVals.get(codeword, None)
        if phrases:
            for phrase in phrases:
                if phrase == word:
                    # print(encodedWord, '<--word-->', word)
                    # check if the word identified is in the overlapping range of its last occurrence
                    phraseSeenAtIdx = self.lengthwisePhraseList[hashTablePos].get(
                        word, None)
                    if phraseSeenAtIdx and startIdx < phraseSeenAtIdx+numSymsInPhrase:
                        # print(phraseSeenAtIdx, 'phraseSeenAtIdx')
                        # print(startIdx, 'startIdx')
                        # print(wLen, ' wLen->', word, 'overlapping word')
                        return None
                    self.freqPhraseToCodewordHashTableList[hashTablePos][word] = [
                        encodedWord, startIdx]
                    # phrases.append(word)
                    #self.allHashVals[codeword] = [phrase]
                    return encodedWord
                elif len(phrase) == len(word):
                    self.noSuffixCollisions[hashTablePos] += 1
                    return
                # branch not needed? No other phrase of different length would have the same codeword
                else:
                    phrases.append(word)
                    self.allHashVals[codeword] = phrases
                    self.lengthwisePhraseList[hashTablePos][word] = startIdx
        else:
            self.allHashVals[codeword] = [word]
            self.lengthwisePhraseList[hashTablePos][word] = startIdx
        return None

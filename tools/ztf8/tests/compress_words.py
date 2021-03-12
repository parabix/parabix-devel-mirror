import codecs
import binascii
import copy
import itertools
import nltk


class Compressor:
    def __init__(self):
        self.name = "ztf"
        # [hashTableLen3 = {}, hashTableLen4 = {}, hashTableLen5_8 = {}, hashTableLen9_16 = {}]
        self.wordsLen = 0
        self.words = []
        self.codewordHashTableList = [{}, {}, {}, {}, {}]
        self.lengthwisePhrasesList = [{}, {}, {}, {}, {}]
        # sub-divide length prefix further into num of words in the phrase of particular length
        # [192, 200, 208, 216, 232]
        self.prefixList = [192, 196, 200, 208, 216]
        self.encodePrefix = 192  # b'\xC0'
        self.encodeSuffix = 0    # b'\x00'
        self.compressed = bytearray(b'')
        self.bitmix = [[4, 1, 7, 2, 0, 6, 5, 3], [1, 2, 4, 5, 0, 3, 6, 7], [
            0, 5, 4, 6, 2, 3, 7, 1], [7, 1, 6, 4, 3, 0, 5, 2], [3, 2, 5, 4, 7, 6, 0, 1]]
        self.hashVals = {}

    def Name(self):
        return 'words'

    def CompressWords(self, words):
        self.words = words
        index = 0
        self.wordsLen = len(words)
        # encode one word or pair of words where consequent symbol is of length 1
        # to avoid generating longer encoded hash for single unicode character
        while index < self.wordsLen:
            word, index = self.getLongestLenPhrase(index)
            # print(word, 'word')
            # if encoded hash for a symbol to be compressed is already calculated,
            # replace the symbol with encoded hash
            wLen = len(word)
            if wLen < 3:
                self.compressed += bytearray(word, 'utf-8')
            hashVal, hashTablePos, notUsed = self.getHashValfromTable(
                wLen, word)

            if hashVal:
                if notUsed:
                    self.compressed += bytearray(word, 'utf-8')
                else:
                    self.compressed += hashVal
            else:
                if 3 <= wLen <= 32:
                    wordLen, encodedSuffix = self.getHashVal(
                        word, 1)
                    encodedPrefix = self.getPrefix(encodedSuffix, wordLen)
                    encodedWord = bytearray(b'')
                    encodedWord.append(encodedPrefix)
                    encodedWord.append(encodedSuffix)
                    # print(encodedWord, 'encodedWord')
                    codeWordHex = str(hex((encodedPrefix << 8) |
                                          encodedSuffix))
                    codeword = str(wLen)+codeWordHex[2:]  # +str(numWords)
                    self.checkPhraseFreq(
                        wLen, hashTablePos, word, 1, None, None)
                self.compressed += bytearray(word, 'utf-8')
        # print(len(words), 'words length')
        # print("hash table size")
        # for h in self.codewordHashTableList:
        #    print(len(h))
        return self.compressed

    def CompressPhrase(self, words, numWords):
        # print(words, 'CompressPhrase words')
        self.words = words
        if numWords == 2:
            wordPhrases = nltk.bigrams(words)
            self.phraseOfTwoWords(words, numWords)
            # need function modification to accept bigrams
            # self.CompressWords(words)
        if numWords == 3:
            # wordPhrases = nltk.trigrams(words)
            # self.prepareHashTables(wordPhrases, numWords)
            self.phraseOfThreeWords(words, numWords, len(words))
        if numWords == 4:
            # wordPhrases = nltk.everygrams(words, 4, 4)
            # self.prepareHashTables(wordPhrases, numWords)
            self.phraseOfFourWords(words, numWords, len(words))
        return self.compressed

    def phraseOfFourWords(self, words, numWords, wordsLen):
        index = 0
        fallBack = []

        while index < wordsLen:
            if index+(numWords-1) < wordsLen:
                word = words[index] + words[index+1] + \
                    words[index+2] + words[index+3]
            else:
                fallBack.extend(words[index::])
                break
            # print(word, 'phraseOfFourWords')
            # if encoded hash for the phrase to be compressed is already calculated,
            # replace the phrase with encoded hash, else fall back to pair or individual word based compression
            wLen = len(word)
            hashVal = None
            hashTablePos = -1
            hashVal, hashTablePos, notUsed = self.getHashValfromTable(
                wLen, word)

            if hashVal:
                if fallBack:
                    # print(fallBack, 'fallBack')
                    self.phraseOfThreeWords(
                        fallBack, numWords-1, len(fallBack))
                    fallBack = []
                # print('compress:', word, 'len:', wLen, 'hashVal:', hashVal)
                self.compressed += hashVal
                index += 4
            else:
                if wLen < 3:  # add the word to fallback and let oneword phrase take care of it?
                    if fallBack:
                        self.phraseOfThreeWords(
                            fallBack, numWords-1, len(fallBack))
                        fallBack = []
                    self.compressed += bytearray(word, 'utf-8')
                    index += 4
                elif wLen <= 32:
                    fallBack.append(words[index])
                    self.checkPhraseFreq(
                        wLen, hashTablePos, word, 1, None, None)
                index += 1
        if fallBack:
            self.phraseOfThreeWords(fallBack, numWords-1, len(fallBack))
            fallBack = []

    def phraseOfThreeWords(self, words, numWords, wordsLen):
        if wordsLen == 2:
            self.phraseOfTwoWords(words, numWords-1)
            return
        if wordsLen == 1:
            self.phraseOfOneWord(words)
            return
        # print(words, 'phraseOfThreeWords')
        index = 0
        fallBack = []

        while index < wordsLen:
            fbWord = words[index]
            if index+(numWords-1) < wordsLen:
                word = fbWord + words[index+1] + words[index+2]
            else:
                fallBack.extend(words[index::])
                break

            # print(word, 'word')
            # if encoded hash for the phrase to be compressed is already calculated,
            # replace the phrase with encoded hash, else fall back to pair or individual word based compression
            wLen = len(word)
            hashVal = None
            hashTablePos = -1
            if wLen < 3:
                if fallBack:
                    self.phraseOfTwoWords(tryFallBack, numWords-1)
                    tryFallBack = []
                self.compressed += bytearray(word, 'utf-8')
                index += 3
            else:
                hashVal, hashTablePos, notUsed = self.getHashValfromTable(
                    wLen, word)

            if hashVal:
                # print('compress:', word, 'len:', wLen, 'hashVal:', hashVal)
                if fallBack:
                    self.phraseOfTwoWords(fallBack, numWords-1)
                    fallBack = []
                # if hashVal found, that means this phrase was already found and been added in the
                # hash table already
                self.compressed += hashVal
                index += 3
            else:
                if 3 <= wLen <= 32:
                    fallBack.append(fbWord)
                    self.checkPhraseFreq(
                        wLen, hashTablePos, word, 1, None, None)
                index += 1
        if fallBack:
            self.phraseOfTwoWords(fallBack, numWords-1)
            fallBack = []

    def phraseOfTwoWords(self, words, numWords):
        if numWords == 1:
            self.phraseOfOneWord(words)
            return
        # print(words, 'phraseOfTwoWords')
        tryFallBack = []
        index = 0
        wordsLen = len(words)
        while index < wordsLen:
            fbWord = words[index]
            if index+1 < wordsLen:
                word = fbWord + words[index+1]
            else:
                tryFallBack.append(fbWord)
                # remaining words/phrases to check
                break
            # if encoded hash for a symbol to be compressed is already calculated,
            # replace the symbol with encoded hash
            wLen = len(word)
            hashVal = None
            hashTablePos = -1
            if wLen < 3:
                # if index+2 >= wordsLen:
                if tryFallBack:
                    self.phraseOfOneWord(tryFallBack)
                    tryFallBack = []
                self.compressed += bytearray(word, 'utf-8')
                index += 2
            else:
                hashVal, hashTablePos, notUsed = self.getHashValfromTable(
                    wLen, word)

            if hashVal:
                # print('compress:', word, 'len:', wLen, 'hashVal:', hashVal)
                if tryFallBack and len(tryFallBack) >= 2:
                    if tryFallBack:
                        self.phraseOfOneWord(tryFallBack)
                        tryFallBack = []
                self.compressed += hashVal
                index += 2
            else:
                if 3 <= wLen <= 32:
                    tryFallBack.append(fbWord)
                    self.checkPhraseFreq(
                        wLen, hashTablePos, word, 1, None, None)  # sfxBytesLen=1
                    index += 1
        if tryFallBack:
            self.phraseOfOneWord(tryFallBack)
            tryFallBack = []

    def phraseOfOneWord(self, words):
        if not words:
            return
        # print(words, 'phraseOfOneWord')
        index = 0
        wordsLen = len(words)
        while index < wordsLen:
            word = words[index]
            index += 1
            # if encoded hash for a symbol to be compressed is already calculated,
            # replace the symbol with encoded hash
            wLen = len(word)
            hashVal = None
            hashTablePos = -1
            if wLen < 3:
                self.compressed += bytearray(word, 'utf-8')
            else:
                hashVal, hashTablePos, notUsed = self.getHashValfromTable(
                    wLen, word)

            if hashVal:
                # print('compress:', word, 'len:', wLen, 'hashVal:', hashVal)
                self.compressed += hashVal
            else:
                if 3 <= wLen <= 32:
                    self.checkPhraseFreq(
                        wLen, hashTablePos, word, 1, None, None)
                    self.compressed += bytearray(word, 'utf-8')

    def getCodeword(self, word):
        wLen = len(word)
        wordLen, encodedSuffix = self.getHashVal(word, 1)
        encodedPrefix = self.getPrefix(encodedSuffix, wordLen)
        encodedWord = bytearray(b'')
        encodedWord.append(encodedPrefix)
        encodedWord.append(encodedSuffix)
        codeWordHex = str(hex((encodedPrefix << 8) | encodedSuffix))
        codeword = str(wLen)+codeWordHex[2:]
        return encodedWord, codeword

    def checkPhraseFreq(self, wLen, hashTablePos, word, sfxBytesLen, encodedWord, codeword):
        if self.lengthwisePhrasesList[hashTablePos].get(word, None):
            if not codeword:
                encodedWord, codeword = self.getCodeword(word)
            if self.hashVals.get(codeword, None):
                wordLen, encodedSuffix = self.getHashVal(
                    word, sfxBytesLen+1)
                if encodedSuffix == 0:
                    return
                encodedWord.append(encodedSuffix)
                sfxHex = str(hex(encodedSuffix))
                codeword += sfxHex[2:]
                if len(codeword) > 8 or len(encodedWord) >= wLen:
                    return
                self.checkPhraseFreq(
                    wLen, hashTablePos, word, sfxBytesLen+1, encodedWord, codeword)
            else:
                self.codewordHashTableList[hashTablePos][word] = [
                    encodedWord, 1]
                # print(word, '-->', encodedWord)
                self.hashVals[codeword] = word  # +str(numWords)
        else:
            self.lengthwisePhrasesList[hashTablePos][word] = 1

    def getHashValfromTable(self, wLen, word):
        hashVal = None
        hashTablePos = -1
        notUsed = 1
        if wLen == 3:
            hashTablePos = 0
            hashVal, notUsed = self.codewordHashTableList[0].get(
                word, [None, None])
            if notUsed:
                self.codewordHashTableList[0][word] = [hashVal, 0]
        elif wLen == 4:
            hashTablePos = 1
            hashVal, notUsed = self.codewordHashTableList[1].get(
                word, [None, None])
            if notUsed:
                self.codewordHashTableList[1][word] = [hashVal, 0]
        elif wLen > 4 and wLen <= 8:
            hashTablePos = 2
            hashVal, notUsed = self.codewordHashTableList[2].get(
                word, [None, None])
            if notUsed:
                self.codewordHashTableList[2][word] = [hashVal, 0]
        elif wLen > 8 and wLen <= 16:
            hashTablePos = 3
            hashVal, notUsed = self.codewordHashTableList[3].get(
                word, [None, None])
            if notUsed:
                self.codewordHashTableList[3][word] = [hashVal, 0]
        elif wLen > 16 and wLen <= 32:
            hashTablePos = 4
            hashVal, notUsed = self.codewordHashTableList[4].get(
                word, [None, None])
            if notUsed:
                self.codewordHashTableList[4][word] = [hashVal, 0]
        return hashVal, hashTablePos, notUsed

    def prepareHashTables(self, wordPhrases, numWords):
        for phrase in wordPhrases:
            phrase = ''.join(phrase)
            phraseLen = len(phrase)
            hashVal = None
            if phraseLen < 3:
                continue
            hashVal, hashTablePos, notUsed = self.getHashValfromTable(
                phraseLen, phrase)
            if not hashVal:
                if hashTablePos != -1:
                    self.checkPhraseFreq(
                        phraseLen, hashTablePos, phrase, 1, None, None)

    def getLongestLenPhrase(self, curIndex):
        word = self.words[curIndex]
        curIndex += 1
        while curIndex < self.wordsLen:
            if len(word) + len(self.words[curIndex]) >= 16:
                break
            word += self.words[curIndex]
            curIndex += 1
        return word, curIndex

    def getPrefix(self, suffix, lgth):
        if lgth == 3:
            pfxBase = self.prefixList[0]
        elif lgth == 4:
            pfxBase = self.prefixList[1]
        elif lgth <= 8:
            pfxBase = self.prefixList[2]
        elif lgth <= 16:
            pfxBase = self.prefixList[3]
        elif lgth <= 32:
            pfxBase = self.prefixList[4]
        # TODO : clean the process of retreiving remaining hash bits!
        remHashBits = suffix * pow(2, 7)
        remHashBits = format(remHashBits, '02x')
        remHashBits = remHashBits[len(remHashBits)-2::]
        remHashBits = bin(int(remHashBits, 16)).zfill(8)[::-1]
        remHashBits = int(remHashBits[::7])
        return (pfxBase+remHashBits)

    def getHashVal(self, word, sfxBytesLen):
        byteToBinary = []
        bitsShuffled = []
        toBinary = []
        wordBytes = bytes(word, 'utf-8')
        for byte in wordBytes:
            # convert to 8-bit binary equivalent of byte
            for bitPos in range(8):
                toBinary.append((byte >> bitPos) & 1)
            byteToBinary = toBinary[::-1]
            for pos, bit in enumerate(self.bitmix[0]):
                b = 0 if byteToBinary[pos] == byteToBinary[bit] else 1
                bitsShuffled.append(b)
            toBinary = []
        for i in range(4):
            prevPos = pow(2, i)
            temp = copy.deepcopy(bitsShuffled)
            for bytePos in range(prevPos, len(word)):
                bitmixIdx = 0
                while bitmixIdx < 8:
                    sIndex = self.bitmix[i+1][bitmixIdx]
                    # print('temp[', bitmixIdx+(8*bytePos), '] ',
                    #      temp[bitmixIdx+(8*bytePos)], 'temp[', 8 *
                    #      (bytePos-prevPos) + sIndex, '] ',
                    #      temp[8*(bytePos-prevPos) + sIndex])
                    b = 0 if temp[bitmixIdx+(8*bytePos)] == temp[8*(bytePos-prevPos) + sIndex] else 1
                    bitsShuffled[bitmixIdx + (8*bytePos)] = b
                    bitmixIdx += 1
        # bitsShuffled = bitsShuffled[::-1]
        hashVal = 0
        lastByte = bitsShuffled[(sfxBytesLen-1)*8: (sfxBytesLen*8)]
        if lastByte:
            hashVal = int("".join(str(x) for x in lastByte), 2)
        # hashVal = format(hashVal, '02x')
        return len(word), hashVal

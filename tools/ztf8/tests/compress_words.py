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
        self.hashTableList = [{}, {}, {}, {}, {}]
        # sub-divide length prefix further into num of words in the phrase of particular length
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
            hashVal = None
            if wLen < 3:
                self.compressed += bytearray(word, 'utf-8')
            elif wLen == 3:
                hashTablePos = 0
                hashVal, notUsed = self.hashTableList[0].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[0][word] = [hashVal, 0]
            elif wLen == 4:
                hashTablePos = 1
                hashVal, notUsed = self.hashTableList[1].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[1][word] = [hashVal, 0]
            elif wLen > 4 and wLen <= 8:
                hashTablePos = 2
                hashVal, notUsed = self.hashTableList[2].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[2][word] = [hashVal, 0]
            elif wLen > 8 and wLen <= 16:
                hashTablePos = 3
                hashVal, notUsed = self.hashTableList[3].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[3][word] = [hashVal, 0]
            elif wLen > 16 and wLen <= 32:
                hashTablePos = 4
                hashVal, notUsed = self.hashTableList[4].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[4][word] = [hashVal, 0]

            if hashVal:
                if notUsed:
                    self.compressed += bytearray(word, 'utf-8')
                else:
                    self.compressed += hashVal
            else:
                if 3 <= wLen <= 32:
                    wordLen, encodedSuffix = self.getHashVal(word)
                    encodedPrefix = self.getPrefix(encodedSuffix, wordLen)
                    encodedWord = bytearray(b'')
                    encodedWord.append(encodedPrefix)
                    encodedWord.append(encodedSuffix)
                    # print(encodedWord, 'encodedWord')
                    if not self.hashVals.get(hex((encodedPrefix << 8) | encodedSuffix)+str(wLen)):
                        self.hashTableList[hashTablePos][word] = [
                            encodedWord, 1]
                        self.hashVals[hex((encodedPrefix << 8)
                                          | encodedSuffix)+str(wLen)] = word
                self.compressed += bytearray(word, 'utf-8')
        return self.compressed

    def CompressPhrase(self, words, numWords):
        #print(words, 'CompressPhrase words')
        if numWords == 2:
            wordPhrases = nltk.bigrams(words)
        if numWords == 3:
            wordPhrases = nltk.trigrams(words)
        self.generatehashTables(wordPhrases)
        self.words = words
        index = 0
        self.wordsLen = len(words)
        fallBack = []

        while index < self.wordsLen:
            if index+2 < self.wordsLen:
                #fbWord = words[index+1] + words[index+2]
                word = words[index] + words[index+1] + words[index+2]
            else:
                fallBack.extend(words[index::])
                # remaining words/phrases to check
                self.fallbackToWordCmp(fallBack, numWords-1)
                fallBack = []
                break
            # if encoded hash for the phrase to be compressed is already calculated,
            # replace the phrase with encoded hash, else fall back to pair or individual word based compression
            wLen = len(word)
            hashVal = None
            if wLen == 3:
                hashTablePos = 0
                hashVal, notUsed = self.hashTableList[0].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[0][word] = [hashVal, 0]
            elif wLen == 4:
                hashTablePos = 1
                hashVal, notUsed = self.hashTableList[1].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[1][word] = [hashVal, 0]
            elif wLen > 4 and wLen <= 8:
                hashTablePos = 2
                hashVal, notUsed = self.hashTableList[2].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[2][word] = [hashVal, 0]
            elif wLen > 8 and wLen <= 16:
                hashTablePos = 3
                hashVal, notUsed = self.hashTableList[3].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[3][word] = [hashVal, 0]
            elif wLen > 16 and wLen <= 32:
                hashTablePos = 4
                hashVal, notUsed = self.hashTableList[4].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[4][word] = [hashVal, 0]

            if hashVal:
                if fallBack and len(fallBack) >= 3:
                    #print(fallBack, 'fallBack')
                    self.fallbackToWordCmp(fallBack, numWords-1)
                    fallBack = []
                if notUsed:
                    fallBack.append(words[index])
                    index += 1
                else:
                    self.compressed += hashVal
                    index += 3
            else:
                if wLen < 3:
                    self.compressed += bytearray(word, 'utf-8')
                    index += 2
                else:
                    fallBack.append(words[index])
                    index += 1
        if fallBack:
            self.fallbackToWordCmp(fallBack, numWords-1)
            fallBack = []
        print(len(words), 'words length')
        print("hash table size")
        for h in self.hashTableList:
            print(len(h))
            # for i in h.items():
            #    print(i)
        return self.compressed

    def generatehashTables(self, wordPhrases):
        for phrase in wordPhrases:
            phrase = ''.join(phrase)
            phraseLen = len(phrase)
            hashVal = None
            if phraseLen < 3:
                continue
            elif phraseLen == 3:
                hashTablePos = 0
                hashVal, notUsed = self.hashTableList[0].get(
                    phrase, [None, None])
            elif phraseLen == 4:
                hashTablePos = 1
                hashVal, notUsed = self.hashTableList[1].get(
                    phrase, [None, None])
            elif phraseLen > 4 and phraseLen <= 8:
                hashTablePos = 2
                hashVal, notUsed = self.hashTableList[2].get(
                    phrase, [None, None])
            elif phraseLen > 8 and phraseLen <= 16:
                hashTablePos = 3
                hashVal, notUsed = self.hashTableList[3].get(
                    phrase, [None, None])
            elif phraseLen > 16 and phraseLen <= 32:
                hashTablePos = 4
                hashVal, notUsed = self.hashTableList[4].get(
                    phrase, [None, None])

            # if hashVal:
                # move these to the final hashTable
            #    continue
            if not hashVal:
                if 3 <= phraseLen <= 32:
                    wordLen, encodedSuffix = self.getHashVal(phrase)
                    encodedPrefix = self.getPrefix(encodedSuffix, wordLen)
                    encodedWord = bytearray(b'')
                    encodedWord.append(encodedPrefix)
                    encodedWord.append(encodedSuffix)
                    # print(encodedWord, 'encodedWord')
                    # self.hashTableList[hashTablePos][phrase] = [encodedWord, notUsed=1]
                    if not self.hashVals.get(hex((encodedPrefix << 8) | encodedSuffix)+str(phraseLen)):
                        # print(hex((encodedPrefix << 8) | encodedSuffix) +
                        #      str(phraseLen), 'key')
                        # print(self.hashVals[
                        #    hex((encodedPrefix << 8) | encodedSuffix)+str(phraseLen)], 'value')
                        #print(phrase, 'phrase to be added', encodedWord)
                        # else:
                        self.hashTableList[hashTablePos][phrase] = [
                            encodedWord, 1]
                        # only encode the longest possible phrases
                        self.hashVals[hex((encodedPrefix << 8)
                                          | encodedSuffix)+str(phraseLen)] = phrase

    def fallbackToWordCmp(self, words, numWords):
        #print(words, 'fallbackToWordCmp')
        if numWords <= 0 or not words:
            return
        if numWords == 2:
            wordPhrasesGen = nltk.bigrams(words)
            wordPhrases = list(wordPhrasesGen)
        if numWords == 1:
            wordPhrases = words
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
                self.fallbackToWordCmp1(tryFallBack)
                tryFallBack = []
                return
            # if encoded hash for a symbol to be compressed is already calculated,
            # replace the symbol with encoded hash
            wLen = len(word)
            hashVal = None
            if wLen < 3:
                if index+2 >= wordsLen:
                    self.fallbackToWordCmp1(tryFallBack)
                    tryFallBack = []
                self.compressed += bytearray(word, 'utf-8')
                # skip a potential phrase after a smaller phrase of len 2
                index += 2
            elif wLen == 3:
                hashTablePos = 0
                hashVal, notUsed = self.hashTableList[0].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[0][word] = [hashVal, 0]
            elif wLen == 4:
                hashTablePos = 1
                hashVal, notUsed = self.hashTableList[1].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[1][word] = [hashVal, 0]
            elif wLen > 4 and wLen <= 8:
                hashTablePos = 2
                hashVal, notUsed = self.hashTableList[2].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[2][word] = [hashVal, 0]
            elif wLen > 8 and wLen <= 16:
                hashTablePos = 3
                hashVal, notUsed = self.hashTableList[3].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[3][word] = [hashVal, 0]
            elif wLen > 16 and wLen <= 32:
                hashTablePos = 4
                hashVal, notUsed = self.hashTableList[4].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[4][word] = [hashVal, 0]

            if hashVal:
                if tryFallBack and len(tryFallBack) >= 2:
                    #print(tryFallBack, 'tryFallBack')
                    self.fallbackToWordCmp1(tryFallBack)
                    tryFallBack = []
                # if notUsed:
                    # tryFallBack.append(fbWord)
                    # self.compressed += bytearray(word, 'utf-8')
                # else:
                self.compressed += hashVal
                index += 2
            else:
                if 3 <= wLen <= 32:
                    #print(word, 'hashVal not found for bigram')
                    # phrase start word and individual word position are same in the list
                    tryFallBack.append(fbWord)
                    wordLen, encodedSuffix = self.getHashVal(word)
                    encodedPrefix = self.getPrefix(encodedSuffix, wordLen)
                    encodedWord = bytearray(b'')
                    encodedWord.append(encodedPrefix)
                    encodedWord.append(encodedSuffix)
                    # print(encodedWord, 'encodedWord')
                    if not self.hashVals.get(hex((encodedPrefix << 8) | encodedSuffix)):
                        # print(hex((encodedPrefix << 8) |
                        #          encodedSuffix)+str(wLen), 'key')
                        # print(self.hashVals[
                        #    hex((encodedPrefix << 8) | encodedSuffix)+str(wLen)], 'value')
                        #print(word, 'word to be added', encodedWord)
                        # else:
                        self.hashTableList[hashTablePos][word] = [
                            encodedWord, 1]
                        self.hashVals[hex((encodedPrefix << 8)
                                          | encodedSuffix)+str(wLen)] = word
                    index += 1
        self.fallbackToWordCmp1(tryFallBack)
        tryFallBack = []

    def fallbackToWordCmp1(self, words):
        #print(words, 'fallbackToWordCmp111111111')
        wordPhrases = words
        tryFallBack = []
        index = 0
        wordsLen = len(words)
        while index < wordsLen:
            word = words[index]
            index += 1
            # if encoded hash for a symbol to be compressed is already calculated,
            # replace the symbol with encoded hash
            wLen = len(word)
            hashVal = None
            if wLen < 3:
                self.compressed += bytearray(word, 'utf-8')
            elif wLen == 3:
                hashTablePos = 0
                hashVal, notUsed = self.hashTableList[0].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[0][word] = [hashVal, 0]
            elif wLen == 4:
                hashTablePos = 1
                hashVal, notUsed = self.hashTableList[1].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[1][word] = [hashVal, 0]
            elif wLen > 4 and wLen <= 8:
                hashTablePos = 2
                hashVal, notUsed = self.hashTableList[2].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[2][word] = [hashVal, 0]
            elif wLen > 8 and wLen <= 16:
                hashTablePos = 3
                hashVal, notUsed = self.hashTableList[3].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[3][word] = [hashVal, 0]
            elif wLen > 16 and wLen <= 32:
                hashTablePos = 4
                hashVal, notUsed = self.hashTableList[4].get(
                    word, [None, None])
                if notUsed:
                    self.hashTableList[4][word] = [hashVal, 0]

            if hashVal:
                # if notUsed:
                #    self.compressed += bytearray(word, 'utf-8')
                # else:
                self.compressed += hashVal
            else:
                if 3 <= wLen <= 32:
                    # phrase start word and individual word position are same in the list
                    wordLen, encodedSuffix = self.getHashVal(word)
                    encodedPrefix = self.getPrefix(encodedSuffix, wordLen)
                    encodedWord = bytearray(b'')
                    encodedWord.append(encodedPrefix)
                    encodedWord.append(encodedSuffix)
                    # print(encodedWord, 'encodedWord')
                    if self.hashVals.get(hex((encodedPrefix << 8) | encodedSuffix)+str(wLen)):
                        tryFallBack.append(word)
                        # print(hex((encodedPrefix << 8) |
                        #          encodedSuffix)+str(wLen), 'key')
                        # print(self.hashVals[
                        #    hex((encodedPrefix << 8) | encodedSuffix)+str(wLen)], 'value')
                        #print(word, 'word to be added ', encodedWord)
                    else:
                        self.hashTableList[hashTablePos][word] = [
                            encodedWord, 1]
                        self.hashVals[hex((encodedPrefix << 8)
                                          | encodedSuffix)+str(wLen)] = word
                    #print(word, 'plaintext single word added')
                    self.compressed += bytearray(word, 'utf-8')

    def pairwise(self, iterable):
        a, b = itertools.tee(iterable)
        next(b, None)
        return itertools.zip_longest(a, b)

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
        # print(remHashBits % 8, 'remHashBits 1')
        remHashBits = format(remHashBits, '02x')
        remHashBits = remHashBits[len(remHashBits)-2::]
        remHashBits = bin(int(remHashBits, 16)).zfill(8)[::-1]
        remHashBits = int(remHashBits[::7])
        # print(remHashBits, 'remHashBits 2')
        return (pfxBase+remHashBits)

    def getHashVal(self, word):
        bits = []
        bitsShuffled = []
        for i, w in enumerate(word):
            dec = ord(w)
            for bitPos in range(8):
                bits.append((dec >> bitPos) & 1)
            for pos, bit in enumerate(self.bitmix[0]):
                # print(bits[pos + 8*i], 'bits[', pos + 8*i, ']')
                # print(bits[bit], 'bits[', bit, ']')
                b = 0 if bits[pos + 8*i] == bits[bit + 8*i] else 1
                bitsShuffled.append(b)
        # print(bitsShuffled)
        for i in range(4):
            prevPos = pow(2, i)
            temp = copy.deepcopy(bitsShuffled)
            for symPos in range(pow(2, i), len(word)):
                bitmixIdx = 0
                while bitmixIdx < 8:
                    sIndex = self.bitmix[i+1][bitmixIdx]
                    # print('temp[', bitmixIdx+(8*symPos), '] ',
                    #      temp[bitmixIdx+(8*symPos)], 'temp[', 8 *
                    #      (symPos-prevPos) + sIndex, '] ',
                    #      temp[8*(symPos-prevPos) + sIndex])
                    b = 0 if temp[bitmixIdx +
                                  (8*symPos)] == temp[8*(symPos-prevPos) + sIndex] else 1
                    bitsShuffled[bitmixIdx + 8*symPos] = b
                    bitmixIdx += 1
            # print(bitsShuffled)
        symLen = len(bitsShuffled)
        bitsShuffled = bitsShuffled[::-1]
        hashVal = 0
        lastByte = bitsShuffled[:8]
        hashVal = int("".join(str(x) for x in lastByte), 2)
        # hashVal = format(hashVal, '02x')
        # print(hashVal)
        return len(word), hashVal

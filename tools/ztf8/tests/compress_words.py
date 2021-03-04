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
        self.prefixList = [192, 196, 200, 208]
        self.encodePrefix = 192  # b'\xC0'
        self.encodeSuffix = 0    # b'\x00'
        self.compressed = bytearray(b'')
        self.bitmix = [[4, 1, 7, 2, 0, 6, 5, 3], [1, 2, 4, 5, 0, 3, 6, 7], [
            0, 5, 4, 6, 2, 3, 7, 1], [7, 1, 6, 4, 3, 0, 5, 2], [3, 2, 5, 4, 7, 6, 0, 1]]

    def Name(self):
        return 'words'

    def CompressPairs(self, words):
        wordPairs = nltk.bigrams(words)
        self.generatehashTables(wordPairs)
        self.words = words
        index = 0
        self.wordsLen = len(words)
        wordEncoding = []
        # encode one word or pair of words where consequent symbol is of length 1
        # to avoid generating longer encoded hash for single unicode character
        while index < self.wordsLen:
            if index+1 < self.wordsLen:
                word = words[index] + words[index+1]
            # if encoded hash for the word pair to be compressed is already calculated,
            # replace the word pair with encoded hash, else fall back to individual word based compression
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
                if wordEncoding:
                    self.fallbackToWordCmp(wordEncoding)
                    wordEncoding = []
                if notUsed:
                    self.compressed += bytearray(word, 'utf-8')
                else:
                    self.compressed += hashVal
                index += 2
            else:
                if len(words[index]) >= 3:
                    wordEncoding.append(words[index])
                else:
                    self.compressed += bytearray(words[index], 'utf-8')
                index += 1
        return self.compressed

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
                if 3 <= wLen <= 16:
                    wordLen, encodedSuffix = self.getHashVal(word)
                    encodedPrefix = self.getPrefix(encodedSuffix, wordLen)
                    encodedWord = bytearray(b'')
                    encodedWord.append(encodedPrefix)
                    encodedWord.append(encodedSuffix)
                    # print(encodedWord, 'encodedWord')
                    self.hashTableList[hashTablePos][word] = [encodedWord, 1]
                self.compressed += bytearray(word, 'utf-8')
        return self.compressed

    def generatehashTables(self, wordPairs):
        for pair in wordPairs:
            pair = ''.join(pair)
            pairLen = len(pair)
            hashVal = None
            if pairLen < 3:
                continue
            elif pairLen == 3:
                hashTablePos = 0
                hashVal, notUsed = self.hashTableList[0].get(
                    pair, [None, None])
            elif pairLen == 4:
                hashTablePos = 1
                hashVal, notUsed = self.hashTableList[1].get(
                    pair, [None, None])
            elif pairLen > 4 and pairLen <= 8:
                hashTablePos = 2
                hashVal, notUsed = self.hashTableList[2].get(
                    pair, [None, None])
            elif pairLen > 8 and pairLen <= 16:
                hashTablePos = 3
                hashVal, notUsed = self.hashTableList[3].get(
                    pair, [None, None])
            elif pairLen > 16 and pairLen <= 32:
                hashTablePos = 4
                hashVal, notUsed = self.hashTableList[4].get(
                    pair, [None, None])

            # if hashVal:
                # move these to the final hashTable
            #    continue
            if not hashVal:
                if 3 <= pairLen <= 16:
                    wordLen, encodedSuffix = self.getHashVal(pair)
                    encodedPrefix = self.getPrefix(encodedSuffix, wordLen)
                    encodedWord = bytearray(b'')
                    encodedWord.append(encodedPrefix)
                    encodedWord.append(encodedSuffix)
                    # print(encodedWord, 'encodedWord')
                    # self.hashTableList[hashTablePos][pair] = [encodedWord, notUsed=1]
                    self.hashTableList[hashTablePos][pair] = [encodedWord, 1]

    def fallbackToWordCmp(self, words):
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
                if notUsed:
                    self.compressed += bytearray(word, 'utf-8')
                else:
                    self.compressed += hashVal
            else:
                if 3 <= wLen <= 16:
                    wordLen, encodedSuffix = self.getHashVal(word)
                    encodedPrefix = self.getPrefix(encodedSuffix, wordLen)
                    encodedWord = bytearray(b'')
                    encodedWord.append(encodedPrefix)
                    encodedWord.append(encodedSuffix)
                    # print(encodedWord, 'encodedWord')
                    self.hashTableList[hashTablePos][word] = [encodedWord, 1]
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

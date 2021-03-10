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
                    self.checkForDuplicateHashVal(
                        encodedPrefix, encodedSuffix, encodedWord, wLen, hashTablePos, word, codeword, 1)  # sfxBytesLen=1
                self.compressed += bytearray(word, 'utf-8')
        #print(len(words), 'words length')
        #print("hash table size")
        # for h in self.hashTableList:
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
            wordPhrases = nltk.trigrams(words)
            self.prepareHashTables(wordPhrases, numWords)
            self.phraseOfThreeWords(words, numWords, len(words))
        if numWords == 4:
            wordPhrases = nltk.everygrams(words, 4, 4)
            self.prepareHashTables(wordPhrases, numWords)
            self.phraseOfFourWords(words, numWords, len(words))
        # for phrase in wordPhrases:
        #    print(phrase)
        #print(len(words), 'words length')
        #print("hash table size")
        # for h in self.hashTableList:
            # print(len(h))
            # for i in h.items():
            #    print(i)
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
            #print(word, 'phraseOfFourWords')
            # if encoded hash for the phrase to be compressed is already calculated,
            # replace the phrase with encoded hash, else fall back to pair or individual word based compression
            wLen = len(word)
            hashVal, hashTablePos, notUsed = self.getHashValfromTable(
                wLen, word)

            if hashVal:
                if fallBack:
                    # print(fallBack, 'fallBack')
                    self.phraseOfThreeWords(
                        fallBack, numWords-1, len(fallBack))
                    fallBack = []
                # bypass the use of notUsed by creating hashTable on the fly
                if notUsed:
                    fallBack.append(words[index])
                    index += 1
                else:
                    #print('compress:', word, '-->', hashVal)
                    self.compressed += hashVal
                    index += 4
            else:
                if wLen < 3:
                    self.compressed += bytearray(word, 'utf-8')
                    index += 4
                else:
                    fallBack.append(words[index])
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
        #print(words, 'phraseOfThreeWords')
        index = 0
        fallBack = []

        while index < wordsLen:
            fbWord = words[index]
            if index+(numWords-1) < wordsLen:
                word = fbWord + words[index+1] + words[index+2]
            else:
                fallBack.extend(words[index::])
                break

            #print(word, 'word')
            # if encoded hash for the phrase to be compressed is already calculated,
            # replace the phrase with encoded hash, else fall back to pair or individual word based compression
            wLen = len(word)
            hashVal = None
            hashTablePos = -1
            if wLen < 3:
                # if index+3 >= wordsLen:
                if fallBack:
                    self.phraseOfTwoWords(tryFallBack, numWords-1)
                    tryFallBack = []
                self.compressed += bytearray(word, 'utf-8')
                index += 3
            else:
                hashVal, hashTablePos, notUsed = self.getHashValfromTable(
                    wLen, word)

            if hashVal:
                #print('compress:', word, '-->', hashVal)
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
                    wordLen, encodedSuffix = self.getHashVal(
                        word, 1)
                    encodedPrefix = self.getPrefix(encodedSuffix, wordLen)
                    encodedWord = bytearray(b'')
                    encodedWord.append(encodedPrefix)
                    encodedWord.append(encodedSuffix)

                    codeWordHex = str(hex((encodedPrefix << 8) |
                                          encodedSuffix))
                    codeword = str(wLen)+codeWordHex[2:]
                    self.checkForDuplicateHashVal(
                        encodedPrefix, encodedSuffix, encodedWord, wLen, hashTablePos, word, numWords, codeword, 1)  # sfxBytesLen=1
                index += 1
        if fallBack:
            self.phraseOfTwoWords(fallBack, numWords-1)
            fallBack = []

    def phraseOfTwoWords(self, words, numWords):
        if numWords == 1:
            self.phraseOfOneWord(words)
            return
        #print(words, 'phraseOfTwoWords')
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
                # skip a potential phrase after a smaller phrase of len 2
                index += 2
            else:
                hashVal, hashTablePos, notUsed = self.getHashValfromTable(
                    wLen, word)

            if hashVal:
                #print('compress:', word, '-->', hashVal)
                if tryFallBack and len(tryFallBack) >= 2:
                    # print(tryFallBack, 'tryFallBack')
                    if tryFallBack:
                        self.phraseOfOneWord(tryFallBack)
                        tryFallBack = []
                # if notUsed:
                    # tryFallBack.append(fbWord)
                    # self.compressed += bytearray(word, 'utf-8')
                # else:
                self.compressed += hashVal
                index += 2
            else:
                if 3 <= wLen <= 32:
                    # print(word, 'hashVal not found for bigram')
                    # phrase start word and individual word position are same in the list
                    tryFallBack.append(fbWord)
                    wordLen, encodedSuffix = self.getHashVal(
                        word, 1)
                    encodedPrefix = self.getPrefix(encodedSuffix, wordLen)
                    encodedWord = bytearray(b'')
                    encodedWord.append(encodedPrefix)
                    encodedWord.append(encodedSuffix)
                    # print(encodedWord, 'encodedWord')
                    codeWordHex = str(hex((encodedPrefix << 8) |
                                          encodedSuffix))
                    codeword = str(wLen)+codeWordHex[2:]
                    self.checkForDuplicateHashVal(
                        encodedPrefix, encodedSuffix, encodedWord, wLen, hashTablePos, word, numWords, codeword, 1)  # sfxBytesLen=1
                    index += 1
        if tryFallBack:
            self.phraseOfOneWord(tryFallBack)
            tryFallBack = []

    def phraseOfOneWord(self, words):
        if not words:
            return
        #print(words, 'phraseOfOneWord')
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
                #print('compress:', word, '-->', hashVal)
                self.compressed += hashVal
            else:
                if 3 <= wLen <= 32:
                    # phrase start word and individual word position are same in the list
                    wordLen, encodedSuffix = self.getHashVal(
                        word, 1)
                    encodedPrefix = self.getPrefix(encodedSuffix, wordLen)
                    encodedWord = bytearray(b'')
                    encodedWord.append(encodedPrefix)
                    encodedWord.append(encodedSuffix)

                    codeWordHex = str(hex((encodedPrefix << 8) |
                                          encodedSuffix))
                    codeword = str(wLen)+codeWordHex[2:]
                    self.checkForDuplicateHashVal(
                        encodedPrefix, encodedSuffix, encodedWord, wLen, hashTablePos, word, 1, codeword, 1)  # sfxBytesLen=1
                    self.compressed += bytearray(word, 'utf-8')

    def checkForDuplicateHashVal(self, prefix, suffix, encodedWord, wLen, hashTablePos, word, numWords, codeword, sfxBytesLen):
        if sfxBytesLen > 1:
            sfxHex = str(hex(suffix))
            codeword += sfxHex[2:]
            #print(codeword, 'codeword')
            if len(codeword) > 10:
                return
        if self.hashVals.get(codeword):
            wordLen, encodedSuffix = self.getHashVal(
                word, sfxBytesLen+1)
            if encodedSuffix == 0:
                return
            encodedWord.append(encodedSuffix)
            if len(encodedWord) >= wLen:
                return
            self.checkForDuplicateHashVal(
                prefix, encodedSuffix, encodedWord, wLen, hashTablePos, word, numWords, codeword, sfxBytesLen+1)
        else:
            self.hashTableList[hashTablePos][word] = [
                encodedWord, 1]
            #print(word, '-->', encodedWord)
            self.hashVals[codeword] = word  # +str(numWords)

    def getHashValfromTable(self, wLen, word):
        hashVal = None
        hashTablePos = -1
        notUsed = 1
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
            # if hashVal:
            # move these to the final hashTable
            #    continue
            if not hashVal:
                if hashTablePos != -1:
                    wordLen, encodedSuffix = self.getHashVal(
                        phrase, 1)
                    encodedPrefix = self.getPrefix(encodedSuffix, wordLen)
                    encodedWord = bytearray(b'')
                    encodedWord.append(encodedPrefix)
                    encodedWord.append(encodedSuffix)
                    # print(encodedWord, 'encodedWord')
                    # self.hashTableList[hashTablePos][phrase] = [encodedWord, notUsed=1]
                    codeWordHex = str(hex((encodedPrefix << 8) |
                                          encodedSuffix))
                    codeword = str(phraseLen)+codeWordHex[2:]
                    self.checkForDuplicateHashVal(
                        encodedPrefix, encodedSuffix, encodedWord, phraseLen, hashTablePos, phrase, numWords, codeword, 1)

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

    def getHashVal(self, word, sfxBytesLen):
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
        lastByte = bitsShuffled[(sfxBytesLen-1)*8: (sfxBytesLen*8)]
        #print(bitsShuffled, 'bitsShuffled')
        #print(lastByte, 'lastByte')
        if lastByte:
            hashVal = int("".join(str(x) for x in lastByte), 2)
        # hashVal = format(hashVal, '02x')
        return len(word), hashVal

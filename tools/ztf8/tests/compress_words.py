import codecs
import binascii
import copy
import itertools
import nltk
import uniseg.wordbreak


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
        # experimental prefix range :
        # lengthGroup 5-8 : 5(D0,D4,D8,DC), 6(D1,D5,D9,DD), 7(D2,D6,DA,DE), 8(D3,D7,DB,DF)
        # lengthGroup 9-16: 9(E0,E8), 10(E1,E9), 11(E2,EA), 12(E3,EB), 13(E4,EC), 14(E5,ED), 15(E6,EE), 16(E7,EF)
        # lengthGroup 17-32: 17(F0), 18(F1), 19(F2), 20(F3), 21(F4), 22(F5), 23(F6), 24(F7), 25(F8), 26(F9), 27(FA), 28(FB), 29(FC), 30(FD), 31(FE), 32(FF)
        self.prefixList = [192, 200, 208, 224, 240]
        self.compressed = bytearray(b'')
        # TODO: perform bit mixing with all of the previous bytes for a phrase.
        # When picking a suffix, choose a byte from same position of bitshuffled for phrases of equal length
        self.bitmix = [[4, 1, 7, 2, 0, 6, 5, 3], [1, 2, 4, 5, 0, 3, 6, 7], [
            0, 5, 4, 6, 2, 3, 7, 1], [7, 1, 6, 4, 3, 0, 5, 2], [3, 2, 5, 4, 7, 6, 0, 1]]
        self.hashVals = {}
        self.collisionsCWLenExceeded = 0
        self.noSuffixCollisions = 0
        self.printPlainText = False
        self.printWordCodeword = False
        self.hideHashVal = False

    def Name(self):
        return 'words'

    def readSegment(self, text):
        words = []
        for word in uniseg.wordbreak.words(text):
            words.append(word)
        return words

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
            hashVal, hashTablePos, startIdx = self.getHashValfromTable(
                wLen, word)

            if hashVal:
                self.compressed += hashVal
            else:
                if 3 <= wLen <= 32:
                    wordLen, encodedSuffix = self.getHashVal(
                        word, wLen)
                    encodedPrefix = self.getPrefix(encodedSuffix[0], wordLen)
                    encodedWord = bytearray(b'')
                    encodedWord.append(encodedPrefix)
                    encodedWord.append(encodedSuffix[0])
                    # print(encodedWord, 'encodedWord')
                    codeWordHex = str(hex((encodedPrefix << 8) |
                                          encodedSuffix[0]))
                    codeword = str(wLen)+codeWordHex[2:]  # +str(numWords)
                    self.checkPhraseFreq(
                        wLen, hashTablePos, word, 1, None, None)
                self.compressed += bytearray(word, 'utf-8')
        # print(len(words), 'words length')
        # print("hash table size")
        # for h in self.codewordHashTableList:
        #    print(len(h))
        return self.compressed

    def CompressPhrase(self, fileName, numWords, printPlainText, printWordCodeword):
        self.printPlainText = printPlainText
        self.printWordCodeword = printWordCodeword
        if self.printPlainText or self.printWordCodeword:
            self.hideHashVal = True
        # print(words, 'CompressPhrase words')
        with open(fileName, 'rt') as infile:
            while True:
                self.collisionsCWLenExceeded = 0
                self.noSuffixCollisions = 0
                segment = infile.read(1000000)
                if not segment:
                    break
                words = self.readSegment(segment)
                self.words = words
                if numWords == 2:
                    wordPhrases = nltk.bigrams(words)
                    self.phraseOfTwoWords(words, numWords)
                    # need function modification to accept bigrams
                    # self.CompressWords(words)
                if numWords == 3:
                    # wordPhrases = nltk.trigrams(words)
                    # self.prepareHashTables(wordPhrases, numWords, 0)
                    self.phraseOfThreeWords(words, numWords, len(words))
                if numWords == 4:
                    # wordPhrases = nltk.everygrams(words, 4, 4)
                    # self.prepareHashTables(wordPhrases, numWords, 0)
                    self.phraseOfFourWords(words, numWords, len(words))
                # for phrase in wordPhrases:
                #    print(phrase)
                print(len(words), 'words length')
                print('collisions: collisionsCWLenExceeded -> ',
                      self.collisionsCWLenExceeded)
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
        infile.close()
        return self.compressed

    def getPhrase(self, words, curIdx, numWords, wordsLen):
        phrase = ""
        singleByteSyms = 0
        numWordsCopy = numWords
        while curIdx < wordsLen and numWords > 0:
            phrase += words[curIdx]
            # exclude space and new line character as words in phrases
            # if len(words[curIdx]) == 1:
            #    singleByteSyms += 1
            if words[curIdx] == ' ' or words[curIdx] == '\n':
                singleByteSyms += 1
            else:
                numWords -= 1
            curIdx += 1
        if numWords == 0:
            return phrase, singleByteSyms
        return None, -1

    def phraseOfFourWords(self, words, numWords, wordsLen):
        index = 0
        fallBack = []

        while index < wordsLen:
            # remove this check?
            if index+(numWords-1) < wordsLen:
                word, singleByteSyms = self.getPhrase(
                    words, index, numWords, wordsLen)
            else:
                fallBack.extend(words[index::])
                break
            # print(word, 'phraseOfFourWords')
            # if encoded hash for the phrase to be compressed is already calculated,
            # replace the phrase with encoded hash, else fall back to pair or individual word based compression
            wLen = len(word)
            hashVal = None
            hashTablePos = -1
            hashVal, hashTablePos, startIdx = self.getHashValfromTable(
                wLen, word)

            if hashVal:
                if fallBack:
                    self.phraseOfThreeWords(
                        fallBack, numWords-1, len(fallBack))
                    fallBack = []
                if self.printPlainText:
                    self.compressed += bytearray(word, 'utf-8')
                if self.printWordCodeword:
                    self.compressed += bytearray(word+'-41>', 'utf-8')
                    self.compressed += bytearray(str(hashVal), 'utf-8')
                if not self.hideHashVal:
                    self.compressed += hashVal
                index += (singleByteSyms + numWords)
            else:
                # this might not be needed if we are creating phrases of 4 words of length > 1
                if wLen < 3:  # add the word to fallback and let oneword phrase take care of it?
                    if fallBack:
                        self.phraseOfThreeWords(
                            fallBack, numWords-1, len(fallBack))
                        fallBack = []
                    self.compressed += bytearray(word, 'utf-8')
                    # to be on the safer side; though this branch would not be taken
                    index += (singleByteSyms + numWords)
                elif 3 <= wLen <= 32:
                    encodedWord = self.checkPhraseFreq(
                        wLen, hashTablePos, word, numWords+singleByteSyms, None, None, index)
                    if encodedWord is not None:
                        if fallBack:
                            self.phraseOfThreeWords(
                                fallBack, numWords-1, len(fallBack))
                            fallBack = []
                        if self.printPlainText:
                            self.compressed += bytearray(word, 'utf-8')
                        if self.printWordCodeword:
                            self.compressed += bytearray(word+'-42>', 'utf-8')
                            self.compressed += bytearray(
                                str(encodedWord), 'utf-8')
                        if not self.hideHashVal:
                            self.compressed += encodedWord
                        index += (singleByteSyms + numWords)
                    else:
                        fallBack.append(words[index])
                        index += 1
                elif wLen > 32:
                    fallBack.append(words[index])
                    index += 1
        if fallBack:
            self.phraseOfThreeWords(fallBack, numWords-1, len(fallBack))
            fallBack = []

    def phraseOfThreeWords(self, words, numWords, wordsLen):
        # print(words, 'phraseOfThreeWords')
        if wordsLen == 2:
            self.phraseOfTwoWords(words, numWords-1)
            return
        if wordsLen == 1:
            self.phraseOfOneWord(words)
            return
        index = 0
        fallBack = []
        while index < wordsLen:
            if index+(numWords-1) < wordsLen:
                word, singleByteSyms = self.getPhrase(
                    words, index, numWords, wordsLen)
            else:
                fallBack.extend(words[index::])
                break
            # if encoded hash for the phrase to be compressed is already calculated,
            # replace the phrase with encoded hash, else fall back to pair or individual word based compression
            wLen = len(word)
            hashVal = None
            hashTablePos = -1
            if wLen < 3:
                if fallBack:
                    self.phraseOfTwoWords(fallBack, numWords-1)
                    fallBack = []
                self.compressed += bytearray(word, 'utf-8')
                index += (singleByteSyms + numWords)
            else:
                hashVal, hashTablePos, startIdx = self.getHashValfromTable(
                    wLen, word)

            if hashVal:
                if fallBack:
                    self.phraseOfTwoWords(fallBack, numWords-1)
                    fallBack = []
                # if hashVal found, that means this phrase was already found and been added in the
                # hash table already
                if self.printPlainText:
                    self.compressed += bytearray(word, 'utf-8')
                if self.printWordCodeword:
                    self.compressed += bytearray(word+'-31>', 'utf-8')
                    self.compressed += bytearray(str(hashVal), 'utf-8')
                if not self.hideHashVal:
                    self.compressed += hashVal
                index += (singleByteSyms + numWords)
            else:
                if 3 <= wLen <= 32:
                    encodedWord = self.checkPhraseFreq(
                        wLen, hashTablePos, word, numWords+singleByteSyms, None, None, index)
                    if encodedWord is not None:
                        if fallBack:
                            self.phraseOfTwoWords(fallBack, numWords-1)
                            fallBack = []
                        if self.printPlainText:
                            self.compressed += bytearray(word, 'utf-8')
                        if self.printWordCodeword:
                            self.compressed += bytearray(word+'-32>', 'utf-8')
                            self.compressed += bytearray(
                                str(encodedWord), 'utf-8')
                        if not self.hideHashVal:
                            self.compressed += encodedWord
                        index += (singleByteSyms + numWords)
                    else:
                        fallBack.append(words[index])
                        index += 1
                elif wLen > 32:
                    fallBack.append(words[index])
                    index += 1
        if fallBack:
            self.phraseOfTwoWords(fallBack, numWords-1)
            fallBack = []

    def phraseOfTwoWords(self, words, numWords):
        # print(words, 'phraseOfTwoWords')
        if numWords == 1:
            self.phraseOfOneWord(words)
            return
        fallBack = []
        index = 0
        wordsLen = len(words)
        while index < wordsLen:
            if index+1 < wordsLen:
                word, singleByteSyms = self.getPhrase(
                    words, index, numWords, wordsLen)
            else:
                fallBack.extend(words[index::])
                # remaining words/phrases to check
                break
            # if encoded hash for a symbol to be compressed is already calculated,
            # replace the symbol with encoded hash
            wLen = len(word)
            hashVal = None
            hashTablePos = -1
            if wLen < 3:
                if fallBack:
                    self.phraseOfOneWord(fallBack)
                    fallBack = []
                self.compressed += bytearray(word, 'utf-8')
                index = index + singleByteSyms + numWords
            else:
                hashVal, hashTablePos, startIdx = self.getHashValfromTable(
                    wLen, word)

            if hashVal:
                if fallBack and len(fallBack) >= 2:
                    if fallBack:
                        self.phraseOfOneWord(fallBack)
                        fallBack = []
                if self.printPlainText:
                    self.compressed += bytearray(word, 'utf-8')
                if self.printWordCodeword:
                    self.compressed += bytearray(word+'-21>', 'utf-8')
                    self.compressed += bytearray(str(hashVal), 'utf-8')
                if not self.hideHashVal:
                    self.compressed += hashVal
                index = index + singleByteSyms + numWords
            else:
                if 3 <= wLen <= 32:
                    # sfxBytesLen=1
                    encodedWord = self.checkPhraseFreq(
                        wLen, hashTablePos, word, numWords+singleByteSyms, None, None, index)
                    if encodedWord is not None:
                        if fallBack:
                            self.phraseOfOneWord(fallBack)
                            fallBack = []
                        if self.printPlainText:
                            self.compressed += bytearray(word, 'utf-8')
                        if self.printWordCodeword:
                            self.compressed += bytearray(word+'-22>', 'utf-8')
                            self.compressed += bytearray(
                                str(encodedWord), 'utf-8')
                        if not self.hideHashVal:
                            self.compressed += encodedWord
                        index = index + singleByteSyms + numWords
                    else:
                        fallBack.append(words[index])
                        index += 1
                elif wLen > 32:
                    fallBack.append(words[index])
                    index += 1
        if fallBack:
            self.phraseOfOneWord(fallBack)
            fallBack = []

    def phraseOfOneWord(self, words):
        if not words:
            return
        # print(words, 'phraseOfOneWord')
        index = 0
        wordsLen = len(words)
        while index < wordsLen:
            word = words[index]
            index += 1
            # TODO : step 2 of single word optimization

            wLen = len(word)
            hashVal = None
            hashTablePos = -1
            if wLen < 3:
                self.compressed += bytearray(word, 'utf-8')
            else:
                hashVal, hashTablePos, startIdx = self.getHashValfromTable(
                    wLen, word)

            if hashVal:
                if self.printPlainText:
                    self.compressed += bytearray(word, 'utf-8')
                if self.printWordCodeword:
                    self.compressed += bytearray(word+'-11>', 'utf-8')
                    self.compressed += bytearray(str(hashVal), 'utf-8')
                if not self.hideHashVal:
                    self.compressed += hashVal
            else:
                if 3 <= wLen <= 32:
                    encodedWord = self.checkPhraseFreq(
                        wLen, hashTablePos, word, 1, None, None, index)
                    if encodedWord is not None:
                        if self.printPlainText:
                            self.compressed += bytearray(word, 'utf-8')
                        if self.printWordCodeword:
                            self.compressed += bytearray(word+'-12>', 'utf-8')
                            self.compressed += bytearray(
                                str(encodedWord), 'utf-8')
                        if not self.hideHashVal:
                            self.compressed += encodedWord
                    else:
                        self.compressed += bytearray(word, 'utf-8')
                elif wLen > 32:
                    self.compressed += bytearray(word, 'utf-8')

    def getCodeword(self, word, numSymsInPhrase):
        wLen = len(word)
        wordLen, encodedSuffix = self.getHashVal(word, numSymsInPhrase)
        # print(word, 'word --> len:', wLen)
        encodedPrefix = self.getPrefix(encodedSuffix[0], wLen)
        # print(encodedPrefix, 'encodedPrefix')
        encodedWord = bytearray(b'')
        encodedWord.append(encodedPrefix)
        finalSuffix = "0"
        for suffix in encodedSuffix:
            encodedWord.append(suffix)
            finalSuffix = str(hex((finalSuffix << 8) | suffix))
        codeWordHex = str(hex((encodedPrefix << 8) | finalSuffix))
        codeword = str(wLen)+codeWordHex[2:]
        return encodedWord, codeword

    def checkPhraseFreq(self, wLen, hashTablePos, word, numSymsInPhrase, encodedWord, codeword, startIdx):
        phraseSeenAtIdx = self.lengthwisePhrasesList[hashTablePos].get(
            word, None)
        if phraseSeenAtIdx:
            if not codeword:
                encodedWord, codeword = self.getCodeword(word, numSymsInPhrase)
            if self.hashVals.get(codeword, None):
                return None
            else:
                if startIdx < phraseSeenAtIdx+numSymsInPhrase:
                    # print(phraseSeenAtIdx, 'phraseSeenAtIdx')
                    # print(startIdx, 'startIdx')
                    # print(wLen, ' wLen->', word, 'overlapping word')
                    return None
                self.codewordHashTableList[hashTablePos][word] = [
                    encodedWord, startIdx]
                self.hashVals[codeword] = word  # +str(numWords)
                return encodedWord
        else:
            self.lengthwisePhrasesList[hashTablePos][word] = startIdx
            # print(hashTablePos, ' word added-->', word)
            return None

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

    def prepareHashTables(self, wordPhrases, numWords, singleByteSyms):
        for phrase in wordPhrases:
            phrase = ''.join(phrase)
            phraseLen = len(phrase)
            hashVal = None
            if phraseLen < 3:
                continue
            hashVal, hashTablePos, startIdx = self.getHashValfromTable(
                phraseLen, phrase)
            if not hashVal:
                if hashTablePos != -1:
                    self.checkPhraseFreq(
                        phraseLen, hashTablePos, phrase, numWords+singleByteSyms, None, None)

    def getLongestLenPhrase(self, curIndex):
        word = self.words[curIndex]
        curIndex += 1
        while curIndex < self.wordsLen:
            if len(word) + len(self.words[curIndex]) >= 16:
                break
            word += self.words[curIndex]
            curIndex += 1
        return word, curIndex

    def getSubPrefix(self, lastNbits, rangeStart, subLen, phraseLen):
        #    0   1   2   3    ---> sublen = num of cols
        #    5   6   7   8    ---> len of phrase
        # 0  D0  D1  D2  D3   ---> pfx based on last 3 bits of suffix; last 3 bits of suffix determine the row num to pick the pfx from.
        # 1  D4  D5  D6  D7
        # 2  D8  D9  DA  DB
        # 3  DC  DD  DE  DF
        # subLen + (phraseLen - rangeStart) + (subLen * lastNbits)
        return (phraseLen - rangeStart) + (subLen * lastNbits)

    def getPrefix(self, suffix, lgth):
        if lgth == 3:
            pfxBase = self.prefixList[0]
            # add last 3 bits of suffix to get the final pfxBase
            binSfx = bin(suffix)[2:]
            pfxBase += int(binSfx[-3:], 2)
        elif lgth == 4:
            pfxBase = self.prefixList[1]
            # add last 3 bits of suffix to get the final pfxBase
            binSfx = bin(suffix)[2:]
            pfxBase += int(binSfx[-3:], 2)
        elif lgth <= 8:
            pfxBase = self.prefixList[2]
            if suffix == 0:
                lastNbits = 0
            else:
                binSfx = bin(suffix)[2:]
                lastNbits = int(binSfx[-2:], 2)
            pfx = self.getSubPrefix(lastNbits, 5, 4, lgth)
            pfxBase += pfx
        elif lgth <= 16:
            pfxBase = self.prefixList[3]
            if suffix == 0:
                lastNbits = 0
            else:
                binSfx = bin(suffix)[2:]
                lastNbits = int(binSfx[-1:], 2)
            pfx = self.getSubPrefix(lastNbits, 9, 8, lgth)
            pfxBase += pfx
        elif lgth <= 32:
            pfxBase = self.prefixList[4]
            pfx = (lgth-17)
            pfxBase += pfx
        return pfxBase

    def getNumSuffixBytes(self, wordLen):
        numSuffix = -1
        if wordLen >= 3 and wordLen <= 8:
            numSuffix = 1
        elif wordLen <= 16:
            numSuffix = 2
        elif wordLen <= 32:
            numSuffix = 3
        return numSuffix

    def getHashVal(self, word, numWordsInPhrase):
        byteToBinary = []
        bitsShuffled = []
        toBinary = []
        wordLen = len(word)
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
        prevPos = 0
        for i in range(1, wordLen):  # range(4)
            prevPos += 1  # pow(2, i)
            temp = copy.deepcopy(bitsShuffled)
            for bytePos in range(prevPos, wordLen):
                bitmixIdx = 0
                while bitmixIdx < 8:
                    sIndex = self.bitmix[i % 4][bitmixIdx]
                    # print('temp[', bitmixIdx+(8*bytePos), '] ',
                    #      temp[bitmixIdx+(8*bytePos)], 'temp[', 8 *
                    #      (bytePos-prevPos) + sIndex, '] ',
                    #      temp[8*(bytePos-prevPos) + sIndex])
                    # Only XOR operation with random bits from prev bytes is being performed.
                    # Incorporating a shift operation of bitsShuffled with prevPos positions
                    # to increase entropy
                    b = 0 if temp[bitmixIdx+(8*bytePos)] == temp[8 *
                                                                 (bytePos-prevPos) + sIndex] else 1
                    bitsShuffled[bitmixIdx + (8*bytePos)] = b
                    bitmixIdx += 1
        hashVal = 0
        #print(bitsShuffled, 'bitsShuffled')
        totalBytes = wordLen
        numSuffix = self.getNumSuffixBytes(wordLen)
        suffixes = []
        sfx = 1
        # switch between numWordsInPhrase and wordLen in below line
        bytePos = wordLen-1
        while bytePos >= 0 and sfx <= numSuffix:
            lastByte = bitsShuffled[bytePos*8: ((bytePos+1)*8)]
            if lastByte:
                hashVal = int("".join(str(x) for x in lastByte), 2)
                #print(hashVal, 'hashVal')
                suffixes.append(hashVal)
                sfx += 1
            bytePos -= 1
        return wordLen, suffixes
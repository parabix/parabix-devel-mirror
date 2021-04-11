import codecs
import binascii
import copy
import uniseg.wordbreak
import compress_words


class Decompressor:
    def __init__(self):
        self.hashTable = {}
        self.codewordHashTableList = [{}, {}, {}, {}, {}]
        self.prefixList = [192, 200, 208, 224, 240]
        self.bitmix = [[4, 1, 7, 2, 0, 6, 5, 3], [1, 2, 4, 5, 0, 3, 6, 7], [
            0, 5, 4, 6, 2, 3, 7, 1], [7, 1, 6, 4, 3, 0, 5, 2], [3, 2, 5, 4, 7, 6, 0, 1]]
        self.hashVals = {}
        self.decompressed = bytearray(b'')
        self.word_list = []
        self.codeWordStartPos = [{}, {}, {}, {}, {}]
        self.phraseToCWTableList = [{}, {}, {}, {}, {}]

    def Name(self):
        return 'text'

    def Decompress(self, textBytes):
        # extract all the plaintext symbols in the compressed text to create the hashTable
        # which is used in decoding the encoded symbols
        index = 0
        length = len(textBytes)
        while index < len(textBytes):
            if textBytes[index] < 192:
                plaintext = bytearray(b'')
                while index < len(textBytes) and textBytes[index] < 192:
                    plaintext.append(textBytes[index])
                    index += 1
            else:
                # TODO: gather all the codewords and plaintext separately; for codewords with same prefix, run the phrase
                # identification and codeword mapping at once.
                # print(plaintext, 'plaintext')
                # identifies correct multi-byte codewords
                codeWord = bytearray(b'')
                cwPfx = 0
                cwIdx = -1
                if textBytes[index] >= 192 and textBytes[index] <= 223 and index+1 < length:
                    if textBytes[index+1] >= 128:
                        plaintext.append(textBytes[index])
                        plaintext.append(textBytes[index+1])
                    else:
                        codeWord.append(textBytes[index])
                        codeWord.append(textBytes[index+1])
                        cwPfx = textBytes[index]
                        cwIdx = index
                    index += 2
                elif textBytes[index] >= 224 and textBytes[index] <= 239 and index+2 < length:
                    if textBytes[index+1] >= 128 and textBytes[index+2] >= 128:
                        plaintext.append(textBytes[index])
                        plaintext.append(textBytes[index+1])
                        plaintext.append(textBytes[index+2])
                    else:
                        codeWord.append(textBytes[index])
                        codeWord.append(textBytes[index+1])
                        codeWord.append(textBytes[index+2])
                        cwPfx = textBytes[index]
                        cwIdx = index
                    index += 3
                elif textBytes[index] >= 240 and textBytes[index] <= 247 and index+3 < length:
                    if textBytes[index+1] >= 128 and textBytes[index+2] >= 128 and textBytes[index+3] >= 128:
                        plaintext.append(textBytes[index])
                        plaintext.append(textBytes[index+1])
                        plaintext.append(textBytes[index+2])
                        plaintext.append(textBytes[index+3])
                    else:
                        codeWord.append(textBytes[index])
                        codeWord.append(textBytes[index+1])
                        codeWord.append(textBytes[index+2])
                        codeWord.append(textBytes[index+3])
                        cwPfx = textBytes[index]
                        cwIdx = index
                    index += 4
                else:
                    codeWord.append(textBytes[index])
                    codeWord.append(textBytes[index+1])
                    codeWord.append(textBytes[index+2])
                    codeWord.append(textBytes[index+3])
                    cwPfx = textBytes[index]
                    cwIdx = index
                    index += 4
                if codeWord:
                    self.decompressed += plaintext
                    cwLen, hashTablePos = self._codeWordLenRange(cwPfx)
                    phraseExpand = bytearray(b'')
                    for idx in range(cwLen):
                        phraseExpand.append(62)
                    self.decompressed += phraseExpand
                    # collect all the start positions of same codewords which correspond to the same decompressed phrase
                    if self.codeWordStartPos[hashTablePos].get(str(codeWord), None):
                        self.codeWordStartPos[hashTablePos][str(
                            codeWord)].append(cwIdx)
                    else:
                        self.codeWordStartPos[hashTablePos][str(codeWord)] = [
                            cwIdx]
                    self.idPhrases(self._Segment(plaintext), 4)
                    plaintext = bytearray(b'')

        # for phraseSict in self.phraseToCWTableList:
            # for cw in phraseSict.items():
                #print(cw, 'phraseeeee')
        # for cwDict in self.codeWordStartPos:
            # for cw in cwDict.items():
                #print(cw, 'cw')
                # for pos in
                # print(self.hashTable, 'hashTable')
        # for h in self.codeWordStartPos:
        #    for i in h.items():
        #        print(i)
        # for h in self.codewordHashTableList:
            # print(len(h))
            # for i in h.items():
            #    print(i)
        # print(''.join(self.word_list), 'self.word_list')
        if plaintext:
            self.decompressed += plaintext
        return self.decompressed

    def getHashTablePos(self, wLen):
        hashTablePos = -1
        if wLen == 3:
            hashTablePos = 0
        elif wLen == 4:
            hashTablePos = 1
        elif wLen > 4 and wLen <= 8:
            hashTablePos = 2
        elif wLen > 8 and wLen <= 16:
            hashTablePos = 3
        elif wLen > 16 and wLen <= 32:
            hashTablePos = 4
        return hashTablePos

    def idPhrases(self, words, numWordsInPhrase):
        # print(words, 'words')
        wordsLen = len(words)
        index = 0
        while index < wordsLen:
            # remove this check?
            if index+(numWordsInPhrase-1) < wordsLen:
                phrase, singleByteSyms = compress_words.Compressor().getPhrase(
                    words, index, numWordsInPhrase, wordsLen)
            else:
                # for w in words[index::]:
                #    self.decompressed += bytearray(w, 'utf-8')
                break
            if len(phrase) > 32:
                index += 1
            else:
                #print(phrase, 'phrase')
                wLen = len(phrase)
                hashTablePos = self.getHashTablePos(wLen)
                hashVal = self.phraseToCWTableList[hashTablePos].get(
                    phrase, None)
                if hashVal is None:
                    codeWord = self._HashCode(
                        phrase, singleByteSyms+numWordsInPhrase)
                    self.phraseToCWTableList[hashTablePos][phrase] = codeWord
                    if str(codeWord) in self.codeWordStartPos[hashTablePos] and str(codeWord) not in self.codewordHashTableList[hashTablePos]:
                        self.codewordHashTableList[hashTablePos][str(
                            codeWord)] = phrase
                index += 1

    def _findHash(self, codeWord, numBytes, hashTablePos):
        # print(''.join(self.word_list), 'self.word_list')
        symIdx = 0
        totalSym = len(self.word_list)
        while symIdx < totalSym:
            newIdx, phrase, numWords, singleByteSyms = self._getPhrase(
                symIdx, numBytes, len(self.word_list))
            symIdx += 1
            if 3 <= len(phrase) <= 32:
                # print(phrase, 'check phrase')
                if codeWord == self._HashCode(phrase, singleByteSyms+numWords):
                    # print(phrase, 'found phrase')
                    self.codewordHashTableList[hashTablePos][str(
                        codeWord)] = phrase
                    # unnecessary?
                    self.word_list.extend(
                        self._Segment(bytearray(phrase, 'utf-8')))
                    #print(phrase, 'found phrase')
                    self.decompressed += bytearray(phrase, 'utf-8')
                    #print(self.decompressed, 'self.decompressed 3')
                    return True
        return False

    def _codeWordLenRange(self, prefix):
        hashTablePos = 0
        if prefix >= 192 and prefix <= 199:
            lenRange = 3
            hashTablePos = 0
        elif prefix >= 200 and prefix <= 207:
            lenRange = 4
            hashTablePos = 1
        elif prefix >= 208 and prefix <= 223:
            subRange = (prefix - 208)
            subRange %= 4
            lenRange = 5+subRange
            hashTablePos = 2
        elif prefix >= 224 and prefix <= 239:
            subRange = (prefix - 224)
            subRange %= 8
            lenRange = 9+subRange
            hashTablePos = 3
        else:
            subRange = prefix - 240
            lenRange = 17+subRange
            hashTablePos = 4
        return lenRange, hashTablePos

    def _Segment(self, plaintext):
        word_list = []
        for word in uniseg.wordbreak.words(plaintext.decode('utf-8')):
            word_list.append(word)
        # print(word_list, 'word_list')
        return word_list

    def _getPhrase(self, curIdx, numBytes, wordsLen):
        # TODO: use rolling hash technique to get next phrase
        phrase = ""
        singleByteSyms = 0
        numWords = 0
        words = self.word_list
        while curIdx < wordsLen and numBytes > 0:
            phrase += words[curIdx]
            # exclude space and new line character as words in phrases
            if words[curIdx] == ' ' or words[curIdx] == '\n':
                singleByteSyms += 1
            else:
                numWords += 1
            numBytes -= len(words[curIdx])
            curIdx += 1
        return curIdx, phrase, numWords, singleByteSyms

    def _HashCode(self, phrase, numWordsInPhrase):
        wordLen, encodedSuffixes = self._getHashVal(phrase, numWordsInPhrase)
        encodedPrefix = 0
        sfxIdx = 0
        encodedPrefix = compress_words.Compressor(
        ).getPrefix(encodedSuffixes[0], wordLen)
        if encodedPrefix <= 247:
            encodedSuffixes[0] = encodedSuffixes[0] % 128

        encodedWord = bytearray(b'')
        encodedWord.append(encodedPrefix)
        finalSuffix = ""
        for suffix in encodedSuffixes:
            encodedWord.append(suffix)
            finalSuffix += str(hex(suffix)[2:])
        codeWordHex = str(hex(encodedPrefix)[2:]) + finalSuffix
        codeword = str(wordLen)+codeWordHex
        # print(encodedWord, 'encodedWord')
        return encodedWord

    def getSubPrefix(self, lastNbits, rangeStart, subLen, phraseLen):
        return (phraseLen - rangeStart) + (subLen * lastNbits)

    def _getPrefix(self, suffix, lgth):
        # if suffix == 0:
        #    return suffix
        if lgth == 3:
            pfxBase = self.prefixList[0]
            # add last 2 bits of suffix to get the final pfxBase
            binSfx = bin(suffix)[2:]
            pfxBase += int(binSfx[-3:], 2)
        elif lgth == 4:
            pfxBase = self.prefixList[1]
            # add last 2 bits of suffix to get the final pfxBase
            binSfx = bin(suffix)[2:]
            pfxBase += int(binSfx[-3:], 2)
        elif lgth <= 8:
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
            pfx = (lgth-17) % 15
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

    def _getHashVal(self, word, numWordsInPhrase):
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
        for i in range(1, len(word)):
            prevPos += 1
            temp = copy.deepcopy(bitsShuffled)
            for bytePos in range(prevPos, len(word)):
                bitmixIdx = 0
                while bitmixIdx < 8:
                    sIndex = self.bitmix[i % 4][bitmixIdx]
                    # print('temp[', bitmixIdx+(8*bytePos), '] ',
                    #      temp[bitmixIdx+(8*bytePos)], 'temp[', 8 *
                    #      (bytePos-prevPos) + sIndex, '] ',
                    #      temp[8*(bytePos-prevPos) + sIndex])
                    b = 0 if temp[bitmixIdx+(8*bytePos)] == temp[8 *
                                                                 (bytePos-prevPos) + sIndex] else 1
                    bitsShuffled[bitmixIdx + (8*bytePos)] = b
                    bitmixIdx += 1
        # bitsShuffled = bitsShuffled[::-1]
        hashVal = 0
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
                # print(hashVal, 'hashVal')
                suffixes.append(hashVal)
                sfx += 1
            bytePos -= 1
        return wordLen, suffixes

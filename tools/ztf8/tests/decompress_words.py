import codecs
import binascii
import copy
import uniseg.wordbreak


class Decompressor:
    def __init__(self):
        self.hashTable = {}
        self.codewordHashTableList = [{}, {}, {}, {}, {}]
        # self.prefixList = [192, 196, 200, 208, 216]
        self.prefixList = [192, 196, 200, 208, 248]
        self.bitmix = [[4, 1, 7, 2, 0, 6, 5, 3], [1, 2, 4, 5, 0, 3, 6, 7], [
            0, 5, 4, 6, 2, 3, 7, 1], [7, 1, 6, 4, 3, 0, 5, 2], [3, 2, 5, 4, 7, 6, 0, 1]]
        self.hashVals = {}
        self.decompressed = bytearray(b'')
        self.word_list = []

    def Name(self):
        return 'text'

    def Decompress(self, textBytes):
        # extract all the plaintext symbols in the compressed text to create the hashTable
        # which is used in decoding the encoded symbols
        index = 0
        # add  error check to confirm any valid UTF-8 sequences are not ignored
        # because they consist of code unit > 0xC0 (192)
        while index < len(textBytes):
            if textBytes[index] < 192:
                plaintext = bytearray(b'')
                while index < len(textBytes) and textBytes[index] < 192:
                    plaintext.append(textBytes[index])
                    index += 1
            else:
                # TODO: gather all the codewords and plaintext separately; for codewords with same prefix, run the phrase
                # identification and codeword mapping at once.
                #print(plaintext, 'plaintext')
                self.decompressed += plaintext
                codeWord = bytearray(b'')
                codeWord.append(textBytes[index])
                codeWord.append(textBytes[index+1])
                # upper bound of prefix may need to be changed
                print(str(codeWord), 'codeWord')
                cwLenRange, hashTablePos = self._codeWordLenRange(
                    textBytes[index])
                #print(cwLenRange, 'cwLenRange-->pfx:', textBytes[index])
                phrase = self.codewordHashTableList[hashTablePos].get(
                    str(codeWord), None)
                if phrase:
                    # if plaintext doesnot contain all the decoded words,
                    # that causes problems for further codewords to find the phrase corresponding to it
                    self.word_list.extend(self._Segment(plaintext))
                    self.decompressed += bytearray(phrase, 'utf-8')
                    self.word_list.extend(
                        self._Segment(bytearray(phrase, 'utf-8')))
                else:
                    self.word_list.extend(self._Segment(plaintext))
                    #print(self.word_list, 'word_list')
                    for phraseLen in cwLenRange:
                        if self._findHash(codeWord, phraseLen, hashTablePos):
                            break
                index += 2
                plaintext = bytearray(b'')
        # print(self.hashTable, 'hashTable')
        for h in self.codewordHashTableList:
            print(len(h))
            # for i in h.items():
            #    print(i)
        if plaintext:
            self.decompressed += plaintext
        return self.decompressed

    def _findHash(self, codeWord, numBytes, hashTablePos):
        symIdx = 0
        totalSym = len(self.word_list)
        while symIdx < totalSym:
            newIdx, phrase, numWords, singleByteSyms = self._getPhrase(
                symIdx, numBytes, len(self.word_list))
            symIdx += 1
            if 3 <= len(phrase) <= 32:
                #print(phrase, 'check phrase')
                if codeWord == self._HashCode(phrase, singleByteSyms+numWords):
                    #print(phrase, 'found phrase')
                    self.codewordHashTableList[hashTablePos][str(
                        codeWord)] = phrase
                    self.word_list.extend(
                        self._Segment(bytearray(phrase, 'utf-8')))
                    self.decompressed += bytearray(phrase, 'utf-8')
                    return True
        return False

    def _codeWordLenRange(self, prefix):
        hashTablePos = 0
        if prefix >= 192 and prefix < 196:
            lenRange = [3, 3]
            hashTablePos = 0
        elif prefix >= 196 and prefix < 200:
            lenRange = [4, 4]
            hashTablePos = 1
        elif prefix >= 200 and prefix < 208:
            subRange = prefix - 200
            lenRange = [5+(subRange % 4), 5+(subRange % 4)]
            hashTablePos = 2
        elif prefix >= 208 and prefix < 224:
            subRange = prefix - 208
            lenRange = [9+(subRange % 8), 9+(subRange % 8)]
            hashTablePos = 3
        else:
            subRange = prefix - 248
            lenRange = [17, 17+((subRange % 8)+8)]
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
        encodedSuffix = self._getHashVal(phrase, numWordsInPhrase)
        encodedPrefix = self._getPrefix(encodedSuffix, len(phrase))
        encodedWord = bytearray(b'')
        encodedWord.append(encodedPrefix)
        encodedWord.append(encodedSuffix)
        return encodedWord

    def _getSubPrefix(self, lastBit, rangeStart, subLen, rangeLen):
        if lastBit == 0:
            return rangeLen - rangeStart
        else:
            return subLen + (rangeLen - rangeStart)

    def _getPrefix(self, suffix, lgth):
        if suffix == 0:
            return suffix
        if lgth == 3:
            pfxBase = self.prefixList[0]
            # add last 2 bits of suffix to get the final pfxBase
            pfxBase += int(bin(suffix)[-2::], 2)
        elif lgth == 4:
            pfxBase = self.prefixList[1]
            # add last 2 bits of suffix to get the final pfxBase
            pfxBase += int(bin(suffix)[-2::], 2)
        elif lgth <= 8:
            pfxBase = self.prefixList[2]
            pfx = self._getSubPrefix(int(bin(suffix)[-1::], 2), 5, 4, lgth)
            pfxBase += pfx
        elif lgth <= 16:
            pfxBase = self.prefixList[3]
            pfx = self._getSubPrefix(int(bin(suffix)[-1::], 2), 9, 8, lgth)
            pfxBase += pfx
        elif lgth <= 32:
            pfxBase = self.prefixList[4]
            pfx = (lgth-17) % 7
            pfxBase += pfx
        return pfxBase

    def _getHashVal(self, word, numWordsInPhrase):
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
        # Exp: fetch the byte of last byte of bitsShuffled to get hashVal
        totalBytes = len(bitsShuffled) // 8
        # End Exp
        lastByte = bitsShuffled[(numWordsInPhrase-1)*8: (numWordsInPhrase*8)]
        if lastByte:
            hashVal = int("".join(str(x) for x in lastByte), 2)
        # hashVal = format(hashVal, '02x')
        return hashVal

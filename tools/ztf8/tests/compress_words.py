import codecs
import binascii
import copy

# the compressed words range from "0xC0 0x00" till "0xD0 0xFF"
# TODO: define upper bound of encoded symbols

# next steps:
# if word x is being compressed, look for all future occurrences of word x withing certain range.
# keeping cursors at the beginning of all occurrences of word x, advance the cursor to find the longest common sequence of words
# among all the occurrences of word x. Encode that sequence of words instead of a single word at a time.


class Compressor:
    def __init__(self):
        self.name = "ztf"
        # [hashTableLen3 = {}, hashTableLen4 = {}, hashTableLen5_8 = {}, hashTableLen9_16 = {}]
        self.hashTableList = [{}, {}, {}, {}]
        self.prefixList = [192, 196, 200, 208]
        self.encodePrefix = 192  # b'\xC0'
        self.encodeSuffix = 0    # b'\x00'
        self.compressed = bytearray(b'')
        self.bitmix = [[4, 1, 7, 2, 0, 6, 5, 3], [1, 2, 4, 5, 0, 3, 6, 7], [
            0, 5, 4, 6, 2, 3, 7, 1], [7, 1, 6, 4, 3, 0, 5, 2], [3, 2, 5, 4, 7, 6, 0, 1]]

    def Name(self):
        return 'words'

    def Compress(self, words):
        index = 0
        # encode one word or pair of words where consequent symbol is of length 1
        # to avoid generating longer encoded hash for single unicode character
        while index < (len(words)):
            if index+2 < len(words):
                extendedLen = len(words[index]) + \
                    len(words[index+1]) + len(words[index+2])
                if (extendedLen > 4 and extendedLen < 16):
                    word = words[index] + words[index+1] + words[index+2]
                    index += 3
                else:
                    word = words[index]
                    index += 1
            else:
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
                hashVal = self.hashTableList[0].get(word, None)
            elif wLen == 4:
                hashTablePos = 1
                hashVal = self.hashTableList[1].get(word, None)
            elif wLen > 4 and wLen <= 8:
                hashTablePos = 2
                hashVal = self.hashTableList[2].get(word, None)
            elif wLen > 8 and wLen <= 16:
                hashTablePos = 3
                hashVal = self.hashTableList[3].get(word, None)

            if hashVal:
                self.compressed += hashVal
            else:
                if 3 <= wLen <= 16:
                    wordLen, encodedSuffix = self.getHashVal(word)
                    encodedPrefix = self.getPrefix(encodedSuffix, wordLen)
                    encodedWord = bytearray(b'')
                    encodedWord.append(encodedPrefix)
                    encodedWord.append(encodedSuffix)
                    # print(encodedWord, 'encodedWord')
                    self.hashTableList[hashTablePos][word] = encodedWord
                self.compressed += bytearray(word, 'utf-8')
            # print(self.hashTable)
        return self.compressed

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
        remHashBits = format(remHashBits, '02x')
        remHashBits = remHashBits[len(remHashBits)-2::]
        remHashBits = bin(int(remHashBits, 16)).zfill(8)[::-1]
        remHashBits = int(remHashBits[::7])
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

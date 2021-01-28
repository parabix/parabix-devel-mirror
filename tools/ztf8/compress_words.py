import codecs
import binascii

# the compressed words range from "0xC0 0x00" till "0xD0 0xFF"
# TODO: define upper bound of encoded symbols

# next steps:
# if word x is being compressed, look for all future occurrences of word x withing certain range.
# keeping cursors at the beginning of all occurrences of word x, advance the cursor to find the longest common sequence of words
# among all the occurrences of word x. Encode that sequence of words instead of a single word at a time.
class Compressor:
    def __init__(self):
        self.name = "ztf"
        self.hashTable = {}
        self.encodePrefix = 192  # b'\xC0'
        self.encodeSuffix = 0    # b'\x00'
        self.compressed = bytearray(b'')

    def Name(self):
        return 'words'

    def Compress(self, words):
        index = 0
        # encode one word or pair of words where consequent symbol is of length 1
        # to avoid generating longer encoded hash for single unicode character
        while index < (len(words)):
            #if index+1 < len(words) and (len(words[index])l == 1 or len(words[index+1]) == 1):
            if words[index] != '\n' and index+1 < len(words) and len(words[index+1]) == 1:
                word = words[index] + words[index+1]
                index += 2
                # TODO: compress any pair of words where first word is of length 1
                # if len(words[index]) == 1:
                #    word = words[index] + words[index+1]
                #    index += 2
            else:
                word = words[index]
                index += 1
            # if encoded hash for a symbol to be compressed is already calculated,
            # replace the symbol with encoded hash
            hashVal = self.hashTable.get(word, None)
            if hashVal:
                self.compressed += hashVal
            # maintain linebreaks in compressed data
            # TODO: consider other Unicode linbreaks too
            elif word == "\n":
                self.compressed += bytearray(word, 'utf-8')
            # if symbol is seen for the first time, encode symbol to create hash
            # and write plaintext symbol to the compressed data
            else:
                self.encodeSuffix += 1
                if self.encodeSuffix == 255:
                    self.encodePrefix += 1
                    self.encodeSuffix = 0
                # encodedWord = bytes([self.encodePrefix, self.encodeSuffix])
                encodedWord = bytearray(b'')
                encodedWord.append(self.encodePrefix)
                encodedWord.append(self.encodeSuffix)
                # print(encodedWord, 'encodedWord')
                self.hashTable[word] = encodedWord
                self.compressed += bytearray(word, 'utf-8')
        print(self.hashTable)
        return self.compressed

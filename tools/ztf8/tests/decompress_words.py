import codecs
import binascii
import uniseg.wordbreak


class Decompressor:
    def __init__(self):
        self.hashTable = {}
        self.decodePrefix = 192  # b'\xC0'
        self.decodeSuffix = 0    # b'\x00'
        self.decompressed = bytearray(b'')

    def Name(self):
        return 'text'

    def Decompress(self, textBytes):
        # extract all the plaintext symbols in the compressed text to create the hashTable
        # which is used in decoding the encoded symbols
        index = 0
        word_list = []
        # add  error check to confirm any valid UTF-8 sequences are not ignored
        # because they consist of code unit > 0xC0 (192)
        while index < len(textBytes):
            if textBytes[index] < 192:
                plaintext = bytearray(b'')
                while index < len(textBytes) and textBytes[index] < 192:
                    plaintext.append(textBytes[index])
                    index += 1
                #print(plaintext, 'plaintext')
                word_list.extend(self._Segment(plaintext))
            else:
                index += 2
        # create a hashTable similar to the compression hashTable with keys and values reversed
        self._HashCode(word_list)
        # refer to the hashTable to decode the encoded symbols
        self._Decode(textBytes)
        #print(self.hashTable, 'hashTable')
        return self.decompressed

    def _Segment(self, plaintext):
        word_list = []
        for word in uniseg.wordbreak.words(plaintext.decode('utf-8')):
            word_list.append(word)
        #print(word_list, 'word_list')
        return word_list

    def _HashCode(self, word_list):
        index = 0
        while index < (len(word_list)):
            if word_list[index] != "\n" and index+1 < len(word_list) and len(word_list[index+1]) == 1:
                word = word_list[index] + word_list[index+1]
                index += 2
            else:
                word = word_list[index]
                index += 1
            if word != "\n":
                self.decodeSuffix += 1
                if self.decodeSuffix == 255:
                    self.decodePrefix += 1
                    self.decodeSuffix = 0
                encodedWord = bytearray(b'')
                encodedWord.append(self.decodePrefix)
                encodedWord.append(self.decodeSuffix)
                # print(encodedWord, 'encodedWord')
                self.hashTable[bytes(encodedWord)] = word

    def _Decode(self, textBytes):
        index = 0
        word = bytearray(b'')
        while index < len(textBytes):
            if textBytes[index] == "\n":
                self.decompressed += bytearray("\n", 'utf-8')
                index += 1
            elif textBytes[index] >= 192:
                if word and word[0] < 192:
                    self.decompressed += bytearray(word)
                    word = bytearray(b'')
                word.append(textBytes[index])
                word.append(textBytes[index+1])
                index += 2
                decmp = self.hashTable.get(bytes(word))
                if decmp:
                    self.decompressed += bytearray(decmp, 'utf-8')
                word = bytearray(b'')
            else:
                word.append(textBytes[index])
                index += 1
        if word:
            self.decompressed += bytearray(word)
# TODO:
# 1. if a sequence of encoded symbols in increasing order of their suffix values are observed,
# they could be replaced by a sequence of values from hashTable
# starting from the first encoded value and calculated offset

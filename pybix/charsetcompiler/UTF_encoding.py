#
# UTF_encoding.py
#

class UTF8:
   name='UTF-8'
   bits=8
   mask=0xFF
   basis_pattern = ["basis_bits.bit_%i"]

class UTF16:
   name='UTF-16'
   bits=16
   mask=0xFFFF

class UTF32:
   name='UTF-32'
   bits=32
   mask=0xFFFFFFFF

class UTF_Encoding_Type:
    def __init__(self, name, bits, mask, default, locked):
	self.name = name
        self.bits = bits
        self.mask = mask
        self.basis_pattern = []
        self.default = default
        self.locked = locked
        self.encoding_error = False
    def __str__(self): return self.name


Encoding = UTF_Encoding_Type


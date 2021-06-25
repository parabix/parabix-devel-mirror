#
#  Character Class Compiler
#
#  Version 0.9 - June 24, 2013
#
#  Copyright (c) 2007-13, Robert D. Cameron
#  Licensed to the public under Open Software License 3.0
#
#  Initial UTF-16 and UTF-32 support added by Dale Denis, June 2013.
#
#  TO DO
#    - add autosensing of 16/32 bit characters from input files
#    - optimization of range logic for 16-bit char sets.
#
#--------------------------------------------------------------------------
#
#  Data types
#  1. Character Set Definitions
#  2. Boolean Expressions
#  3. Code Generator Objects
#
import sys, optparse
import re, binascii, string
import EBCDIC


import charset_def
CharDef = charset_def.CharDef
CharSetDef = charset_def.CharSetDef

import charsets
DefinitionSet = charsets.DefinitionSet

import charset_input_parser

import UTF_encoding
Encoding_Type = UTF_encoding.UTF_Encoding_Type

from pablo_expr import *
from CC_compiler import *


def main():   

    global options
    # Option definition
    option_parser = optparse.OptionParser(usage='python %prog [options] <input file>', version='0.8')
 
    option_parser.add_option('-u', '--character_encoding',
			     dest='character_encoding',
                             type='string',
                             default='Default',
                             help='character encoding; default: UTF-8',
                             )  
    option_parser.add_option('-b', '--basis_pattern', 
    			     dest='basis_pattern',
    			     type='string',
    			     default='basis_bits.bit_%i',
    			     help='pattern for basis bit streams; default: basis_bits.bit_%i',
    			     )
    option_parser.add_option('-l', '--little_endian',
                             dest='little_endian',
                             action='store_true',
                             default=False,
                             help='sets bit numbering of the output to little-endian',
                             )
    option_parser.add_option('-g', '--gensym_pattern', 
    			     dest='gensym_pattern',
    			     type='string',
    			     default='temp%i',
    			     help='pattern for generated temporaries; default: temp%i',
    			     )
    option_parser.add_option('-E', '--EBCDIC', 
    			     dest='use_EBCDIC',
			     action='store_true', 
			     default=False,
    			     help='generate definitions for EBCDIC input',
    			     )
    option_parser.add_option('-p', '--pablo', 
    			     dest='Pablo_skeleton',
			     action='store_true', 
			     default=False,
    			     help='generate pablo skeleton',
    			     )
    option_parser.add_option('-t', '--test', 
    			     dest='test_skeleton',
			     action='store_true', 
			     default=False,
    			     help='generate pablo test skeleton',
    			     )
    options, args = option_parser.parse_args(sys.argv[1:])

    # Set the encoding.
	
    #If the user has entered the encoding type as a command-line argument
    #then the encoding type that is to be used is locked.
    if options.character_encoding == UTF_encoding.UTF32.name:
        UTF_encoding.Encoding = Encoding_Type(options.character_encoding, 
	UTF_encoding.UTF32.bits, UTF_encoding.UTF32.mask, False, True)
    elif options.character_encoding == UTF_encoding.UTF16.name:
        UTF_encoding.Encoding = Encoding_Type(options.character_encoding, 
	UTF_encoding.UTF16.bits, UTF_encoding.UTF16.mask, False, True)
    elif options.character_encoding == UTF_encoding.UTF8.name: 
        UTF_encoding.Encoding = Encoding_Type(options.character_encoding,
        UTF_encoding.UTF8.bits, UTF_encoding.UTF8.mask, False, True)
    elif options.character_encoding == 'Default': 
        UTF_encoding.Encoding = Encoding_Type(UTF_encoding.UTF8.name, 
	UTF_encoding.UTF8.bits, UTF_encoding.UTF8.mask, True, False)
    else:
        print "ERROR: Invalid encoding format."
        return

    # If we have a valid encoding format then set the basis pattern.
    UTF_encoding.Encoding.basis_pattern = string.split(options.basis_pattern, ",")
    if len(UTF_encoding.Encoding.basis_pattern) == 1:
        # If we have the default basis pattern string then adjust it
        # for UTF-16 or UTF-32.  If the encoding is UTF-8 then we will
        # leave it as is.
        if "basis_bits.bit_%i" in UTF_encoding.Encoding.basis_pattern[0]:
	    if UTF_encoding.UTF16.name in UTF_encoding.Encoding.name:
                UTF_encoding.Encoding.basis_pattern[0] = "u16_bit%i"
            elif UTF_encoding.UTF32.name in UTF_encoding.Encoding.name:
                UTF_encoding.Encoding.basis_pattern[0] = "u32_bit%i"
    elif len(UTF_encoding.Encoding.basis_pattern) == 2:
        if UTF_encoding.UTF16.name not in UTF_encoding.Encoding.name:
            print "ERROR: Invalid encoding for the basis pattern variables."
            return
    elif len(UTF_encoding.Encoding.basis_pattern) == 3:
        if UTF_encoding.UTF32.name not in UTF_encoding.Encoding.name:
            print "ERROR: Invalid encoding for the basis pattern variables."
            return
    else:
        print "ERROR: Invalid number of basis pattern variables."
        return
               
           
    # Positional arguments
    if (len(args) == 1):
        # if the specified argument is not in the DefinitionSet, then assume that it's a filename
        if args[0] not in DefinitionSet:
            #define the characters in the list
            defs = charset_input_parser.input_chardef(args[0])
	    if UTF_encoding.Encoding.encoding_error == True:
               if UTF_encoding.Encoding.default:
                  print "ERROR: The input file contains characters with mixed encodings."
               else:
                  print ''.join(["ERROR: The input file contains encodings that are not ",
                                  UTF_encoding.Encoding.name, "."])
               return
	else: defs = DefinitionSet[args[1]]
        if options.use_EBCDIC:
	    defs = EBCDIC.ascii2ebcdic_chardeflist(defs)
        cgo = CC_compiler(UTF_encoding.Encoding, options.gensym_pattern, options.little_endian, '')
	stmts = cgo.chardeflist2py(defs)
	if options.Pablo_skeleton or options.test_skeleton:
          b = string.split(options.basis_pattern, ".")
	  if len(b) == 2: 
            basis_struct = string.upper(b[0][0]) + b[0][1:]
            basis_struct_var = b[0]
            bit_pattern = b[1]
          else: 
            basis_struct = 'Bits'
            basis_stuct_var = 'bits'
            bit_pattern = bit[0]
          struct_defs = "class %s():\n" % basis_struct
          for i in range(8): struct_defs += "\t" + bit_pattern % i + " = 0\n"
          struct_sets = {}
	  for d in defs:
            n = string.split(d.name, ".")
            if len(n) == 2:
              if not n[0] in struct_sets.keys(): struct_sets[n[0]] = []
              struct_sets[n[0]].append(n[1])
          for k in struct_sets.keys():
            struct_defs += "\nclass %s():\n" % (string.upper(k[0])+k[1:])
            for f in struct_sets[k]: struct_defs += "\t" + f + " = 0\n"
          print struct_defs
          params = ", ".join([basis_struct_var] + struct_sets.keys())
          print "\ndef Do_defs(%s):\n" % params
          print stmts
	  if options.test_skeleton:
            for d in defs:
              print '\tprint_register<BitBlock>("%s",%s);\n' % (d.name, d.name)
          print "\ndef main():\n\tDo_defs(%s)\n" % params
        else: 
	   print stmts 
           #fo = open("icgrep_dev/Z_equations.txt", "wb")
           #fo.write(stmts);

	   # Close opend file
	   #fo.close() 
    else:
        option_parser.print_usage()
        

if __name__ == "__main__": main()


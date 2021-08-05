# -*- coding: utf-8 -*-
import charset_def
# Alias
CharDef = charset_def.CharDef
CharSetDef = charset_def.CharSetDef

# 
# UTF-8 bytes occurring in the additional XML 1.1 line break
# characters NEL and LS.

UTF8_XML11_WS_bytes = [CharDef('NEL1', chr(0xC2)),
			CharDef('NEL2', chr(0x85)),
			CharDef('LS1', chr(0xE2)),
			CharDef('LS2', chr(0x80)),
			CharDef('LS3', chr(0xA8))]
	    
DefinitionSet = {}

DefinitionSet['WS_Control_10'] = [CharSetDef('control.x00_x1F', ['\x00-\x1F']),
				  CharDef('control.CR', '\x0D'),
				  CharDef('control.LF', '\x0A'),
				  CharDef('control.HT', '\x09'),
				  CharDef('control.SP', ' '),
				  CharSetDef('lex.WS', ['\x0D', '\x0A', '\x09', ' '])]

DefinitionSet['WS_Control_10_EBCDIC'] = [CharSetDef('Control', ['\x00-\x03', '\x37',
							        '\x2d-\x2f', '\x16',
								'\x0b-\x13', '\x3c-\x3d',
								'\x32', '\x26-\x27',
								'\x18-\x19', '\x3f', 
								'\x1c-\x1f']),
				  	 CharDef('CR', '\x0D'),
				  	 CharDef('LF', '\x25'),
				  	 CharDef('HT', '\x05'),
				  	 CharDef('SP', '\x40')]

DefinitionSet['WS_Control_11_EBCDIC'] = [CharSetDef('Control', ['\x00-\x3f']),
				  	 CharDef('CR', '\x0D'),
				  	 CharDef('LF', '\x25'),
				  	 CharDef('HT', '\x05'),
				  	 CharDef('SP', '\x40'),
					 CharDef('NEL', '\x15')]


DefinitionSet['WS_Control_11_ISO8859'] = [CharSetDef('Control', ['\x00-\x1F', '\x7F', '\x80-\x9F']),
				  	  CharDef('CR', '\x0D'),
				  	  CharDef('LF', '\x0A'),
				  	  CharDef('HT', '\x09'),
				  	  CharDef('SP', ' '),
					  CharDef('NEL', '\x85')]

DefinitionSet['Markup'] = [CharDef('RefStart', '&'),
			   CharDef('Semicolon', ';'),
			   CharDef('LAngle', '<'),
			   CharDef('RAngle', '>'),
			   CharDef('RBracket', ']'),
			   CharDef('Hyphen', '-'),
			   CharDef('QMark', '?'),
			   CharDef('Equals', "="),
			   CharDef('SQuote', "'"),
			   CharDef('DQuote', '"'),
			   CharDef('Slash', '/')
			   ]

DefinitionSet['Digit_and_Hex'] = [CharSetDef('Digit', ['0-9']),
				  CharSetDef('Hex', ['0-9', 'A-F', 'a-f'])]

DefinitionSet['Markup2'] = DefinitionSet['Markup'] + DefinitionSet['Digit_and_Hex']

DefinitionSet['LexicalItems'] = [CharSetDef('MarkupStart', ['<', '&']),
				 CharDef('RAngle', '>'),
				 CharDef('RBracket', ']'),
				 CharDef('Hyphen', '-'),
				 CharDef('QMark', '?'),
				 CharSetDef('Quote', ['"', "'", '<', '&']),
				 CharSetDef('NameFollow', [';', '=', '/', '>', '?', ')', '[', '|', '?', '*', '+', ','])
				 ]
DefinitionSet['LexicalItems2'] = [CharSetDef('MarkupStart', ['<', '&']),
				 CharDef('RAngle', '>'),
				 CharDef('RBracket', ']'),
				 CharDef('Hyphen', '-'),
				 CharDef('QMark', '?'),
				 CharSetDef('Quote', ['"', "'", '<', '&']),
				 CharSetDef('NameFollow', [' -,', '/', ';-?', '[-^', '{-~'])
				 ]

DefinitionSet['Digit_and_Hex'] = [CharSetDef('lex.Digit', ['0-9']),
				  CharSetDef('lex.Hex', ['0-9', 'A-F', 'a-f'])]
				  
DefinitionSet['LexicalItems_with_Digit'] = DefinitionSet['LexicalItems2'] + DefinitionSet['Digit_and_Hex']

DefinitionSet['LI_with_MarkupPass'] = DefinitionSet['LexicalItems_with_Digit'] + [CharSetDef('AmpHashSlash', ['&', '#', '/'])]

#
# Byte classifications in UTF-8 validation.
DefinitionSet['UTF8'] =		[
				CharSetDef('u8.unibyte', ['\x00-\x7F']),
				CharSetDef('u8.prefix', ['\xC0-\xFF']),
				CharSetDef('u8.prefix2', ['\xC0-\xDF']),
				CharSetDef('u8.prefix3', ['\xE0-\xEF']),
				CharSetDef('u8.prefix4', ['\xF0-\xFF']),
				CharSetDef('u8.suffix', ['\x80-\xBF']),
				CharSetDef('u8.badprefix', ['\xC0-\xC1', '\xF5-\xFF']),
				CharDef('u8.xE0', '\xE0'),
				CharDef('u8.xED', '\xED'),
				CharDef('u8.xF0', '\xF0'),
				CharDef('u8.xF4', '\xF4'),
				CharSetDef('u8.xA0_xBF', ['\xA0-\xBF']),
				CharSetDef('u8.x80_x9F', ['\x80-\x9F']),
				CharSetDef('u8.x90_xBF', ['\x90-\xBF']),
				CharSetDef('u8.x80_x8F', ['\x80-\x8F'])  
				]

DefinitionSet['JSON_Control'] = [
				#Control characters
				CharSetDef('Ctrl.x00_x1F', ['\x00-\x1F']),
				CharDef('Ctrl.CR', '\x0D'),
				CharDef('Ctrl.LF', '\x0A'),
				CharDef('Ctrl.HT', '\x09'),
				CharDef('Ctrl.SP', ' '),
				]

DefinitionSet['JSON_Lexical'] = [
				# Object
				CharDef('Lex.LCurlyBrace','{'),
				CharDef('Lex.Colon',':'),
				CharDef('Lex.Comma',','),
				CharDef('Lex.RCurlyBrace','}'),
				
				# Array
				CharDef('Lex.LSquareBracket','['),
				CharDef('Lex.RSquareBracket',']'),
				
				# Number
				CharDef('Lex.Minus', '-'),
				CharDef('Lex.Zero', '0'),
				CharSetDef('Lex.Digit1_9', ['1-9']),
				CharSetDef('Lex.Digit0_9', ['0-9']),
				CharDef('Lex.DecimalPoint', '.'),
				CharSetDef('Lex.Ee', ['E','e']),
				CharSetDef('Lex.PlusMinus', ['+','-']),
				CharSetDef('Lex.Number',['-','0-9','.','E','e','+','-']),
				
				# String
				CharDef('Lex.DQuote','\"'),
				CharDef('Lex.RSolidus','\\'),
				#CharDef('Lex.Solidus','/'),
				#CharDef('Lex.b','b'),
				#CharDef('Lex.f','f'),
				#CharDef('Lex.n','n'),
				#CharDef('Lex.r','r'),
				#CharDef('Lex.t','t'),
				CharDef('Lex.u','u'),
				CharSetDef('Lex.Escape', ['\"','\\','/','b','f','n','r','t','u']),
				CharSetDef('Lex.HexDigit', ['0-9','a-f','A-F']),
								
				# WS
				CharSetDef('Lex.WS', ['\x0D', '\x0A', '\x09', ' ']),
				
				# true
				CharDef('Lex.t','t'),
				CharDef('Lex.r','r'),
				#CharDef('Lex.u','u'),
				CharDef('Lex.e','e'),
				CharSetDef('Lex.True', ['t','r','u','e']),
				
				# false
				CharDef('Lex.f','f'),
				CharDef('Lex.a','a'),
				CharDef('Lex.l','l'),
				CharDef('Lex.s','s'),
				#CharDef('Lex.e','e'),
				CharSetDef('Lex.False', ['f','a','l','s','e']),
				
				# null
				CharDef('Lex.n','n'),
				#CharDef('Lex.u','u'),
				#CharDef('Lex.l','l'),
				#CharDef('Lex.l','l')				
				CharSetDef('Lex.Null', ['n','u','l'])
]


DefinitionSet['JSON'] = DefinitionSet['UTF8'] + DefinitionSet['JSON_Control'] + DefinitionSet['JSON_Lexical'] 

#
# Following definitions for the parabix2 unbounded bitstream prototype
#
# First all the special characters in the XML delimiters.

xml_marks =    [CharDef('lex.RefStart', '&'),
		CharDef('lex.Semicolon', ';'),
		CharDef('lex.LAngle', '<'),
	 	CharDef('lex.RAngle', '>'),
		CharDef('lex.LBracket', '['),
		CharDef('lex.RBracket', ']'),
		CharDef('lex.Exclam', '!'),
		CharDef('lex.QMark', '?'),
		CharDef('lex.Hyphen', '-'),
		CharDef('lex.Equals', "="),
		CharDef('lex.SQuote', "'"),
		CharDef('lex.DQuote', '"'),
		CharDef('lex.Slash', '/'),
		CharDef('lex.Hash', '#'),
		CharDef('lex.x', 'x'),
		CharDef('lex.Colon', ':')
		]
#
# NameFollow: all characters that may legally follow an XML name, plus
# any others that may not be used in names.
# 1. All non-WS characters that may legally follow an XML name.
namefollow = [CharSetDef('lex.NameFollow', [';', '=', '/', '>', '?', ')', '[', '|', '?', '*', '+', ','])]
#
# NameScan: all ASCII characters that may legally occur in a Name,
# plus all UTF-8 prefix and suffix bytes.
namescan = [CharSetDef('lex.NameScan', ['_', '-', '.', '0-:', 'A-Z', 'a-z', '\x80-\xFF'])]

#
namelex = [CharSetDef('lex.ASCII_name_start', ['_', ':', 'A-Z', 'a-z']),
           CharSetDef('lex.ASCII_name_char', ['_', '-', '.', '0-:', 'A-Z', 'a-z']),
	   CharSetDef('lex.NameScan', ['_', '-', '.', '0-:', 'A-Z', 'a-z', '\x80-\xFF'])]

# 
UTF8_BOM_bytes = [CharDef('u8.xEF', '\xEF'), CharDef('u8.xBF', '\xBF'), CharDef('u8.xBE', '\xBE')]

DefinitionSet['Parabix2'] = (xml_marks + namelex + DefinitionSet['WS_Control_10']
                             + DefinitionSet['Digit_and_Hex'] + DefinitionSet['UTF8'] + UTF8_BOM_bytes)

DefinitionSet['CSV'] = [CharDef('BackSlash', '\\'),
                        CharDef('DQuote', '"'),
                        CharDef('SQuote', '\''),
                        CharDef('CR', '\x0D'),
                        CharDef('LF', '\x0A'),
                        CharDef('Comma', ','),
                        CharDef('HT', '\x09'),
                        CharDef('Period', '.')]


# -*- coding: utf-8 -*-
# charset_input_parser.py
# 
# This library contains functions to parse line deliminated charset definitions 
# of the form 'character class name = []' and produces (character class name, character class item list). 
#
import charset_def
import UTF_encoding

debug = False
        
def report_CharSetDef(charset_declaration_list):
	"""
	Diagnostic function. Prints out the character class name and character class item list for each 
  charset definition.
	"""
	print "-----------CharSetDef-----------"
	for element in charset_declaration_list:
		print "name: "  + element[0] + " | items: "  + str(element[1])
	print "-----------CharSetDef-----------"

def split(statement):
	"""
	Splits a charset definition statement on the first occurence of '='
	and returns a two item token list.
	"""
        if len(statement)==0:
            return []
        
        tokens_tuple = statement.partition('=')
        tokens_list = []

        # don't append the delimiter ('=') to the list
        tokens_list.append(tokens_tuple[0])
        tokens_list.append(tokens_tuple[2])
        
	# trim spaces 
	for i in range(0,len(tokens_list)):
            tokens_list[i] = tokens_list[i].strip()
            
	return tokens_list
        

def isValidDeclaration(statement):
	"""
	Validates character set definition statement syntax as 'character class name = [character or range expression]'
	WARNING: This function does not validate the regular expression on the definition statement. The task will be done by genCharSetItems.
	"""
	# split up the string to a list of tokens
	declaration_list = split(statement)
            
	if len(declaration_list) == 2:
	    token = declaration_list[1]

	    # token[0] != '[' or token[-1] != ']' handles the case when the first and the last characters are not a square brackets pair
	    if token[0] != '[' or token[-1] != ']':
                return False
	else:
	    return False
	return True
    
def genCharSetItems(token, items):
	"""
	Generates a list of items from a given token (of type string).
	Returns true if the regular expression is valid.
	Eg. input = "[A-Za-z_]"
	    output = ['a-z', 'A-Z', '_']
	Example of invalid regular expression: [Z-A]
	"""
	token_length = len(token) - 1
	
	# let's process the items and append into a list (items)
	# check from index one to length-1 because we want to skip the square brackets
	index = 1
	
	while index < token_length:

            # range case: we want the pattern of a-b where a is not '-' and a <= b
	    if index+2 < token_length and token [index+1] == '-':
                    
                # allows range whose starting point is '-' if it is declared at the beginning of the list
                if (token [index] == '-' and index == 1) or token [index] != '-':
                   if isValidCharacterRange(token[index], token[index+2]):
                      items.append(token[index:index+3])
                      index += 3
                         
                   else:
                      print "Invalid range: " + token[index:index+3] + ", starting point is greater than ending point."
                      return False
                      
                else:
                   print "Invalid range: " + token[index:index+3] + ", starting point of a range cannot be '-' if it is not at the beginning of the list."
                   return False                               

	    else:
		items.append(token[index])
		index += 1

	return True

def isValidCharacterRange(c1, c2):
        """
        Takes two characters: starting point, c1, and ending point, c2.
        Returns true if c1 <= c2, returns false otherwise.
        """
        return ord(c1) <= ord(c2)
        
def parseCharsetInput(string):
        """
        Takes a line of charset declaration as an argument and generates a list of (name, items) pair.
        Returns an empty pair if the charset declared in the file is not valid. 
        """
        if len(string) == 0:
            return

        # split up the string to a list of tokens
        if isValidDeclaration(string):
            tokens_list = split (string)
            # get the items and store the (name, items) pair to the charset_declaration_list
            items = []
            if genCharSetItems(tokens_list[1], items):
               return (tokens_list[0], items)
            else:
               print "Invalid regular expression: " + string
               return ()
        else:
            print "Invalid charset declaration: " + string
            return ()

def processCharsetInput(input_filename):
	"""
	Takes input_filename as an argument and reads the file to generate the charset items and its name.
	Returns a list containing pairs of (name, items) to be passed to the CharSetDef class
	"""
	# get input from file and read it
	input_handle = open(input_filename, 'r')
	string = input_handle.readline()
	
	# This list contains pairs of (name, items) to be passed to the CharSetDef class
	charset_declaration_list = []

	# read per line
	while (string != ""):

		#Before we encode lets check to ensure that we are being presented with
                #characters encoded with the encoding that we expect.

                #If the input file contains a character that has been explicitly encoded as UTF-8
                if string.find(r'\x') > -1:
                    #The default encoding is UTF8 so if the encoding isn't UTF8 then
                    #we know that the encoding is locked to another encoding type.
                    if UTF_encoding.Encoding.name != UTF_encoding.UTF8.name:
     		   	UTF_encoding.Encoding.encoding_error = True
                    else:
                        UTF_encoding.Encoding.locked = True

 		#If the input file contains a character that has been explicitly encoded as UTF-16
                if string.find(r'\u') > -1:
                    if UTF_encoding.Encoding.locked == False:
                        UTF_encoding.Encoding.name = UTF_encoding.UTF16.name
                        UTF_encoding.Encoding.bits = UTF_encoding.UTF16.bits
                        UTF_encoding.Encoding.mask = UTF_encoding.UTF16.mask
                        UTF_encoding.Encoding.locked = True
                    elif UTF_encoding.Encoding.name != UTF_encoding.UTF16.name:
                        UTF_encoding.Encoding.encoding_error = True
		
		#If the input file contains a character that has been explictly encoded as UTF-32
		if string.find(r'\U') > -1:
                    if UTF_encoding.Encoding.locked == False:
                        UTF_encoding.Encoding.name = UTF_encoding.UTF32.name
                        UTF_encoding.Encoding.bits = UTF_encoding.UTF32.bits
                        UTF_encoding.Encoding.mask = UTF_encoding.UTF32.mask
                        UTF_encoding.Encoding.locked = True
                    elif UTF_encoding.Encoding.name != UTF_encoding.UTF32.name:
                        UTF_encoding.Encoding.encoding_error = True
		
		string = string.decode('unicode_escape')
                
                # '#' indicates comment
                if string[0] != '#':  
		   # check if the last character a new line (\n) character
                   if string[-1] == '\n':
        	      string = string [:-1]
        	   if len(string) != 0:
                      # get the pair of name and items from the declared charsets
                      pair = parseCharsetInput(string)
                      if len(pair) == 2:
                         charset_declaration_list.append(pair)
                else:
                   #comment case
                   pass
                   
                string = input_handle.readline()

	input_handle.close()

	# Check if we parse it properly
	if debug:
		report_CharSetDef(charset_declaration_list)
	return charset_declaration_list

def input_chardef(filename):
    """
    Returns a list of declared CharSet from the declarations in the input file
    """
    defs = []
    charset_declaration_list = processCharsetInput(filename)
    
    for charset_declaration in charset_declaration_list:
        defs.append(charset_def.CharSetDef (charset_declaration[0], charset_declaration[1]))

    return defs


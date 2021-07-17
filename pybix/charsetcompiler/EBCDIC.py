import codecs
from charset_def import *
def ascii2ebcdic_chardeflist(defs):
	encoder = codecs.getencoder('cp037')
	return [xlate_chardef(d, encoder) for d in defs]

def xlate_char_or_range(charset_item, encoder):
    if len(charset_item) == 1:
        return encoder(charset_item[0])
    elif len(charset_item) == 3:
        if charset_item[1] == '-' and ord(charset_item[0]) <= ord(charset_item[2]):
             return Make_Range(ord(charset_item[0]), ord(charset_item[2]))
    print charset_item
    raise BadCharSetItem
	
def xlate_chardef(chardef, encoder):
  if isinstance(chardef, CharDef):
    return CharDef(chardef.name, encoder(chardef.items[0])[0], chardef.complemented)
  else:
    cdefs = []
    for item in chardef.items:
	if len(item) == 1: cdefs.append(encoder(item)[0])
        elif len(item) == 3:
	  for v in range(ord(item[0]), ord(item[-1])+1):
	    cdefs.append(encoder(chr(v))[0])
	else: raise BadCharSetItem
    return CharSetDef(chardef.name, cdefs, chardef.complemented)



#
#  Character Class Compiler - callable CharClassCompiler object.
#
import charset_input_parser

import UTF_encoding
Encoding_Type = UTF_encoding.UTF_Encoding_Type

from pablo_expr import *

class CC_compiler():
    def __init__(self, encoding, gensym_pattern, little_endian_bit_numbering = True, typedecl = 'BitBlock '):
        self.mEncoding = encoding
        self.little_endian = little_endian_bit_numbering
        self.gensym_template = gensym_pattern
        self.gensym_counter = 0
        self.generated_code = []
        self.common_expression_map = {}
        self.typedecl = typedecl
        predeclared = [self.bit_var(i) for i in range(0, self.mEncoding.bits)]
        for sym in predeclared: self.common_expression_map[sym] = sym
        self.canonical_sym_map = {}

    def add_common_expressions(self, enclosing_cgo):
        for expr in enclosing_cgo.common_expression_map.keys():
            self.common_expression_map[expr] = enclosing_cgo.common_expression_map[expr]
        for sym in enclosing_cgo.canonical_sym_map.keys():
            self.canonical_sym_map[sym] = enclosing_cgo.canonical_sym_map[sym]

    def bit_var(self, n):

            if len(self.mEncoding.basis_pattern) == 1:
                return self.mEncoding.basis_pattern[0] % n
            
            if self.mEncoding.name == UTF_encoding.UTF16.name:
                if options.little_endian == True:
                    if n >= 8:
                        return self.mEncoding.basis_pattern[0] % (n - 8)
                    else:
                        return self.mEncoding.basis_pattern[1] % n
                else:
                    if n <= 7:
                        return self.mEncoding.basis_pattern[0] % n
                    else:
                        return self.mEncoding.basis_pattern[1] % (n - 8)

            if self.mEncoding.name == UTF_encoding.UTF32.name:
                if options.little_endian == True:
                    if n >= 21:
                        return "unused_bit%i" % (n - 21)
                    elif n < 21 and n >= 16:
                        return self.mEncoding.basis_pattern[0] % (n - 16)
                    elif n < 16 and n >= 8:
                        return self.mEncoding.basis_pattern[1] % (n - 8)
                    elif n < 8:
                        return self.mEncoding.basis_pattern[2] % n
                else:
                    if n <= 10:
                        return "unused_bit%i" % n
                    elif n > 10 and n <= 15:
                        return self.mEncoding.basis_pattern[0] % (n - 8)
                    elif n > 15 and n <= 23:
                        return self.mEncoding.basis_pattern[1] % (n - 16)
                    elif n > 23:
                        return self.mEncoding.basis_pattern[2] % (n - 24)

    def make_bitv(self, n):                 
            if self.little_endian == True:
                return Var(self.bit_var(n))
            else:
                return Var(self.bit_var((self.mEncoding.bits - 1) -n)) 
                
    def bit_pattern_expr(self, pattern, selected_bits):
            if selected_bits == 0: return TrueLiteral()
            bit_terms = []
            bit_no = 0
            while selected_bits:
              test_bit = 1 << bit_no
              if selected_bits & test_bit:
                if (pattern & test_bit) == 0:
                    bit_terms = [make_not(self.make_bitv(bit_no))] + bit_terms
                else: bit_terms = [self.make_bitv(bit_no)] + bit_terms
              else: bit_terms = [TrueLiteral()] + bit_terms
              # Appending TrueLiteral() for nonselected bits is intended
              # to keep consistent grouping of variables in the next loop.
              selected_bits &= ~test_bit
              bit_no += 1
            while len(bit_terms) > 1:
                new_terms = []
                for i in range(0, len(bit_terms)/ 2):
                    new_terms.append(make_and(bit_terms[2*i], bit_terms[2*i+1]))
                if len(bit_terms) % 2 == 1:
                    new_terms.append(bit_terms[-1])
                bit_terms = new_terms           
            return bit_terms[0]
    
    def char_test_expr(self, chval):
            return self.bit_pattern_expr(chval, self.mEncoding.mask)  

    def GE_Range(self, N, n):
            if N == 0: return TrueLiteral()
            elif N % 2 == 0 and (n >> (N - 2)) == 0:
                return make_or(make_or(self.make_bitv(N-1), self.make_bitv(N-2)),
                                self.GE_Range(N - 2, n))
            elif N % 2 == 0 and (n >> (N - 2)) == 3:   # >= 11xxxx 
                return make_and(make_and(self.make_bitv(N-1), self.make_bitv(N-2)),
                                self.GE_Range(N - 2, n - (3 << (N-2))))
            elif N >= 1:
                hi_bit = n & (1 << (N-1))
                lo_bits = n - hi_bit
                lo_range = self.GE_Range(N-1, lo_bits)
                if hi_bit == 0:
                    # If the hi_bit of n is not set, then whenever the corresponding bit
                    # is set in the target, the target will certainly be >=.  Otherwise,
                    # the value of GE_range(N-1, lo_bits) is required.
                    return make_or(self.make_bitv(N-1), lo_range)
                else: 
                    # If the hi_bit of n is set, then the corresponding bit must be set
                    # in the target for >= and GE_range(N-1, lo_bits) must also be true.
                    return make_and(self.make_bitv(N-1), lo_range)

    def LE_Range(self, N, n):
            # If an N-bit pattern is all ones, then it is always
            # true that any n-bit value is LE this pattern.
            # Handling this as a special case avoids an overflow
            # issue with n+1 requiring more than N bits.
            if n+1 == 2 ** N:
                return TrueLiteral()
            else:
                return make_not(self.GE_Range(N, n+1))

    def Make_Range(self, n1, n2):  # require n2 >= n1
            diff_bits = n1 ^ n2
            diff_count = 0
            while diff_bits > 0:
                diff_count += 1
                diff_bits >>= 1
            if n2 < n1 or diff_count > self.mEncoding.bits: raise Exception("Bad range: (%x, %x)." % (n1, n2))
            mask = 2**(diff_count) - 1
            common = self.bit_pattern_expr(n1 & ~mask, self.mEncoding.mask^mask)    
            if diff_count == 0: return common
            mask = 2**(diff_count-1) - 1
            lo_test = self.GE_Range(diff_count-1, n1 & mask)
            hi_test = self.LE_Range(diff_count-1, n2 & mask)
            return make_and(common, make_sel(self.make_bitv(diff_count-1), hi_test, lo_test))


    def char_or_range_expr(self, charset_item):
            (lo, hi) = charset_item
            if lo == hi:
                return self.char_test_expr(lo)
            else:
                return self.Make_Range(lo, hi)

    def charset_expr(self, chardef):
            if chardef.items == []: return FalseLiteral()
            if len(chardef.items) > 2:
                combine = True
                #If all of the charset items are single codepoints
                #such that X0 == Y0, X1 == Y1 etc.
                for item in chardef.items:
                    if item[0] != item[1]:
                        combine = False
                        break
                if combine == True:
                    #If charset items are all of the form X1 = X0 + 2.
                    for i in range(len(chardef.items) - 1):
                        curr_item = chardef.items[i]
                        next_item = chardef.items[i+1]
                        if curr_item[0] != next_item[0] - 2:
                            combine = False
                            break
                if combine == True:
                    lo = chardef.items[0][0]
                    hi = chardef.items[-1][0]
                    print "Combined odd/even range %x-%x" % (lo, hi)
                    #
                    if lo & 1 == 1: return make_and(self.char_or_range_expr((lo&~1, hi)), self.make_bitv(0))
                    else: return make_and(self.char_or_range_expr((lo, hi|1)), make_not(self.make_bitv(0)))
            e1 = self.char_or_range_expr(chardef.items[0])
            for i in range(1, len(chardef.items)):   
                e1 = make_or(e1, self.char_or_range_expr(chardef.items[i]))
            if chardef.complemented: return make_not(e1)
            else: return e1

    def add_assignment(self, varname, expr_string):
        if self.common_expression_map.has_key(expr_string):
            assigned = self.common_expression_map[expr_string]
            if assigned == varname: return
        else:
            self.common_expression_map[expr_string] = varname
            assigned = expr_string    
        self.generated_code.append('\t%s%s = %s\n' % (self.typedecl, varname, assigned))

    # An assignment to a variable name that uniquely specifies the expr
    def add_canonical_assignment(self, canonical_var, expr):
        if not canonical_var in self.canonical_sym_map.keys():
            self.add_assignment(canonical_var, expr)
            self.common_expression_map[expr] = canonical_var       
            self.canonical_sym_map[canonical_var] = expr       

    def add_if_stmt(self, test_expr, generated_subcode):
        test_line = 'if %s:\n' % (self.expr_string_to_variable(self.expr2py(test_expr)))
        self.generated_code.append('\t' + test_line)
        self.generated_code += ['\t' + line for line in generated_subcode]
        self.generated_code.append('\t#end' + test_line)
        

    def expr_string_to_variable(self, expr_string):
        if self.common_expression_map.has_key(expr_string):
            return self.common_expression_map[expr_string]
        else:
            self.gensym_counter += 1                           
            sym = self.gensym_template % self.gensym_counter  
            self.add_assignment(sym, expr_string) 
            return sym

    def showcode(self):
        s = ''
        for stmt in self.generated_code: s += stmt
        return s

    def expr2py(self, expr):
            """Translate a Boolean expression into three-address python code.
            """
            if isinstance(expr, TrueLiteral): return '-1'
            elif isinstance(expr, FalseLiteral): return '0'
            elif isinstance(expr, Var): 
               # This is a hack.
               self.common_expression_map[expr.varname] = expr.varname
               return expr.varname
            elif isinstance(expr, Not):
               e = self.expr_string_to_variable(self.expr2py(expr.operand))
               return '(~%s)' % (e)
            elif isinstance(expr, Or):
               e1 = self.expr_string_to_variable(self.expr2py(expr.operand1))
               e2 = self.expr_string_to_variable(self.expr2py(expr.operand2))
               return '(%s | %s)' % (e1, e2)
            elif isinstance(expr, Xor):
               e1 = self.expr_string_to_variable(self.expr2py(expr.operand1))
               e2 = self.expr_string_to_variable(self.expr2py(expr.operand2))
               return '(%s ^ %s)' % (e1, e2)
            elif isinstance(expr, And):
               if isinstance(expr.operand1, Not):
                   e1 = self.expr_string_to_variable(self.expr2py(expr.operand1.operand))
                   e2 = self.expr_string_to_variable(self.expr2py(expr.operand2))
                   return '(%s &~ %s)' % (e2, e1)
               elif isinstance(expr.operand2, Not):
                   e1 = self.expr_string_to_variable(self.expr2py(expr.operand1))
                   e2 = self.expr_string_to_variable(self.expr2py(expr.operand2.operand))
                   return '(%s &~ %s)' % (e1, e2)
               else:
                   e1 = self.expr_string_to_variable(self.expr2py(expr.operand1))
                   e2 = self.expr_string_to_variable(self.expr2py(expr.operand2))
                   return '(%s & %s)' % (e1, e2)
            elif isinstance(expr, Sel):
               sel = self.expr_string_to_variable(self.expr2py(expr.sel))
               e1 = self.expr_string_to_variable(self.expr2py(expr.true_branch))
               e2 = self.expr_string_to_variable(self.expr2py(expr.false_branch))
               return '((%s & %s)|(~(%s) & %s))' %(sel, e1, sel, e2)
            elif isinstance(expr, Adv):
               e = self.expr_string_to_variable(self.expr2py(expr.operand))
               if expr.offset == 1: return 'Advance(%s)' % (e)
               else: return 'Advance(%s, %i)' % (e, expr.offset)
            else: raise Exception("Bad expression: %s" % repr(expr))

    def chardef2py(self, chardef):
            self.add_assignment(chardef.name, self.expr2py(self.charset_expr(chardef)))

    def chardef_canonical(self, chardef):
            self.add_canonical_assignment(chardef.name, self.expr2py(self.charset_expr(chardef)))
    
    def chardeflist2py(self, chardeflist):
            for d in chardeflist:
                self.chardef2py(d) 
            return self.showcode()



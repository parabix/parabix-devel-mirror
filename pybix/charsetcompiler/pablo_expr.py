#
#  Pablo Expressions:  bitwise Boolean Expressions + Advance.
#
import ast

class BitwiseExpr:
   """The BitwiseExpr class and its subclasses provide a symbolic
      representation of bitwise Boolean expressions.
   """
   pass

class Var(BitwiseExpr):
    def __init__(self, varname):
        self.varname = varname
    def __str__(self): return 'Var("' + self.varname + '")'
    def toAST(self): return ast.Name(id=self.varname, ctx=ast.Load())

class TrueLiteral(BitwiseExpr):
    def __init__(self):
        self.value = True
    def __str__(self): return 'T'
    def toAST(self): return ast.Name(id='True', ctx=ast.Load())

class FalseLiteral(BitwiseExpr):
    def __init__(self):
        self.value = False
    def __str__(self): return 'F'
    def toAST(self): return ast.Name(id='False', ctx=ast.Load())

class Not(BitwiseExpr):
    def __init__(self, expr):
        self.operand = expr
    def __str__(self): return 'Not(%s)' % (self.operand.__str__())
    def toAST(self): return ast.UnaryOp(ast.Not(), self.operand.toAST())

class And(BitwiseExpr):
    def __init__(self, expr1, expr2):
        self.operand1 = expr1
        self.operand2 = expr2
    def __str__(self): return 'And(%s, %s)' % (self.operand1.__str__(), self.operand2.__str__())
    def toAST(self): return ast.BinOp(self.operand1.toAST(), ast.BitAnd(), self.operand2.toAST())

class Or(BitwiseExpr):
    def __init__(self, expr1, expr2):
        self.operand1 = expr1
        self.operand2 = expr2
    def __str__(self): return 'Or(%s, %s)' % (self.operand1.__str__(), self.operand2.__str__())
    def toAST(self): return ast.BinOp(self.operand1.toAST(), ast.BitOr(), self.operand2.toAST())

class Xor(BitwiseExpr):
    def __init__(self, expr1, expr2):
        self.operand1 = expr1
        self.operand2 = expr2
    def __str__(self): return 'Xor(%s, %s)' % (self.operand1.__str__(), self.operand2.__str__())
    def toAST(self): return ast.BinOp(self.operand1.toAST(), ast.BitXor(), self.operand2.toAST())

class Sel(BitwiseExpr):
    def __init__(self, expr1, expr2, expr3):
        self.sel = expr1
        self.true_branch = expr2
        self.false_branch = expr3
    def __str__(self): return 'Sel(%s, %s, %s)' % (self.sel.__str__(), self.true_branch.__str__(), self.false_branch.__str__())
    def toAST(self): return ast.IfExpr(self.sel.toAST(), self.true_branch.toAST(), self.false_branch.toAST())

class Adv(BitwiseExpr):
    def __init__(self, expr, n):
        self.operand = expr
        self.offset = n
    def __str__(self): 
       if self.offset == 1: return 'Advance(%s)' % (self.operand.__str__()) 
       else: return 'Advance(%s, %i)' % (self.operand.__str__(), self.offset)
    def toAST(self): return ast.Call(ast.Attribute(ast.Name('pablo', ast.Load()), 'Advance', ast.Load()), [self.operand.toAST(), ast.Num(self.offset)])

#
# Optimizing Constructors for Boolean Expressions
# - Maintaining Assembler Instruction Form:
#   - All boolean algebraic rules involving true/false applied.
#   - Negations restricted:
#       - no negations within or (DeMorgan's to nand)
#       - at most one negation within and
#

def make_not(expr):
    if isinstance(expr, TrueLiteral):
        return FalseLiteral()
    elif isinstance(expr, FalseLiteral):
        return TrueLiteral()
    elif isinstance(expr, Not):
        return expr.operand
    else: return Not(expr)

def make_and(expr1, expr2):
    if isinstance(expr1, TrueLiteral):
        return expr2
    elif isinstance(expr2, TrueLiteral):
        return expr1
    elif isinstance(expr1, FalseLiteral):
        return FalseLiteral()
    elif isinstance(expr2, FalseLiteral):
        return FalseLiteral()
    elif equal_exprs(expr1, expr2): return expr1
    elif isinstance(expr1, Not):
        if isinstance(expr2, Not):
            return make_not(make_or(expr1.operand, expr2.operand))
        elif equal_exprs(expr1.operand, expr2): return FalseLiteral()
        else: return And(expr1, expr2)
    elif isinstance(expr2, Not):
        if equal_exprs(expr1, expr2.operand): return FalseLiteral()
        else: return And(expr1, expr2)
    else: return And(expr1, expr2)

def make_or(expr1, expr2):
    if isinstance(expr1, FalseLiteral):
        return expr2
    elif isinstance(expr2, FalseLiteral):
        return expr1
    elif isinstance(expr1, TrueLiteral):
        return TrueLiteral()
    elif isinstance(expr2, TrueLiteral):
        return TrueLiteral()
    elif isinstance(expr1, Not):
        return make_not(make_and(expr1.operand, make_not(expr2)))
    elif isinstance(expr2, Not):
        return make_not(make_and(make_not(expr1), expr2.operand))
    elif equal_exprs(expr1, expr2): return expr1
    elif isinstance(expr1, And) and isinstance(expr2, And):
        # These optimizations factor out common components that
        # can occur when sets are formed by union (e.g., union of
        # [a-z] and [A-Z].
        if equal_exprs(expr1.operand1, expr2.operand1):
            return make_and(expr1.operand1, make_or(expr1.operand2, expr2.operand2))
        elif equal_exprs(expr1.operand2, expr2.operand2):
            return make_and(expr1.operand2, make_or(expr1.operand1, expr2.operand1))
        elif equal_exprs(expr1.operand1, expr2.operand2):
            return make_and(expr1.operand1, make_or(expr1.operand2, expr2.operand1))
        elif equal_exprs(expr1.operand2, expr2.operand1):
            return make_and(expr1.operand2, make_or(expr1.operand1, expr2.operand2))
        else: return Or(expr1, expr2)
    else: return Or(expr1, expr2)

def make_sel(if_expr, T_expr, F_expr):
    if isinstance(if_expr, TrueLiteral):
        return T_expr
    elif isinstance(if_expr, FalseLiteral):
        return F_expr
    elif isinstance(T_expr, TrueLiteral):
        # if x then T else y = x or y
        return make_or(if_expr, F_expr)  
    elif isinstance(T_expr, FalseLiteral):
        # if x then F else y = not(x) and y
        return make_and(make_not(if_expr), F_expr)  
    elif isinstance(F_expr, FalseLiteral):
        # if x then y else F = x and y
        return make_and(if_expr, T_expr)  
    elif isinstance(F_expr, TrueLiteral):
        # if x then y else T = not(x) or y
        return make_or(make_not(if_expr), T_expr)
    elif equal_exprs(T_expr, F_expr):
        return T_expr
    elif isinstance(T_expr, Not) and equal_exprs(T_expr.operand, F_expr):
        return make_xor(if_expr, F_expr)
    elif isinstance(F_expr, Not) and equal_exprs(F_expr.operand, T_expr):
        return make_xor(if_expr, F_expr)
    else: return Sel(if_expr, T_expr, F_expr)


def make_xor(expr1, expr2):
    if isinstance(expr1, FalseLiteral):
        return expr2
    elif isinstance(expr2, FalseLiteral):
        return expr1
    elif isinstance(expr1, TrueLiteral):
        return make_not(expr2)
    elif isinstance(expr2, TrueLiteral):
        return make_not(expr1)
    elif isinstance(expr1, Not) and isinstance(expr2, Not):
        return make_xor(expr1.operand, expr2.operand)
    else: return Xor(expr1, expr2)

# Return True if e1 and e2 can be proven equivalent according
# to some rules, False otherwise.   Note that False may
# be returned in some cases when the exprs are equivalent.
def equal_exprs(e1, e2):
    if isinstance(e1, FalseLiteral): return isinstance(e2, FalseLiteral)
    elif isinstance(e1, TrueLiteral): return isinstance(e2, TrueLiteral)
    elif isinstance(e1, Var) and isinstance(e2, Var):
        return e1.varname == e2.varname
    elif isinstance(e1, Not) and isinstance(e2, Not):
        return equal_exprs(e1.operand, e2.operand)
    elif isinstance(e1, And) and isinstance(e2, And):
        if equal_exprs(e1.operand1, e2.operand1):
            return equal_exprs(e1.operand2, e2.operand2)
        elif equal_exprs(e1.operand1, e2.operand2):
            return equal_exprs(e1.operand2, e2.operand1)
        else: return False
    elif isinstance(e1, Or) and isinstance(e2, Or):
        if equal_exprs(e1.operand1, e2.operand1):
            return equal_exprs(e1.operand2, e2.operand2)
        elif equal_exprs(e1.operand1, e2.operand2):
            return equal_exprs(e1.operand2, e2.operand1)
        else: return False
    elif isinstance(e1, Xor) and isinstance(e2, Xor):
        if equal_exprs(e1.operand1, e2.operand1):
            return equal_exprs(e1.operand2, e2.operand2)
        elif equal_exprs(e1.operand1, e2.operand2):
            return equal_exprs(e1.operand2, e2.operand1)
        else: return False
    elif isinstance(e1, Sel) and isinstance(e2, Sel):
        if equal_exprs(e1.sel, e2.sel):
             if equal_exprs(e1.true_branch, e2.true_branch):
                 return equal_exprs(e1.false_branch, e2.false_branch)
             else: return False
        else: return False
    elif isinstance(e1, Adv) and isinstance(e2, Adv):
        if e1.offset == e2.offset: return equal_exprs(e1.operand(), e2.operand())
        else: return False
    else: return False

def make_shift_forward(expr, n):
    if isinstance(expr, FalseLiteral):
        return expr
    elif n == 0:
        return expr
    elif isinstance(expr, Adv):
        return Adv(expr.operand, n + expr.offset)
    else: return Adv(expr, n)



#
#  Pablo Statements.
#
import ast

class PabloStmt:
   pass

class Assign(PabloStmt):
    def __init__(self, varname, expr):
        self.varname = varname
        self.expr = expr
    def __str__(self): return 'Assign("' + self.varname + ', ' + self.expr.__str__() + '")'
    def toAST(self): return ast.Assign([ast.Name(id=self.varname, ctx=ast.Store())], self.expr.toAST())

class IfStmt(PabloStmt):
    def __init__(self, predicate, stmts):
        self.predicate = predicate
        self.stmts = stmts
    def __str__(self): return 'If(%s, %s)' % (self.predicate.__str__(), self.stmts.__str__())
    def toAST(self): return ast.If(self.predicate.toAST(), [s.toAST() for s in self.stmts], [])

class WhileLoop(PabloStmt):
    def __init__(self, predicate, stmts):
        self.predicate = predicate
        self.stmts = stmts
    def __str__(self): return 'While(%s, %s)' % (self.predicate.__str__(), self.stmts.__str__())
    def toAST(self): return ast.While(self.predicate.toAST(), [s.toAST() for s in self.stmts], [])



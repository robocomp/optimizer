# -*- coding: utf-8 -*-

from sympy import *
from sympy.abc import x,y,mu,tau,a,b,R
init_printing(use_unicode=True)

a, b, R = symbols('a b R')

m = Matrix([[1, 1, a+b], [1, -1,-a-b], [1, -1, -a+b],[1, 1, a-b]])

res =  ((m.T*m)/R)**-1 * m.T
res.simplify()
f = gcd(tuple(res))
res2 = MatMul(f, (res/f), evaluate=False)

pprint(res2)
print(latex(res2))



import numpy as np
from sympy import *
p0,p1,p05,p0d,p1d= symbols("p0,p1,p05,p0d,p1d")
a=np.matrix([[1,0,0,0,0],
            [1,1,1,1,1],
            [1,1/2,1/4,1/8,1/16],
            [0,1,0,0,0],
            [0,1,2,3,4]])
b=np.linalg.inv(a)
print(b)
y=np.matrix[p0,p1,p05,p0d,p1d]
#z=np.matmul(b,transpose(y))

print(z)
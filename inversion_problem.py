from sympy import *
from time import time
from mpmath import radians
import numpy as np

R_sol = Matrix([[0.0684405930068017, 0.792566422358903, 0.605932629405354], [-0.0521301776042901, -0.603686007756541, 0.795515963775658], [0.996292291354210, -0.0860329598943190, 0]])

R_my = Matrix([[0.0684478539570781, 0.792565795319597, 0.605932629405354], [-0.0521357081616808, -0.603685530150066, 0.795515963775658], [0.996291503137084, -0.0860420872413579, 0]])

print("matrx difference:")
print( R_my-R_sol)
print("\n")
print("matrix inversions difference:")
print( R_my.inv("LU")-R_sol.inv("LU") )
print("\n")

import numpy as np
import cvxpy as cvx

A = np.matrix([[0, 1, 0, 0],
            [-48.6, -1.25, 48.6, 0],
            [0, 0, 0, 1],
            [19.5, 0, -19.5, 0]])

B = np.matrix([0, 0, 0, 1]).T

C = np.matrix([1, 0, 0, 0])

# Compose sets of vertices bounding the space of h and p elementwise:
V_H = [np.matrix([0, 0, -3.33, 0]),
       np.matrix([0, 0, 3.33, 0])]

# Helper functions
e = lambda s, i: np.matrix([0 if n != i else 1 for n in range(s)]).T

def affine_func(v, A, B):
    """Defined as A(v) = A + B*sum(v_ij*e_q(i)*e_n(j)')"""
    n = A.shape[1]
    q = B.shape[1]
    summand = 0
    for i in range(q):
        for j in range(n):
            summand += v[i,j]*e(q, i)*e(n, j).T
    return A + B*summand

def gamma_func(alpha, A, B, C, P, R):
    """Defined as Gamma(alpha) = A(alpha)'*P + P*A(alpha) - (C'*R + R'*C)"""
    aff = affine_func(alpha, A, B)
    return aff.T*P + P*aff - (C.T*R + R.T*C)

# Compose variables to optimize over
P = cvx.Semidef(4, 'P')
R = cvx.Variable(1, 4, 'R')

# Compose the LMI block matrix
bl1, bl2 = [gamma_func(alpha, A, B, C, P, R) for alpha in V_H]
Z = np.zeros((4, 4))
lmi = cvx.vstack(cvx.hstack(bl1, Z), cvx.hstack(Z, bl2))

# Form the input to cvxpy
obj = cvx.Minimize(0)
consts = [lmi <= -1e-13, P >= 1e-13]
prob = cvx.Problem(obj, consts)

# Solve the problem
prob.solve(verbose=True)
"""
LQR solver
==========

Adapted from Mark `Wilfried Mueller <https://www.mwm.im/lqr-controllers-with-python/>`_
"""

import numpy as np
import scipy.linalg


def lqr(A, B, Q, R):
    """Solve the continuous time lqr controller.
    dx/dt = A x + B u
    cost = integral x.T*Q*x + u.T*R*u
    ref: Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = np.array(scipy.linalg.solve_continuous_are(A, B, Q, R))

    # compute the lqr gain
    K = np.array(scipy.linalg.inv(R).dot(B.T.dot(X)))
    eigVals, eigVecs = scipy.linalg.eig(A-B.dot(K))
    return K, X, eigVals


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    ref: Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = np.array(scipy.linalg.solve_discrete_are(A, B, Q, R))

    # compute the lqr gain
    K = np.array(scipy.linalg.inv(B.T.dot(X.dot(B))+R).dot(B.T.dot(X.dot(A))))
    eigVals, eigVecs = scipy.linalg.eig(A-B.dot(K))
    return K, X, eigVals

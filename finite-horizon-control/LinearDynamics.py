import numpy
import pypoman

"""
n_x
"""

"""
LinearDynamics()
Description:
    A simple Linear Dynamical system stored in an object.
    Defines the linear system:
        x^+ = A x + B u + w , w \in W
            y = C x + v       , v \in V

"""
class LinearDynamics:

    def __init__(self,A,B,C=None,W::Polyhedron,V::Polyhedron):

        self.checkA(A) 

    def checkA(self,A):

        if A.ndim != 2:
            raise ValueError("Expected for A to be square matrix, but received a matrix with " + str(A.ndims) + " ndims." )

        if np.size(A,axis=0) != np.size(A,axis=1):
            raise ValueError("Input matrix A is not square! size = " + str(np.size(A)) )
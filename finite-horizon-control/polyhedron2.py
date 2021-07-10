import numpy as np

"""
DefinePolyhedronWithBounds
Description:
"""
def DefinePolyhedronWithBounds(lb,ub):
    # Constants
    dim  = np.size(lb,0)
    dim2 = np.size(ub,0)

    if dim != dim2:
        raise ValueError("Dimensions of lb (" + str(dim) + ") is different from dimension of ub (" + str(dim2) + ")." )

    n = dim

    # Algorithm
    A = np.vstack([-np.eye(n),+np.eye(n)])
    b = np.concatenate( (lb,ub) , axis=None )

    return Polyhedron(A,b,[],[])

"""
Polyhedron
Description:

"""
class Polyhedron:

    def __init__(self,A=None,b=None,Ae=None,be=None):
        """
        Constructor
        """

        # Input Checking
        matricesAreUndefined = (A is None) and (b is None) and (Ae is None) and (be is None)
        # boundsNotGiven = (lb is None) and (ub is None)
        if matricesAreUndefined:
            raise ValueError("Polyhedron should be defined with a pair of valid matrices defining some form of Polyhedron.")

        # Check the dimensions of each matrices

        # Save the Values of the Constraint Matrices
        self.A = A 
        self.b = b 
        self.Ae = Ae
        self.be = be 

import numpy as np
import pypoman

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

        self.Dim = np.size(A,axis=1)

    """
    contains
    Description:
        Determines whether or not the given point x is contained int he polyhedron.
    """
    def contains(self,x):

        if x.ndim != 1:
            raise ValueError("Input point x is not a vector. The value of ndim is " + str(x.ndim) + "." )

        if np.size(x,axis=0) != self.Dim:
            # If the input vector is of the same size as the 
            raise ValueError("Input point x is not of the proper dimension. Expected dimension " + str(self.Dim) + ", but x is of dimension " + str(np.size(x,axis=0)) + ".")

        # constants

        A = self.A
        b = self.b

        Ae = self.Ae
        be = self.be

        # Algorithm
        contains_flag = True

        if not(A is None):
            # A is defined.
            contains_flag = contains_flag and ( A.dot(x) <= b ).all()
        
        if not(Ae is None):
            # Ae is defined
            contains_flag = contains_flag and (Ae.dot(x) == be).all()

        return contains_flag

    """
    is_empty
    Description:
        Determines whether or not the given polyhedron is empty.
    """
    def is_empty(self):

        # Constants

        # Algorithm

        c = np.ones(self.Dim)
        
        if not(self.A is None):
            G = self.A
            h = self.b
        else:
            G = np.zeros(self.Dim)
            h = np.zeros([0.0])

        if not(self.Ae is None):
            A = self.Ae
            b = self.be
        else:
            A = None
            b = None

        try:
            temp = pypoman.solve_lp(c,G,h,A,b)
            # print(temp)
            return False
        except ValueError as e:
            return True
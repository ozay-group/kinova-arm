"""
LinearDynamics_test.py
Description:
    Test suite for the LinearDynamics object.
"""

class TestPolyhedron(unittest.TestCase):

    def test_constructor1(self):

        try:
            p1 = Polyhedron()
        except ValueError as e:
            assert True

    def test_len(self):

        bound_array = np.array([1.0,2.0,3.0])
        bound_size = np.size(bound_array)
        #print(bound_size)
        assert (bound_size == 3)

    def test_transpose(self):
        bound_array = np.array([1.0,2.0,3.0])
        # print(np.transpose(bound_array.transpose()))

    """
    test_contains1
    Description:
        Tests error handling when Polyhedron receives value which is not a vector in the contains() function.
    """
    def test_contains1(self):
        p1 = Polyhedron(np.array([ [1.0,0.0],[0.0,1.0],[-1.0,0.0],[0.0,-1.0] ]),np.array([1.0,1.0,1.0,1.0]))
        pointToTry = np.array([[1.0,0.0],[0.0,1.0]])

        try:
            p1.contains(pointToTry)
        except ValueError as e:
            assert str(e) == "Input point x is not a vector. The value of ndim is 2."
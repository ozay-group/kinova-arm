import unittest
import numpy as np

from polyhedron2 import Polyhedron, DefinePolyhedronWithBounds

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

    """
    test_contains2
    Description:
        Tests error handling when Polyhedron receives x value in contains function which is not of the appropriate dimension.
    """
    def test_contains2(self):
        p1 = Polyhedron(np.array([ [1.0,0.0],[0.0,1.0],[-1.0,0.0],[0.0,-1.0] ]),np.array([1.0,1.0,1.0,1.0]))
        pointToTry = np.array([1.0,0.0,1.0])

        try:
            p1.contains(pointToTry)
        except ValueError as e:
            assert str(e) == "Input point x is not of the proper dimension. Expected dimension 2, but x is of dimension 3."

    """
    test_contains3
    Description:
        Tests whether or not our implementation properly identifies when a point is contained in a Polyhedron.
    """
    def test_contains3(self):
        p1 = Polyhedron(np.array([ [1.0,0.0],[0.0,1.0],[-1.0,0.0],[0.0,-1.0] ]),np.array([1.0,1.0,1.0,1.0]))
        pointToTry = np.array([0.0,0.2])

        assert p1.contains(pointToTry)

    """
    test_contains4
    Description:
        Tests whether or not our implementation properly identifies when a point IS NOT contained in a Polyhedron.
    """
    def test_contains4(self):
        p1 = Polyhedron(np.array([ [1.0,0.0],[0.0,1.0],[-1.0,0.0],[0.0,-1.0] ]),np.array([1.0,1.0,1.0,1.0]))
        pointToTry = np.array([-3.0,0.2])

        assert not(p1.contains(pointToTry))

    """
    test_contains5
    Description:
        Tests whether or not our implementation properly identifies when a point IS contained in a Polyhedron
        with equality constraints.
    """
    def test_contains5(self):
        p1 = Polyhedron(
            np.array([ [1.0,0.0],[0.0,1.0],[-1.0,0.0],[0.0,-1.0] ]),np.array([1.0,1.0,1.0,1.0]),
            np.array([[1.0,-1.0]]),np.array([0.0]))
        pointToTry = np.array([0.2,0.2])

        assert p1.contains(pointToTry)

    """
    test_contains6
    Description:
        Tests whether or not our implementation properly identifies when a point IS NOT contained in a Polyhedron
        with equality constraints.
    """
    def test_contains6(self):
        p1 = Polyhedron(
            np.array([ [1.0,0.0],[0.0,1.0],[-1.0,0.0],[0.0,-1.0] ]),np.array([1.0,1.0,1.0,1.0]),
            np.array([[1.0,-1.0]]),np.array([0.0]))
        pointToTry = np.array([0.2,0.5])

        assert not(p1.contains(pointToTry))

    """
    test_is_empty1
    Description:

    """
    def test_is_empty1(self):
        p1 = Polyhedron(np.array([ [1.0,0.0],[0.0,1.0],[-1.0,0.0],[0.0,-1.0] ]),np.array([1.0,1.0,1.0,1.0]))

        assert not(p1.is_empty())

    """
    test_is_empty2
    Description:
        Creates a polytope that should be empty and verifies that with a quick call to our custom function.
    """
    def test_is_empty2(self):
        p1 = Polyhedron(np.array([ [1.0,0.0],[0.0,1.0],[-1.0,0.0],[0.0,-1.0],[1.0,0.0] ]),np.array([1.0,1.0,1.0,1.0,-2.0]))

        assert p1.is_empty()

    #     self.assertEqual('foo'.upper(), 'FOO')

    # def test_isupper(self):
    #     self.assertTrue('FOO'.isupper())
    #     self.assertFalse('Foo'.isupper())

    # def test_split(self):
    #     s = 'hello world'
    #     self.assertEqual(s.split(), ['hello', 'world'])
    #     # check that s.split fails when the separator is not a string
    #     with self.assertRaises(TypeError):
    #         s.split(2)

class TestDefinePolyhedronWithBounds(unittest.TestCase):

    def test_error_handling1(self):
        try:
            p1 = DefinePolyhedronWithBounds( np.array([1.0,2.1,3.1]) , np.array([2.0,3.8]) )
        except ValueError as e:
            assert True

    # Testing the constructor truly works by verifying A
    def test_constructor1(self):
        lb2 = np.array([1.0,1.0,1.0])
        ub2 = np.array([2.0,2.0,2.0])
        p2 = DefinePolyhedronWithBounds(lb2,ub2)

        assert np.equal(p2.A,np.vstack([-np.eye(3),np.eye(3)])).all()

    # Testing the constructor truly works by verifying b
    def test_constructor2(self):
        lb2 = np.array([1.0,1.0,1.0])
        ub2 = np.array([2.0,2.0,2.0])
        p2 = DefinePolyhedronWithBounds(lb2,ub2)

        assert np.equal(p2.b,np.concatenate((lb2,ub2),axis=None)).all()

if __name__ == '__main__':
    unittest.main()
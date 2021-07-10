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
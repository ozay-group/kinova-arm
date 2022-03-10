"""
language.py
Description:
    Creates the Language object and (maybe?) tests it.
"""

import unittest

class Language:
    """
    A Language is a set of words where each word is a sequence of integers.    
        
    """

    def __init__(self,words:tuple=()) -> None :
        """
        __init__
        Description:
            Initializes the set.
        """
        # Constants
        Language.words = words

    def cardinality(self) -> int :
        return len(self.words)

    def contains(self,word_in) -> bool :
        """
        Description:
        Checks whether or not a given word word_in is in the language self.
        """

        # algorithm
        for temp_word in self.words:
            if word_in == temp_word:
                return True

        # if no words matched, then word_in is not contained
        return False

class TestLanguageMethods(unittest.TestCase):
    """
    test_construct1
    Description:
        Test the empty language construction.
    """
    def test_construct1(self):
        L0 = Language()
        self.assertEqual(L0.words,())
        self.assertEqual(L0.cardinality(),0)

    """
    test_contains1
    Description:
        Test the contains() function with word that is not in target language.
    """
    def test_contains1(self):
        L0 = Language([1,1,1],[2,2,2])
        self.assertTrue(L0.contains([2,2,2]))

    # def test_isupper(self):
    #     self.assertTrue('FOO'.isupper())
    #     self.assertFalse('Foo'.isupper())

    # def test_split(self):
    #     s = 'hello world'
    #     self.assertEqual(s.split(), ['hello', 'world'])
    #     # check that s.split fails when the separator is not a string
    #     with self.assertRaises(TypeError):
    #         s.split(2)

if __name__ == '__main__':
    unittest.main()
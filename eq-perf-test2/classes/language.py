"""
language.py
Description:
    Creates the Language object and (maybe?) tests it.
"""

import unittest

#from sympy import false

class Language():
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
        self.words = words

    def __str__(self):
        """
        __str__
        Description:
            Displays a string for this object.
        """

        # Constants
        word_limit = 5 # Maximum Number of Words to display.

        # Algorithm
        output_string = "Language with " + str(self.cardinality()) + " words "

        if self.cardinality() <= word_limit:
            output_string += "( "
            for word_index in range( self.cardinality() ):
                output_string += "- word" + str(word_index) + " = " + str(self.words[word_index]) + " "

                if word_index != self.cardinality()-1:
                    output_string += ", "

            output_string += ")"

        return output_string

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
    
    def is_superset_of(self,L_in)->bool:
        """
        is_superset_of
        Description:
            This function verifies whether or not self (a language) is a superset of L_in.
        """

        # Verify that all words from L_in are in self.
        for word in L_in.words:
            if not( self.contains(word) ):
                # self does not contain the word "word"!
                # This means self can't be a superset of L_in.
                return False

        # If all words in L_in are in self, then return true
        return True

    def is_subset_of(self,L_in)->bool:
        """
        is_subset_of
        Description:
            This function verifies whether or not self (a language) is a SUBSET of L_in.
        """

        # Verify that all words from L_in are in self.
        for word in self.words:
            if not( L_in.contains(word) ):
                # self does not contain the word "word"!
                # This means self can't be a superset of L_in.
                return False

        # If all words in L_in are in self, then return true
        return True

    def __ge__(self,L_in)->bool:
        """
        >=
        Description:
            Operator definition of the supseteq operator.
        """
        return self.is_superset_of(L_in)

    def __le__(self,L_in)->bool:
        """
        <=
        Description:
            Operator definition of the subseteq operator.
        """
        return self.is_subset_of(L_in)


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
        Test the contains() function with word that is in target language.
    """
    def test_contains1(self):
        L0 = Language(([1,1,1],[2,2,2]))
        self.assertTrue(L0.contains([2,2,2]))

    def test_contains2(self):
        """
        test_contains2
        Description:
            Tests the contains() function, where the word is not in the target language.
        """
        L0 = Language(([1,1,1],[2,2,2]))
        self.assertFalse(L0.contains([3,3,3]))

    def test_is_supset_of1(self):
        """
        test_is_superset_of1
        Description:
            Tests the is_superset_of function for Language.
            In this case neither language is a superset of the other.
        """
        L1 = Language(([1,1,1],[2,2,2]))
        L2 = Language(([2,2,2],[3,3,3]))

        self.assertFalse(L1.is_superset_of(L2))

    def test_is_supset_of2(self):
        """
        test_is_superset_of2
        Description:
            Tests the is_superset_of function for Language.
            In this case L1 is a superset of L2.
        """
        L1 = Language(([1,1,1],[2,2,2],[3,3,3]))
        L2 = Language(([2,2,2],[3,3,3]))

        self.assertTrue(L1.is_superset_of(L2))

    def test_ge_1(self):
        """
        test_ge_1
        Description:
            Tests the >= function for Language.
            In this case neither language is a superset of the other.
        """
        L1 = Language(([1,1,1],[2,2,2]))
        L2 = Language(([2,2,2],[3,3,3]))

        self.assertFalse(L1 >= L2)

    def test_le_1(self):
        """
        test_le_1
        Description:
            Tests the <= function for Language.
            In this case neither language is a subset of the other.
        """
        L1 = Language(([1,1,1],[2,2,2]))
        L2 = Language(([2,2,2],[3,3,3]))

        self.assertFalse(L1 <= L2)

    def test_le_2(self):
        """
        test_le_2
        Description:
            Tests the <= function for Language.
            In this case where L1 is a subset of L2.
        """
        L1 = Language(([1,1,1],[2,2,2]))
        L2 = Language(([1,1,1],[2,2,2],[3,3,3]))

        self.assertTrue(L1 <= L2)

    

if __name__ == '__main__':
    unittest.main()
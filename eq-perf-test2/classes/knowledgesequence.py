"""
language.py
Description:
    Creates the Language object and (maybe?) tests it.
"""

import unittest
from classes.language import Language

#from sympy import false

class KnowledgeSequence:
    """
    A Language is a set of words where each word is a sequence of integers.    
        
    """

    def __init__(self,sequence:list=[]) -> None :
        """
        __init__
        Description:
            Initializes the set.
        """
        # Constants
        self.sequence = sequence

    def __str__(self):
        """
        __str__
        Description:
            Displays a string for this object.
        """

        # Constants
        lang_limit = 10 # Maximum Number of Words to display.

        # Algorithm
        output_string = "KnowledgeSequence with " + str( len(self.sequence) ) + " Languages\n"

        for seq_index in range(len(self.sequence)):
            output_string += str(seq_index) + ": " + str(self.sequence[seq_index]) + "\n"

        return output_string
    
    def is_superset_of(self,KS_in)->bool:
        """
        is_superset_of
        Description:
            This function verifies whether or not self (a knowledge sequence) is a superset of KS_in.
        """

        # Input Processing
        if len(self.sequence) != len(KS_in.sequence):
            raise ValueError("The two sequences under comparison should have the same length, but sequence 1 has length " + str(len(self.sequence)) + ", and sequence 2 has length " + str(len(KS_in.sequence) + "."))

        # Iterate through all Languages in each sequence
        for t in range(len(self.sequence)):
            self_L_t = self.sequence[t]
            KS_in_L_t = KS_in.sequence[t]

            if not( self_L_t.is_superset_of(KS_in_L_t) ):
                return False

        # If all languages in L_in the sequence are valid, then return true
        return True

    def is_subset_of(self,ks_in)->bool:
        """
        is_subset_of
        Description:
            This function verifies whether or not self (a language) is a SUBSET of L_in.
        """

        # Input Processing
        if len(self.sequence) != len(ks_in.sequence):
            raise ValueError("The two sequences under comparison should have the same length, but sequence 1 has length " + str(len(self.sequence)) + ", and sequence 2 has length " + str(len(KS_in.sequence) + "."))

        # Iterate through all Languages in each sequence
        for t in range(len(self.sequence)):
            self_L_t = self.sequence[t]
            KS_in_L_t = ks_in.sequence[t]

            if not( self_L_t.is_subset_of(KS_in_L_t) ):
                return False

        # If all words in L_in are in self, then return true
        return True

    def __ge__(self,ks_in)->bool:
        """
        >=
        Description:
            Operator definition of the supseteq operator.
        """
        return self.is_superset_of(ks_in)

    def __le__(self,ks_in)->bool:
        """
        <=
        Description:
            Operator definition of the subseteq operator.
        """
        return self.is_subset_of(ks_in)

    def time_horizon(self)->int:
        """
        len
        Description:
            Returns the number of elements in the sequence.
        """
        return len(self.sequence)


class TestKnowledgeSequenceMethods(unittest.TestCase):
    """
    test_construct1
    Description:
        Test the empty language construction.
    """
    def test_construct1(self):
        """
        test_construct1
        Description:
            Creating a simple KnowledgeSequence object and then verify that its sequence member variable is correctly shaped.
        """
        ks1 = KnowledgeSequence([
            Language( ([1,1,1],[2,2,2],[3,3,3]) ),
            Language( ([2,2,2],[3,3,3]) ),
            Language( ([2,2,2]) )
        ])

        self.assertTrue( len(ks1.sequence) == 3 )
        

    def test_is_superset_of1(self):
        """
        test_is_superset_of1
        Description:
            Tests the is_superset_of function for KnowledgeSequence.
            In this case the sequences are the same, so the function should return true.
        """
        L1 = Language(([1,1,1],[2,2,2]))
        L2 = Language(([2,2,2],[3,3,3]))

        ks1 = KnowledgeSequence([L2,L2])
        ks2 = KnowledgeSequence([L2,L2])

        self.assertTrue(ks1.is_superset_of(ks2))

    def test_is_superset_of2(self):
        """
        test_is_superset_of2
        Description:
            Tests the is_superset_of function for KnowledgeSequence.
            In this case the sequences are not all uniformly a superset of one or the other.
        """
        L1 = Language(([1,1,1],[2,2,2]))
        L2 = Language(([2,2,2],[3,3,3]))

        ks1 = KnowledgeSequence([L1,L1])
        ks2 = KnowledgeSequence([L2,L2])

        self.assertFalse(ks1.is_superset_of(ks2))

    def test_is_superset_of3(self):
        """
        test_is_superset_of3
        Description:
            Tests the is_superset_of function for KnowledgeSequence.
            In this case ks1 should be a superset of ks2.
        """
        L1 = Language(([1,1,1],[2,2,2],[3,3,3]))
        L2 = Language(([2,2,2],[3,3,3]))

        ks1 = KnowledgeSequence([L1,L1])
        ks2 = KnowledgeSequence([L2,L2])

        self.assertTrue(ks1.is_superset_of(ks2))

    def test_ge_1(self):
        """
        test_ge_1
        Description:
            Tests the >= function for KnowledgeSequence.
            In this case neither language is a superset of the other.
        """
        L1 = Language(([1,1,1],[2,2,2],[3,3,3]))
        L2 = Language(([2,2,2],[3,3,3]))

        ks1 = KnowledgeSequence([L1,L1])
        ks2 = KnowledgeSequence([L2,L2])

        self.assertTrue(ks1 >= ks2)

    def test_le_1(self):
        """
        test_le_1
        Description:
            Tests the <= function for KnowledgeSequence.
            In this case the knowledge sequences are equal, and so one is trivially a subset of the other.
        """
        L1 = Language(([1,1,1],[2,2,2]))
        L2 = Language(([2,2,2],[3,3,3]))

        ks1 = KnowledgeSequence([L2,L2])
        ks2 = KnowledgeSequence([L2,L2])

        self.assertTrue(ks1 <= ks2)

    def test_le_2(self):
        """
        test_le_2
        Description:
            Tests the <= function for Language.
            In this case neither knowledge sequence is a subset of the other.
        """
        L1 = Language(([1,1,1],[2,2,2]))
        L2 = Language(([2,2,2],[3,3,3]))

        ks1 = KnowledgeSequence([L1,L1])
        ks2 = KnowledgeSequence([L2,L2])

        self.assertFalse(ks1 <= ks2)


if __name__ == '__main__':
    unittest.main()
import unittest

# import local files
import classes.consistentbeliefcontroller as cbc
import classes.language as language
import classes.affinedynamics as ad
import classes.knowledgesequence as ks

# run all unit tests

if __name__ == '__main__':
    #all_module_names = ['language','consistentbeliefcontroller']

    suite = unittest.TestSuite()
    loader = unittest.TestLoader()

    suite.addTests(loader.loadTestsFromModule(language))
    suite.addTests(loader.loadTestsFromModule(consistentbeliefcontroller))
    suite.addTests(loader.loadTestsFromModule(ad))
    suite.addTests(loader.loadTestsFromModule(ks))
    print("Constructed test suite!")
    print(suite)

    results = unittest.TestResult()


    suite.run(results)
    print("\nRan test suite!")
    print(results)

    # Print formatted results
    if results.wasSuccessful():
        print("\nAll tests ran successfully!\n")
    else:
        print("There was an issue while running the tests!\n")
        print(results.errors)
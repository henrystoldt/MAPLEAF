import unittest

from MAPLEAF.Utilities import evalExpression, cacheLastResult
import time


class TestUtilities(unittest.TestCase):
    
    def test_evalExpression(self):
        varDict = { "bodyWeight": 0.15 }
        result = evalExpression( "0.007506 + 0.01/bodyWeight", varDict)
        self.assertAlmostEqual(result, 0.0741726666666)

    def test_cacheLastResults(self):
        # Check that the function is faster after caching than before
        @cacheLastResult
        def cachedFunc():
            time.sleep(0.000005)
            return 5

        startTime = time.time()
        val1 = cachedFunc()
        endTime = time.time()
        uncachedTime = endTime - startTime
        print(uncachedTime)

        startTime = time.time()
        val2 = cachedFunc()
        endTime = time.time()
        cachedTime = endTime - startTime
        print(cachedTime)

        self.assertTrue(cachedTime < uncachedTime)
        self.assertEqual(val1, val2)
            
        
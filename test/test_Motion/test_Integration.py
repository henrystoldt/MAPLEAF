import unittest

from MAPLEAF.Motion.Integration import AdaptiveIntegrator, ClassicalIntegrator


# https://lpsa.swarthmore.edu/NumInt/NumIntSecond.html
def sampleDerivative(time, val):
    return -2 * val

# https://resources.saylor.org/wwwresources/archived/site/wp-content/uploads/2011/11/ME205-8.3-TEXT.pdf
def sampleDerivative2(time, val):
    return -2.2067e-12 * (val**4 - 81e8)

class TestIntegrator(unittest.TestCase):
    def setUp(self):
        #Define Vectors to be used in the checks below
        pass

    def test_IntegrateRK2Midpoint(self):
        #Simple test case, original result from website was 2.0175!
        integrate = ClassicalIntegrator(method="RK2Midpoint")
        v1 = integrate(3, 0, sampleDerivative, 0.1).newValue
        v2 = integrate(v1, 0.1, sampleDerivative, 0.1).newValue
        self.assertEqual(v2, 2.0172)

        v1 = integrate(1200, 0, sampleDerivative2, 240).newValue
        v2 = integrate(v1, 240, sampleDerivative2, 240).newValue
        self.assertAlmostEqual(v2, 976.87, 2)

    def test_IntegrateRK12(self):
        #Simple test case, original result from website was 2.0175!
        integrate = AdaptiveIntegrator(method="RK12Adaptive", controller="PID", targetError=1500)
        v1 = integrate(3, 0, sampleDerivative, 0.1).newValue
        v2 = integrate(v1, 0.1, sampleDerivative, 0.1).newValue
        self.assertAlmostEqual(v2, 2.0172)

        v1 = integrate(1200, 0, sampleDerivative2, 240).newValue
        v2 = integrate(v1, 240, sampleDerivative2, 240).newValue
        self.assertAlmostEqual(v2, 976.87, 2)

    def test_IntegrateRK23Adaptive_BogackiShampine(self):
        #Simple test case, original result from website was 2.0175!
        integrate = AdaptiveIntegrator(method="RK23Adaptive", controller="PID", targetError=1500)
        v1 = integrate(3, 0, sampleDerivative, 0.1).newValue
        v2 = integrate(v1, 0.1, sampleDerivative, 0.1).newValue
        self.assertAlmostEqual(v2, 2.01064533)

    def test_IntegrateRK45Adaptive_DormandPrince(self):
        #Simple test case, original result from website was 2.0175!
        integrate = AdaptiveIntegrator(method="RK45Adaptive", controller="PID", targetError=1500)
        v1 = integrate(3, 0, sampleDerivative, 0.1).newValue
        v2 = integrate(v1, 0.1, sampleDerivative, 0.1).newValue
        self.assertAlmostEqual(v2, 2.0109602376)

    def test_IntegrateRK78Adaptive_DormandPrince(self):
        #Simple test case, original result from website was 2.0175!
        integrate = AdaptiveIntegrator(method="RK78Adaptive", controller="PID", targetError=1500)
        v1 = integrate(3, 0, sampleDerivative, 0.1).newValue
        v2 = integrate(v1, 0.1, sampleDerivative, 0.1).newValue
        self.assertAlmostEqual(v2, 2.010960138)

    def test_IntegratorRK2Heun(self):
        #More complicated test case, based on radiative cooling
        integrate = ClassicalIntegrator(method="RK2Heun")        
        v1 = integrate(1200, 0, sampleDerivative2, 240).newValue
        v2 = integrate(v1, 240, sampleDerivative2, 240).newValue
        self.assertAlmostEqual(v2, 584.27, 2)

    def test_IntegrateEuler(self):
        integrate = ClassicalIntegrator(method="Euler")
        v1 = integrate(3, 0, sampleDerivative, 0.1).newValue
        self.assertEqual(v1, 2.4)

    def test_IntegrateRK4(self):
        integrate = ClassicalIntegrator(method="RK4")
        v1 = integrate(3, 0, sampleDerivative, 0.2).newValue
        self.assertEqual(v1, 2.0112)

    def test_IntegrateRK4_38(self):    
        integrate = ClassicalIntegrator(method="RK4_3/8")
        v1 = integrate(3, 0, sampleDerivative, 0.2).newValue
        self.assertAlmostEqual(v1, 2.0112)

#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()

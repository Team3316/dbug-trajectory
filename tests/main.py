import unittest

from tests.test_curve import CurveTests

if __name__ == '__main__':
    suite = unittest.makeSuite(CurveTests, 'test')

    runner = unittest.TextTestRunner()
    runner.run(suite)
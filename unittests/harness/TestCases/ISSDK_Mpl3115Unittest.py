'''
@author    : B35385
@name      : ISSDK_Mpl3115Unittest
@copyright : Copyright (c) 2016, NXP Semiconductors.

********************************************************************************
The test will load the DUT Driver Example onto the DUT Board.
Once loaded the Driver Example will execute automatically and the test will capture the output.
********************************************************************************
'''

import time
import unittest
import outputFormat
import serialComDrv
import driverExceptions
import testConfiguration

MPL3115_SHIELD_NAME   = None
BOARD_SHIELD_APPENDED = False

class ISSDK_Mpl3115Unittest(unittest.TestCase):

    output = outputFormat.OutputFormat()
    driver = serialComDrv.SerialComDrv()

    def setUp(self):
        global BOARD_SHIELD_APPENDED
        if not BOARD_SHIELD_APPENDED:
            BOARD_SHIELD_APPENDED = True
            self.__class__.__name__ = "%s_%s_%s" % (testConfiguration.BOARD_NAME, MPL3115_SHIELD_NAME, self.__class__.__name__)
        try:
            # Open the connection to the Simulator first.
            print ("\n\nOpening Connection...")
            self.driver.openConnection(testConfiguration.BOARD_OBJECT_HANDLE)

        except driverExceptions.DriverError:
            print("DriverException: Error, method: setUp")
            self.output.Assert(False, "DriverException: Error, method: setUp", self.output.lineno())

    def tearDown(self):
        self.driver.closeConnection()
        time.sleep(3)

    def test_unittest(self):
        self.driver.dutPortFlush()
        self.output.testInit("ISSDK_mpl3115_unittests", "ISSDK Driver Examples for MPL3115.")
        print("\n\n****************** Executing Test  ******************\n")
        try:
            # Load the DUT image onto the DUT Board.
            testConfiguration.BOARD_OBJECT_HANDLE.loadDutImage((testConfiguration.MPL3115_TEST_FILE).replace("P3115", MPL3115_SHIELD_NAME))
            time.sleep(1)

            # Read the Test output.
            testOutput = self.driver.dutPortRead()
            print(testOutput)

            # Extract and Capture subtest results for Unity framework Tests.
            self.output.parseSubtestResults(testOutput)

            self.output.assert_(testOutput.endswith(outputFormat.UNITY_FWK_TEST_PASS_STR), "Tests Failed.", self.output.lineno())

        except driverExceptions.DriverError:
            self.output.assert_(False, "DriverException: Error, method: setUp", self.output.lineno())
        except:
            self.output.assert_(False, "Exception: Default Exception Error.", self.output.lineno())

        print("\n\n****************** Test Completed  ******************\n")
        self.output.testEnd()

def suite(shield_name = "P3115"):
    # Since these test may run on MULT2B also, we capture the Shield name from the suite file and update.
    global MPL3115_SHIELD_NAME
    MPL3115_SHIELD_NAME = shield_name

    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(ISSDK_Mpl3115Unittest))

    return suite

if __name__ == "__main__":
    unittest.main()

'''
@author    : B35385
@name      : ISSDK_Fxas21002IntExamples
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

# Test case specific configuration.
FXAS21002_RUN_TIME    = 10  # Time for example to process configured samples.
FXAS21002_SHIELD_NAME = None
BOARD_SHIELD_APPENDED = False

class ISSDK_Fxas21002IntExamples(unittest.TestCase):

    output = outputFormat.OutputFormat()
    driver = serialComDrv.SerialComDrv()

    def setUp(self):
        global BOARD_SHIELD_APPENDED
        if not BOARD_SHIELD_APPENDED:
            BOARD_SHIELD_APPENDED = True
            self.__class__.__name__ = "%s_%s_%s" % (testConfiguration.BOARD_NAME, FXAS21002_SHIELD_NAME, self.__class__.__name__)
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

    def test_interrupt(self):
        self.driver.dutPortFlush()
        self.output.testInit("ISSDK_fxas21002_interrupt", "ISSDK Driver Examples for FXAS21002.")
        print("\n\n****************** Executing Test  ******************\n")
        try:
            # Load the DUT image onto the DUT Board.
            testConfiguration.BOARD_OBJECT_HANDLE.loadDutImage((testConfiguration.FXAS21002_INT_FILE).replace("AGM01", FXAS21002_SHIELD_NAME))
            time.sleep(FXAS21002_RUN_TIME) # Wait for samples to be processed.

            # Read the Test output.
            testOutput = self.driver.dutPortRead()
            print(testOutput)

            # Extract and Capture sample processing results for Driver Example Tests.
            testResult = self.output.parseDriverExampleResults(testOutput)

            self.output.assert_(testResult, "Test Failed.", self.output.lineno())

        except driverExceptions.DriverError:
            self.output.assert_(False, "DriverException: Error, method: setUp", self.output.lineno())
        except:
            self.output.assert_(False, "Exception: Default Exception Error.", self.output.lineno())

        print("\n\n****************** Test Completed  ******************\n")
        self.output.testEnd()

def suite(shield_name = "AGM01"):
    # Since these test may run on MULT2B also, we capture the Shield name from the suite file and update.
    global FXAS21002_SHIELD_NAME
    FXAS21002_SHIELD_NAME = shield_name

    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(ISSDK_Fxas21002IntExamples))

    return suite

if __name__ == "__main__":
    unittest.main()

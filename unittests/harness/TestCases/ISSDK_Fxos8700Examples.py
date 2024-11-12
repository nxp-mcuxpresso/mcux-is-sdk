'''
@author    : B35385
@name      : ISSDK_Fxos8700Examples
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
FXOS8700_RUN_TIME     = 3  # Time for example to process configured samples.
FXOS8700_RUN_REPEATS  = 6  # Repeat reads to clean up input buffer.
FXOS8700_SHIELD_NAME  = None
BOARD_SHIELD_APPENDED = False

class ISSDK_Fxos8700Examples(unittest.TestCase):

    output = outputFormat.OutputFormat()
    driver = serialComDrv.SerialComDrv()

    def setUp(self):
        global BOARD_SHIELD_APPENDED
        if not BOARD_SHIELD_APPENDED:
            BOARD_SHIELD_APPENDED = True
            self.__class__.__name__ = "%s_%s_%s" % (testConfiguration.BOARD_NAME, FXOS8700_SHIELD_NAME, self.__class__.__name__)
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

    def test_pfifo(self):
        self.driver.dutPortFlush()
        self.output.testInit("ISSDK_fxos8700_fifo", "ISSDK Driver Examples for FXOS8700.")
        print("\n\n****************** Executing Test  ******************\n")
        try:
            # Load the DUT image onto the DUT Board.
            testConfiguration.BOARD_OBJECT_HANDLE.loadDutImage((testConfiguration.FXOS8700_FIFO_FILE).replace("AGM01", FXOS8700_SHIELD_NAME))
            fullOutput = ''
            for _ in range(FXOS8700_RUN_REPEATS): # This example dumps a lot of output, need to keep reading at intervals.
                time.sleep(FXOS8700_RUN_TIME) # Wait for samples to be processed.
                # Read the Test output.
                testOutput = self.driver.dutPortRead()
                print(testOutput)
                fullOutput = fullOutput + testOutput

            # Extract and Capture sample processing results for Driver Example Tests.
            testResult = self.output.parseDriverExampleResults(fullOutput)

            self.output.assert_(testResult, "Test Failed.", self.output.lineno())

        except driverExceptions.DriverError:
            self.output.assert_(False, "DriverException: Error, method: setUp", self.output.lineno())
        except:
            self.output.assert_(False, "Exception: Default Exception Error.", self.output.lineno())

        print("\n\n****************** Test Completed  ******************\n")
        self.output.testEnd()

    def test_poll(self):
        self.driver.dutPortFlush()
        self.output.testInit("ISSDK_fxos8700_poll", "ISSDK Driver Examples for FXOS8700.")
        print("\n\n****************** Executing Test  ******************\n")
        try:
            # Load the DUT image onto the DUT Board.
            testConfiguration.BOARD_OBJECT_HANDLE.loadDutImage((testConfiguration.FXOS8700_POLL_FILE).replace("AGM01", FXOS8700_SHIELD_NAME))
            fullOutput = ''
            for _ in range(FXOS8700_RUN_REPEATS): # This example dumps a lot of output, need to keep reading at intervals.
                time.sleep(FXOS8700_RUN_TIME) # Wait for samples to be processed.
                # Read the Test output.
                testOutput = self.driver.dutPortRead()
                print(testOutput)
                fullOutput = fullOutput + testOutput

            # Extract and Capture sample processing results for Driver Example Tests.
            testResult = self.output.parseDriverExampleResults(fullOutput)

            self.output.assert_(testResult, "Test Failed.", self.output.lineno())

        except driverExceptions.DriverError:
            self.output.assert_(False, "DriverException: Error, method: setUp", self.output.lineno())
        except:
            self.output.assert_(False, "Exception: Default Exception Error.", self.output.lineno())

        print("\n\n****************** Test Completed  ******************\n")
        self.output.testEnd()

def suite(shield_name = "AGM01"):
    # Since these test may run on MULT2B also, we capture the Shield name from the suite file and update.
    global FXOS8700_SHIELD_NAME
    FXOS8700_SHIELD_NAME = shield_name

    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(ISSDK_Fxos8700Examples))

    return suite

if __name__ == "__main__":
    unittest.main()

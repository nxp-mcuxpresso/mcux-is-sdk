'''
@author    : B35385
@name      : ISSDK_P3115Examples
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
MPL3115_RUN_TIME      = 8  # Time for example to process configured samples.
MPL3115_RUN_REPEATS   = 2  # Repeat reads to clean up input buffer.
MPL3115_START_TIME    = 20 # Time for MPL3115 to reach steady state.
MPL3115_SHIELD_NAME   = None
BOARD_SHIELD_APPENDED = False

class ISSDK_P3115Examples(unittest.TestCase):

    output = outputFormat.OutputFormat()
    driver = serialComDrv.SerialComDrv()

    def setUp(self):
        global BOARD_SHIELD_APPENDED
        if not BOARD_SHIELD_APPENDED:
            BOARD_SHIELD_APPENDED = True
            self.__class__.__name__ = "%s_%s_%s" % (testConfiguration.BOARD_NAME, MPL3115_SHIELD_NAME.split('-')[0], self.__class__.__name__)
        # Load reset binary which resets the sensor after every run.
        testConfiguration.BOARD_OBJECT_HANDLE.loadDutImage((testConfiguration.MPL3115_RST_FILE).replace("P3115", MPL3115_SHIELD_NAME))
        time.sleep(1)
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

    def test_poll_interrupt(self):
        self.driver.dutPortFlush()
        self.output.testInit("ISSDK_mpl3115_poll_interrupt", "ISSDK Driver Examples for MPL3115.")
        print("\n\n****************** Executing Test  ******************\n")
        try:
            # Load the DUT image onto the DUT Board.
            testConfiguration.BOARD_OBJECT_HANDLE.loadDutImage((testConfiguration.MPL3115_NORM_INT_FILE).replace("P3115", MPL3115_SHIELD_NAME))
            time.sleep(MPL3115_START_TIME)
            fullOutput = ''
            for _ in range(MPL3115_RUN_REPEATS): # This example dumps a lot of output, need to keep reading at intervals.
                time.sleep(MPL3115_RUN_TIME) # Wait for samples to be processed.
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

    def test_fifo_interrupt(self):
        self.driver.dutPortFlush()
        self.output.testInit("ISSDK_mpl3115_fifo_interrupt", "ISSDK Driver Examples for MPL3115.")
        print("\n\n****************** Executing Test  ******************\n")
        try:
            # Load the DUT image onto the DUT Board.
            testConfiguration.BOARD_OBJECT_HANDLE.loadDutImage((testConfiguration.MPL3115_FIFO_INT_FILE).replace("P3115", MPL3115_SHIELD_NAME))
            time.sleep(MPL3115_START_TIME)
            fullOutput = ''
            for _ in range(MPL3115_RUN_REPEATS): # This example dumps a lot of output, need to keep reading at intervals.
                time.sleep(MPL3115_RUN_TIME) # Wait for samples to be processed.
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

def suite(shield_name = "P3115"):
    # Since these test may run on MULT2B also, we capture the Shield name from the suite file and update.
    global MPL3115_SHIELD_NAME
    MPL3115_SHIELD_NAME = shield_name

    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(ISSDK_P3115Examples))

    return suite

if __name__ == "__main__":
    unittest.main()

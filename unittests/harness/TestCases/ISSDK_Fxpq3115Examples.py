'''
@author    : B35385
@name      : ISSDK_Fxpq3115Examples
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
FXPQ3115_RUN_TIME     = 8  # Time for example to process configured samples.
FXPQ3115_RUN_REPEATS  = 2  # Repeat reads to clean up input buffer.
FXPQ3115_START_TIME   = 15 # Time for FXPQ3115 to reach steady state.
BOARD_SHIELD_APPENDED = False

class ISSDK_Fxpq3115Examples(unittest.TestCase):

    output = outputFormat.OutputFormat()
    driver = serialComDrv.SerialComDrv()

    def setUp(self):
        global BOARD_SHIELD_APPENDED
        if not BOARD_SHIELD_APPENDED:
            BOARD_SHIELD_APPENDED = True
            self.__class__.__name__ = "%s_B3115_%s" % (testConfiguration.BOARD_NAME, self.__class__.__name__)
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

    def test_fifo(self):
        self.driver.dutPortFlush()
        self.output.testInit("ISSDK_fxpq3115_fifo", "ISSDK Driver Examples for FXPQ3115.")
        print("\n\n****************** Executing Test  ******************\n")
        try:
            # Load the DUT image onto the DUT Board.
            testConfiguration.BOARD_OBJECT_HANDLE.loadDutImage(testConfiguration.FXPQ3115_FIFO_FILE)
            time.sleep(FXPQ3115_START_TIME)
            fullOutput = ''
            for _ in range(FXPQ3115_RUN_REPEATS): # This example dumps a lot of output, need to keep reading at intervals.
                time.sleep(FXPQ3115_RUN_TIME) # Wait for samples to be processed.
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

    def test_altitude(self):
        self.driver.dutPortFlush()
        self.output.testInit("ISSDK_fxpq3115_altitude", "ISSDK Driver Examples for FXPQ3115.")
        print("\n\n****************** Executing Test  ******************\n")
        try:
            # Load the DUT image onto the DUT Board.
            testConfiguration.BOARD_OBJECT_HANDLE.loadDutImage(testConfiguration.FXPQ3115_ALT_FILE)
            time.sleep(FXPQ3115_START_TIME)
            fullOutput = ''
            for _ in range(FXPQ3115_RUN_REPEATS): # This example dumps a lot of output, need to keep reading at intervals.
                time.sleep(FXPQ3115_RUN_TIME) # Wait for samples to be processed.
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

    def test_normal(self):
        self.driver.dutPortFlush()
        self.output.testInit("ISSDK_fxpq3115_normal", "ISSDK Driver Examples for FXPQ3115.")
        print("\n\n****************** Executing Test  ******************\n")
        try:
            # Load the DUT image onto the DUT Board.
            testConfiguration.BOARD_OBJECT_HANDLE.loadDutImage(testConfiguration.FXPQ3115_NORM_FILE)
            time.sleep(FXPQ3115_START_TIME)
            fullOutput = ''
            for _ in range(FXPQ3115_RUN_REPEATS): # This example dumps a lot of output, need to keep reading at intervals.
                time.sleep(FXPQ3115_RUN_TIME) # Wait for samples to be processed.
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

    def test_1shot(self):
        # Load reset binary which resets the sensor to set it to default state.
        testConfiguration.BOARD_OBJECT_HANDLE.loadDutImage(testConfiguration.FXPQ3115_RST_FILE)
        self.driver.dutPortFlush()
        self.output.testInit("ISSDK_fxpq3115_oneshot", "ISSDK Driver Examples for FXPQ3115.")
        print("\n\n****************** Executing Test  ******************\n")
        try:
            # Load the DUT image onto the DUT Board.
            testConfiguration.BOARD_OBJECT_HANDLE.loadDutImage(testConfiguration.FXPQ3115_ONE_FILE)
            time.sleep(FXPQ3115_START_TIME)

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

def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(ISSDK_Fxpq3115Examples))

    return suite

if __name__ == "__main__":
    unittest.main()

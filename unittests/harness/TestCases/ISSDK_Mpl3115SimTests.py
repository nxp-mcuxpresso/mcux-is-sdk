'''
@author    : B35385
@name      : ISSDK_Mpl3115SimTests
@copyright : Copyright (c) 2016, NXP Semiconductors.

********************************************************************************
The test will first load the simulator image onto the Simulator Board.
The test will format a Configuration Message and send it to the ISSDK Simulator over UART.
The Simulator will load MPL3115 profile and will subsequently support execution of Unit-Tests.
The test will then format Add Sample Messages with pre-defined samples and send them to the Simulator.
The simulator will then loop over these samples, sending them to the DUT when data is read over I2C.
Once the Simulator is configured, the test will load the DUT Unit-Test image onto the DUT Board.
Once loaded the Unit-Tests will execute automatically and the test will capture the output.
********************************************************************************
'''

import time
import unittest
import outputFormat
import serialComDrv
import driverExceptions
import testConfiguration
import simProtocolAdapter

# Test case specific configuration.
MPL3115_SAMPLE_WIDTH   = 5 # 3 Byte Pressure/Altitude + 2 Bytes Temperature.
MPL3115_SLAVE_ADDERSS  = '60'
BOARD_SHIELD_APPENDED  = False

class ISSDK_Mpl3115SimTests(unittest.TestCase):

    output = outputFormat.OutputFormat()
    driver = serialComDrv.SerialComDrv()

    def setUp(self):
        global BOARD_SHIELD_APPENDED
        if not BOARD_SHIELD_APPENDED:
            BOARD_SHIELD_APPENDED = True
            self.__class__.__name__ = "%s_%s" % (testConfiguration.BOARD_NAME, self.__class__.__name__)
        self.output.testInit("ISSDK_Mpl3115SimTests", "ISSDK Unit-Tests for MPL3115.")

        try:
            # Open the connection to the Simulator first.
            print ("\n\nOpening Connection...")
            self.driver.openConnections(testConfiguration.BOARD_OBJECT_HANDLE)
            self.driver.simPortFlush()
            self.driver.dutPortFlush()

        except driverExceptions.DriverError:
            print("DriverException: Error, method: setUp")
            self.output.Assert(False, "DriverException: Error, method: setUp", self.output.lineno())

    def tearDown(self):
        self.driver.closeConnections()
        self.output.testEnd()
        time.sleep(3)

    def test_unittests(self):
        try:
            print("\n\n****************** Executing Test  ******************\n")

            # Get samples from samples files.
            (sampleCount, sampleList) = simProtocolAdapter.encodeAddSampleCommands(MPL3115_SAMPLE_WIDTH, testConfiguration.MPL3115_SAMPLE_FILE)
            self.output.assert_(sampleList, "Parsing Samples File Failed.", self.output.lineno())

            # Configure the Simulator for MPL3115
            cmdPacket = simProtocolAdapter.encodeConfigureCommand(sampleCount, MPL3115_SLAVE_ADDERSS)
            self.driver.simPortWrite(cmdPacket)
            time.sleep(0.1) # Wait for Simulator to complete Configuration.
            print(self.driver.simPortRead())

            # Add Samples to Simulator Rung Buffer.
            for offset in range(sampleCount):
                self.driver.simPortWrite(sampleList[offset])
            time.sleep(0.1) # Wait for all samples to be loaded.
            print(self.driver.simPortRead())

            # Load the DUT image onto the DUT Board.
            testConfiguration.BOARD_OBJECT_HANDLE.loadDutImage(testConfiguration.MPL3115_TEST_FILE)
            time.sleep(1) # Wait for all Tests to be executed.

            # Read the Test output.
            testOutput = self.driver.dutPortRead()
            print(testOutput)

            # Extract and Capture subtest results for Unity framework Tests.
            self.output.parseSubtestResults(testOutput)

            self.output.assert_(testOutput.endswith(outputFormat.UNITY_FWK_TEST_PASS_STR), "Tests Failed.", self.output.lineno())

            print("\n\n****************** Test Completed  ******************\n")


        except driverExceptions.DriverError:
            self.output.assert_(False, "DriverException: Error, method: setUp", self.output.lineno())
        except:
            self.output.assert_(False, "Exception: Default Exception Error.", self.output.lineno())

def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(ISSDK_Mpl3115SimTests))

    return suite

if __name__ == "__main__":
    unittest.main()

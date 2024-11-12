'''
@author    : B35385
@name      : ISSDK_Mpl3115Streamtest
@copyright : Copyright (c) 2016, NXP Semiconductors.

********************************************************************************
The test will first load the simulator image onto the Simulator Board.
The test will format a Configuration Message and send it to the ISSDK Simulator over UART.
The Simulator will load MPL3115 profile and will subsequently support execution of DUT.
The test will then format Add Sample Messages with pre-defined samples and keep sending them to the Simulator.
The simulator will then loop over these samples, sending them to the DUT when data is read over I2C.
Once the Simulator is configured, the test will load the DUT Stream-Test image onto the DUT Board.
Once loaded the DUT will execute automatically and the test will capture the output.
********************************************************************************
'''

import sys
import time
import unittest
import outputFormat
import serialComDrv
import driverExceptions
import testConfiguration
import simProtocolAdapter

# Test case specific configuration.
MPL3115_TEST_SAMPLES   = 32500  # Max 15-bit value and multiples of samples in file(500).
MPL3115_TEST_DURATION  = 5      # Duration in minutes.
MPL3115_SAMPLE_WIDTH   = 5      # 3 Byte Pressure/Altitude + 2 Bytes Temperature.
MPL3115_SAMPLE_DEPTH   = 100    # Size of Simulator circular buffer.
MPL3115_SLAVE_ADDERSS  = '60'   # MPL3115 I2C Slave Address.
BOARD_SHIELD_APPENDED  = False

class ISSDK_Mpl3115Streamtest(unittest.TestCase):

    output = outputFormat.OutputFormat()
    driver = serialComDrv.SerialComDrv()

    def setUp(self):
        global BOARD_SHIELD_APPENDED
        if not BOARD_SHIELD_APPENDED:
            BOARD_SHIELD_APPENDED = True
            self.__class__.__name__ = "%s_%s" % (testConfiguration.BOARD_NAME, self.__class__.__name__)
        self.output.testInit("ISSDK_Mpl3115Streamtest", "ISSDK Stream-Test for MPL3115.")

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
            print("\n\n****************** Executing Test for duration : %d minutes ******************\n" % MPL3115_TEST_DURATION)

            # Get samples from samples files.
            (sampleCount, sampleList) = simProtocolAdapter.encodeAddSampleCommands(MPL3115_SAMPLE_WIDTH, testConfiguration.MPL3115_8K_SAMPLE_FILE)
            self.output.assert_(sampleList, "Parsing Samples File Failed.", self.output.lineno())

            # Configure the Simulator for MPL3115
            cmdPacket = simProtocolAdapter.encodeConfigureCommand(MPL3115_SAMPLE_DEPTH, MPL3115_SLAVE_ADDERSS)
            self.driver.simPortWrite(cmdPacket)
            time.sleep(0.1) # Wait for Simulator to complete Configuration.
            print(self.driver.simPortRead())

            # Load the DUT image onto the DUT Board.
            testConfiguration.BOARD_OBJECT_HANDLE.loadDutImage(testConfiguration.MPL3115_STREAM_FILE)
            time.sleep(0.1) # Wait for DUT to start execution.

            t_end = time.time() + 60*MPL3115_TEST_DURATION
            while time.time() < t_end:
                print "\nTest time remaining : %d seconds" % (t_end - time.time())
                for repeat in range(MPL3115_TEST_SAMPLES/sampleCount):
                    # Keep Injecting samples to Simulator Ring Buffer.
                    for offset in range(sampleCount):
                        # Override the Temperature in the Sample to be in incremental sequence.
                        sampleList[offset][6] = ((sampleCount*repeat)+offset) & 0x00FF
                        sampleList[offset][5] = ((sampleCount*repeat)+offset) >> 8
                        self.driver.simPortWrite(sampleList[offset])
                    sys.stdout.write(self.driver.dutPortRead())
                    sys.stdout.write(self.driver.simPortRead())

            time.sleep(1) # Wait for DUT to complete execution.
            # Read the Test output.
            testOutput = self.driver.dutPortRead()
            print(testOutput)

            # Extract and Capture result for Streaming Tests.
            result = self.output.parseStreamtestResults(testOutput, MPL3115_TEST_SAMPLES)
            self.output.assert_(result, "Incorrect samples; Test Failed.", self.output.lineno())

            print("\n\n****************** Test Completed  ******************\n")


        except driverExceptions.DriverError:
            self.output.assert_(False, "DriverException: Error, method: setUp", self.output.lineno())
        except:
            self.output.assert_(False, "Exception: Default Exception Error.", self.output.lineno())

def suite():

    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(ISSDK_Mpl3115Streamtest))

    return suite

if __name__ == "__main__":
    unittest.main()

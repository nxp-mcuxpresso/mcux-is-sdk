'''
@author    : B35385
@name      : ISSDK_Host_Demo
@copyright : Copyright (c) 2017, NXP Semiconductors.

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
HOST_DEMO_RUN_TIME        = 5   # Time for example to run ADS and print result.
HOST_DEMO_WAIT_TIME       = 0.1  # Time for board to send response (MAX 100ms).
HOST_DEMO_SHIELD_NAME     = None
HOST_DEMO_ADS_SHIELD_NAME = None
BOARD_SHIELD_APPENDED     = False
HOST_DEV_INFO_CMD         = "\x7E\x60\x7E"

class ISSDK_Host_Demo(unittest.TestCase):

    output = outputFormat.OutputFormat()
    driver = serialComDrv.SerialComDrv()

    def setUp(self):
        global BOARD_SHIELD_APPENDED
        if not BOARD_SHIELD_APPENDED:
            BOARD_SHIELD_APPENDED = True
            self.__class__.__name__ = "%s_%s_%s" % (testConfiguration.BOARD_NAME, HOST_DEMO_SHIELD_NAME, self.__class__.__name__)
        try:
            # Open the connection to the Board first.
            print ("\n\nOpening Connection...")
            self.driver.openConnection(testConfiguration.BOARD_OBJECT_HANDLE)

        except driverExceptions.DriverError:
            print("DriverException: Error, method: setUp")
            self.output.Assert(False, "DriverException: Error, method: setUp", self.output.lineno())

    def tearDown(self):
        self.driver.closeConnection()
        time.sleep(3)

    def test_host_demo(self):
        global HOST_DEMO_SHIELD_NAME
        global HOST_DEMO_ADS_SHIELD_NAME
        self.driver.dutPortFlush()
        self.output.testInit("ISSDK_HOST_DEMO", "ISSDK Driver Examples for HOST_DEMO.")
        print("\n\n****************** Executing Test  ******************\n")
        try:
            # Load the DUT image onto the DUT Board.
            testConfiguration.BOARD_OBJECT_HANDLE.loadDutImage((testConfiguration.HOST_DEMO_FILE).replace("AGM01", HOST_DEMO_SHIELD_NAME))
            time.sleep(HOST_DEMO_RUN_TIME) # Wait for example to run.

            # Read the Test output.
            adsOutput = self.driver.dutPortRead()
            print(adsOutput)

            txMsg = []
            for char in (HOST_DEV_INFO_CMD):
                txMsg.append(ord(char))
            print("\n DEV INFO COMMAND  " + '[{}]'.format(', '.join(hex(x) for x in txMsg)))

            # Send Device Info Command to Board
            self.driver.dutPortWrite(HOST_DEV_INFO_CMD)
            time.sleep(HOST_DEMO_WAIT_TIME) # Wait for Board to send response.

            responseOutput = ''.join(self.driver.dutPortRead().split()).replace("Changed:", '')
            rxMsg = []
            for char in (responseOutput):
                rxMsg.append(ord(char))
            print(" DEV INFO RESPONSE " + '[{}]'.format(', '.join(hex(x) for x in rxMsg)))

            # Extract and Capture ADS processing results and Dev Info Response for Host Demo Tests.
            testResult = self.output.parseHostDemoResults(testConfiguration.BOARD_NAME_ADS, HOST_DEMO_ADS_SHIELD_NAME, adsOutput + responseOutput)
            self.output.assert_(testResult, "Test Failed.", self.output.lineno())

        except driverExceptions.DriverError:
            self.output.assert_(False, "DriverException: Error, method: setUp", self.output.lineno())
        except:
            self.output.assert_(False, "Exception: Default Exception Error.", self.output.lineno())

        print("\n\n****************** Test Completed  ******************\n")
        self.output.testEnd()

def suite(shield_name = "AGM01", ads_shield = "AGM01"):
    # Since these test may run on MULT2B also, we capture the Shield name from the suite file and update.
    global HOST_DEMO_SHIELD_NAME
    global HOST_DEMO_ADS_SHIELD_NAME
    HOST_DEMO_SHIELD_NAME = shield_name
    HOST_DEMO_ADS_SHIELD_NAME = ads_shield

    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(ISSDK_Host_Demo))

    return suite

if __name__ == "__main__":
    unittest.main()

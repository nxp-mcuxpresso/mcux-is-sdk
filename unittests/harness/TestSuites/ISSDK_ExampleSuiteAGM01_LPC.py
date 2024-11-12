'''
@author    : B35385
@name      : ISSDK_ExampleSuiteAGM01
@copyright : Copyright (c) 2016, NXP Semiconductors.

********************************************************************************
This is an automated test suite designed to run the ISSDK Driver Examples.
********************************************************************************
'''

import sys
import lpcBoardDrv
import serialComDrv
import outputFormat
import testConfiguration

output = outputFormat.OutputFormat()
output.suiteInit("ISSDK Driver Examples for AGM01 Sensor Board with LPC", "Test ISSDK Drivers")

import unittest
import xmlrunner
import ISSDK_Host_Demo
import ISSDK_AGM01IntExamples
import ISSDK_Fxos8700Examples
import ISSDK_Fxas21002Examples


ISSDKTestSuite = unittest.TestSuite()
ISSDKTestSuite.addTest(ISSDK_AGM01IntExamples.suite())
ISSDKTestSuite.addTest(ISSDK_Fxos8700Examples.suite())
ISSDKTestSuite.addTest(ISSDK_Fxas21002Examples.suite())
ISSDKTestSuite.addTest(ISSDK_Host_Demo.suite(shield_name = "AGM01", ads_shield = "AGM01"))

# Confirm intended Board Type
if testConfiguration.BOARD_NAME != "LPC54114" and testConfiguration.BOARD_NAME != "QN9080":
    print("This suite runs only on LPC54114 and QN9080")
    sys.exit()

# Auto select 1 COM Port for MBED based Boards.
lpcBoardDrv.BOARD_TYPE = testConfiguration.BOARD_NAME
lpcBoardDrv.DUT_COM_PORT_NAME = lpcBoardDrv.select_lpc_board_com_port(sequence=testConfiguration.DUT_PORT_SEQ)
print("Selected Comport : DUT = [%s]" % lpcBoardDrv.DUT_COM_PORT_NAME)

if testConfiguration.BAMBOO_REPORTS:
    # Use xmlrunner to enable bamboo patch to generate XML reports.
    xmlrunner.XMLTestRunner(output='test-reports', stream=sys.stdout, verbosity=0).run(ISSDKTestSuite)
else:
    unittest.TextTestRunner(verbosity=0).run(ISSDKTestSuite)

output.suiteEnd()
if testConfiguration.BAMBOO_REPORTS:
    serialComDrv.waitForDisconnection(lpcBoardDrv.DUT_COM_PORT_NAME)
sys.exit()

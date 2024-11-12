'''
@author    : B35385
@name      : ISSDK_ExampleSuiteAGMP03
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
output.suiteInit("ISSDK Driver Examples for AGMP03 Sensor Board with LPC", "Test ISSDK Drivers")

import unittest
import xmlrunner
import ISSDK_Host_Demo
import ISSDK_Fxls8962Examples
import ISSDK_Fxas21002Examples
import ISSDK_Fxas21002IntExamples
import ISSDK_Mag3110Examples
import ISSDK_Mpl3115Examples
import ISSDK_P3115Examples

ISSDKTestSuite = unittest.TestSuite()
ISSDKTestSuite.addTest(ISSDK_P3115Examples.suite(shield_name = "AGMP03-MPL3115"))
ISSDKTestSuite.addTest(ISSDK_Mpl3115Examples.suite(shield_name = "AGMP03-MPL3115"))
ISSDKTestSuite.addTest(ISSDK_Fxas21002IntExamples.suite(shield_name = "AGMP03"))
ISSDKTestSuite.addTest(ISSDK_Fxas21002Examples.suite(shield_name = "AGMP03"))
ISSDKTestSuite.addTest(ISSDK_Mag3110Examples.suite(shield_name = "AGMP03"))
ISSDKTestSuite.addTest(ISSDK_Fxls8962Examples.suite())
ISSDKTestSuite.addTest(ISSDK_Host_Demo.suite(shield_name = "AGMP03", ads_shield = "AGMP03"))

# Confirm intended Board Type
if testConfiguration.BOARD_NAME != "LPC54114":
    print("This suite runs only on LPC54114")
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

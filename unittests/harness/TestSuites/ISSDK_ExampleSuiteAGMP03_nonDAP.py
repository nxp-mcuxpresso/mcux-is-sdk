'''
@author    : B35385
@name      : ISSDK_ExampleSuiteAGMP03
@copyright : Copyright (c) 2016, NXP Semiconductors.

********************************************************************************
This is an automated test suite designed to run the ISSDK Driver Examples.
********************************************************************************
'''

import sys
import pneBoardDrv
import outputFormat
import serialComDrv
import testConfiguration

output = outputFormat.OutputFormat()
output.suiteInit("ISSDK Driver Examples for AGMP03 Sensor Board with non PnE MCU Boards", "Test ISSDK Drivers")

import unittest
import xmlrunner
import ISSDK_Fxls8962Examples
import ISSDK_Fxas21002Examples
import ISSDK_Fxas21002IntExamples
import ISSDK_Mag3110Examples
import ISSDK_Mpl3115Examples
import ISSDK_Host_Demo

ISSDKTestSuite = unittest.TestSuite()
ISSDKTestSuite.addTest(ISSDK_Mpl3115Examples.suite(shield_name = "AGMP03-MPL3115"))
ISSDKTestSuite.addTest(ISSDK_Fxas21002IntExamples.suite(shield_name = "AGMP03"))
ISSDKTestSuite.addTest(ISSDK_Fxas21002Examples.suite(shield_name = "AGMP03"))
ISSDKTestSuite.addTest(ISSDK_Mag3110Examples.suite(shield_name = "AGMP03"))
ISSDKTestSuite.addTest(ISSDK_Fxls8962Examples.suite())
ISSDKTestSuite.addTest(ISSDK_Host_Demo.suite(shield_name = "AGMP03", ads_shield = "AGMP03"))

# Confirm intended Board Type
if testConfiguration.BOARD_NAME != "KL27Z":
    print("This suite runs only on FRDM-KL27Z")
    sys.exit()

# Auto select 1 COM Port for PE Micro based Board.
pneBoardDrv.DUT_COM_PORT_NAME = pneBoardDrv.select_pne_board_com_port(sequence=testConfiguration.DUT_PORT_SEQ)
print("Selected Comport : DUT = [%s]" % pneBoardDrv.DUT_COM_PORT_NAME)

# Auto select 1 Drive for PE Micro based Board.
pneBoardDrv.DUT_BOARD_DRIVE = pneBoardDrv.select_pne_board_drive(sequence=testConfiguration.DUT_BOARD_SEQ)
print("Selected Drives  : DUT = [%s]" % pneBoardDrv.DUT_BOARD_DRIVE)

if testConfiguration.BAMBOO_REPORTS:
    # Use xmlrunner to enable bamboo patch to generate XML reports.
    xmlrunner.XMLTestRunner(output='test-reports', stream=sys.stdout, verbosity=0).run(ISSDKTestSuite)
else:
    unittest.TextTestRunner(verbosity=0).run(ISSDKTestSuite)

output.suiteEnd()
if testConfiguration.BAMBOO_REPORTS:
    serialComDrv.waitForDisconnection(pneBoardDrv.DUT_COM_PORT_NAME)
sys.exit()

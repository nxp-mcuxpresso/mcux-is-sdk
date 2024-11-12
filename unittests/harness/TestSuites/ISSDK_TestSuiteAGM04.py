'''
@author    : B35385
@name      : ISSDK_TestSuiteAGM04
@copyright : Copyright (c) 2016, NXP Semiconductors.

********************************************************************************
This is an automated test suite designed to run the ISSDK Driver Unittests.
********************************************************************************
'''

import sys
import outputFormat
import mbedBoardDrv
import testConfiguration

output = outputFormat.OutputFormat()
output.suiteInit("ISSDK Unittests for AGM04 Sensor Board", "Test ISSDK Drivers")

import unittest
import xmlrunner
import ISSDK_Mma865xUnittest
import ISSDK_Fxas21002Unittest
import ISSDK_Mag3110Unittest

ISSDKTestSuite = unittest.TestSuite()
ISSDKTestSuite.addTest(ISSDK_Mma865xUnittest.suite(shield_name = "AGM04"))
ISSDKTestSuite.addTest(ISSDK_Fxas21002Unittest.suite(shield_name = "AGM04"))
ISSDKTestSuite.addTest(ISSDK_Mag3110Unittest.suite(shield_name = "AGM04"))

# Confirm intended Board Type
if testConfiguration.BOARD_NAME != "K64F":
    print("This suite runs only on FRDM-K64F")
    sys.exit()

# Auto select 1 COM Port for MBED based Boards.
mbedBoardDrv.DUT_COM_PORT_NAME = mbedBoardDrv.select_mbed_board_com_port(sequence=testConfiguration.DUT_PORT_SEQ)
print("Selected Comport : DUT = [%s]" % mbedBoardDrv.DUT_COM_PORT_NAME)

# Auto select 1 MBED Board for DUT.
mbedBoardDrv.DUT_BOARD_HANDLE = mbedBoardDrv.MbedBoardDrv.InitBoard(sequence=testConfiguration.DUT_BOARD_SEQ)
print("Selected Board   : DUT = [%s]" % mbedBoardDrv.DUT_BOARD_HANDLE.getBoardName())

if testConfiguration.BAMBOO_REPORTS:
    # Use xmlrunner to enable bamboo patch to generate XML reports.
    xmlrunner.XMLTestRunner(output='test-reports', stream=sys.stdout, verbosity=0).run(ISSDKTestSuite)
else:
    unittest.TextTestRunner(verbosity=0).run(ISSDKTestSuite)

# Release DUT and SIM Boards.
mbedBoardDrv.UninitDutBoard()

output.suiteEnd()
sys.exit()

'''
@author    : B35385
@name      : ISSDK_TestSuiteSA9500
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
output.suiteInit("ISSDK Unittests for FRDM-STBC-SA9500 Sensor Board", "Test ISSDK Drivers")

import unittest
import xmlrunner
import ISSDK_Fxlc95000Unittests

ISSDKTestSuite = unittest.TestSuite()
ISSDKTestSuite.addTest(ISSDK_Fxlc95000Unittests.suite())

# Confirm intended Board Type
if testConfiguration.BOARD_NAME != "K22F":
    print("This suite run only on FRDM-K22F")
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

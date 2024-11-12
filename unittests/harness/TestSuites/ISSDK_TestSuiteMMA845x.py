'''
@author    : B35385
@name      : ISSDK_TestSuiteMMA845x
@copyright : Copyright (c) 2016, NXP Semiconductors.

********************************************************************************
This is an automated test suite designed to run the ISSDK Driver Unittests.
********************************************************************************
'''

import sys
import pneBoardDrv
import outputFormat
import testConfiguration

output = outputFormat.OutputFormat()
output.suiteInit("ISSDK Unittests for OnBoard Sensor MMA845x", "Test ISSDK Drivers")

import unittest
import xmlrunner
import ISSDK_Mma845xUnittest

ISSDKTestSuite = unittest.TestSuite()
ISSDKTestSuite.addTest(ISSDK_Mma845xUnittest.suite())

# Confirm intended Board Type
if testConfiguration.BOARD_NAME != "KL25Z" and testConfiguration.BOARD_NAME != "KL27Z":
    print("This suite runs only on FRDM-KL25Z and FRDM-KL27Z")
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
sys.exit()

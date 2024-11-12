'''
@author    : B35385
@name      : ISSDK_ExampleSuiteMult2B
@copyright : Copyright (c) 2016, NXP Semiconductors.

********************************************************************************
This is an automated test suite designed to run the ISSDK Driver Examples.
********************************************************************************
'''

import sys
import outputFormat
import mbedBoardDrv
import serialComDrv
import testConfiguration

output = outputFormat.OutputFormat()
output.suiteInit("ISSDK Driver Examples for MULT-2B Sensor Board", "Test ISSDK Drivers")

import unittest
import xmlrunner
import ISSDK_Host_Demo
import ISSDK_Mma865xExamples
import ISSDK_Mag3110Examples
import ISSDK_Mpl3115Examples
import ISSDK_Mma9553Examples
import ISSDK_Fxos8700Examples
import ISSDK_Fxas21002Examples
import ISSDK_Fxls8471qExamples

ISSDKTestSuite = unittest.TestSuite()
ISSDKTestSuite.addTest(ISSDK_Mpl3115Examples.suite(shield_name   = "MULT2B-MPL3115"))
ISSDKTestSuite.addTest(ISSDK_Fxls8471qExamples.suite(shield_name = "MULT2B-FXLS8471Q"))
ISSDKTestSuite.addTest(ISSDK_Fxos8700Examples.suite(shield_name  = "MULT2B"))
ISSDKTestSuite.addTest(ISSDK_Fxas21002Examples.suite(shield_name = "MULT2B"))
ISSDKTestSuite.addTest(ISSDK_Mma865xExamples.suite())
ISSDKTestSuite.addTest(ISSDK_Mag3110Examples.suite())
ISSDKTestSuite.addTest(ISSDK_Mma9553Examples.suite())
ISSDKTestSuite.addTest(ISSDK_Host_Demo.suite(shield_name = "MULT2B", ads_shield = "MULT2-B"))

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
if testConfiguration.BAMBOO_REPORTS:
    serialComDrv.waitForDisconnection(mbedBoardDrv.DUT_COM_PORT_NAME)
sys.exit()

'''
@author    : B35385
@name      : ISSDK_TestSuiteStreaming
@copyright : Copyright (c) 2016, NXP Semiconductors.

********************************************************************************
This is an automated test suite designed to run the ISSDK Streaming Test Cases.
********************************************************************************
'''

import sys
import outputFormat
import mbedBoardDrv
import testConfiguration

output = outputFormat.OutputFormat()
output.suiteInit("ISSDK Simulator Based Unit-Test Cases ", "Test ISSDK Drivers")

import unittest
import xmlrunner
import ISSDK_Mpl3115Streamtest


ISSDKTestSuite = unittest.TestSuite()
ISSDKTestSuite.addTest(ISSDK_Mpl3115Streamtest.suite())

# Confirm intended Board Type
if testConfiguration.BOARD_NAME != "K64F":
    print("This suite run only on FRDM-K64F")
    sys.exit()

# Auto select 2 COM Ports for MBED based Boards.
(mbedBoardDrv.SIM_COM_PORT_NAME, mbedBoardDrv.DUT_COM_PORT_NAME) = mbedBoardDrv.select_mbed_board_com_ports(sequenceSim=testConfiguration.SIM_PORT_SEQ, sequenceDut=testConfiguration.DUT_PORT_SEQ)
print("Selected COM Ports : SIM = [%s]       and DUT = [%s]" % (mbedBoardDrv.SIM_COM_PORT_NAME, mbedBoardDrv.DUT_COM_PORT_NAME))

# Auto select 2 MBED Boards for SIM and DUT.
(mbedBoardDrv.SIM_BOARD_HANDLE, mbedBoardDrv.DUT_BOARD_HANDLE) = mbedBoardDrv.MbedBoardDrv.InitBoards(sequenceSim=testConfiguration.SIM_BOARD_SEQ, sequenceDut=testConfiguration.DUT_BOARD_SEQ)
print("Selected Boards    : SIM = [%s]   and DUT = [%s]" % (mbedBoardDrv.SIM_BOARD_HANDLE.getBoardName(), mbedBoardDrv.DUT_BOARD_HANDLE.getBoardName()))

# Copy the Simulator image to the Simulator Board and reset it to start execution.
mbedBoardDrv.loadSimImage()

if testConfiguration.BAMBOO_REPORTS:
    # Use xmlrunner to enable bamboo patch to generate XML reports.
    xmlrunner.XMLTestRunner(output='test-reports', stream=sys.stdout, verbosity=0).run(ISSDKTestSuite)
else:
    unittest.TextTestRunner(verbosity=0).run(ISSDKTestSuite)

# Release DUT and SIM Boards.
mbedBoardDrv.UninitBoards()

output.suiteEnd()
sys.exit()

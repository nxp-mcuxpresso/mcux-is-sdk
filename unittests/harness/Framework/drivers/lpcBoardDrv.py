'''
@author    : B35385
@name      : lpcBoardDrv
@copyright : Copyright (c) 2017, NXP Semiconductors.

********************************************************************************
This is the LPC-LinkII Board handler file.
It defines APIs used for handling operations on the DUT board.
********************************************************************************
'''

import os
import serial.tools.list_ports


BOARD_TYPE = None
DUT_COM_PORT_NAME = None
LPC_LOADER_SCRIPT = "unittests\harness\\BambooScripts\LPCLoad.cmd"
QN_LOADER_SCRIPT = "unittests\harness\\BambooScripts\QNLoad.cmd"

def loadDutImage(flashFile):
    """
    This method copies the DUT flash file to the Board MSD.
    The arg 'flashFile' has to be the relative path to the file from directory of execution.
    """
    print "Loading DUT Image %s." %  flashFile
    if BOARD_TYPE == "QN9080":
        os.system(QN_LOADER_SCRIPT + " " + flashFile)
    elif BOARD_TYPE == "LPC54114":
        os.system(LPC_LOADER_SCRIPT + " " + flashFile)
    else:
        print "Error: Board not recognised!"


def select_lpc_board_com_port(sequence=0):
    """
    Returns COM Port ID for 1 LPC-LinkII Board.
    """
    pnePorts = []
    lpcLink2PortName = 'mbed Serial Port'
    dapLinkPortName = 'USB Serial Device'
    # In list of COM Ports look for mbed boards.
    for port in serial.tools.list_ports.comports():
        if port[1].startswith(lpcLink2PortName) or port[1].startswith(dapLinkPortName):
            pnePorts.append(port[0])

    if len(pnePorts) < 1:
        print("The framework needs 1 LPC-LinkII Board COM Port to be connected.")
        return None

    return pnePorts[sequence]


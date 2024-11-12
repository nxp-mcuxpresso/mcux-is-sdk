'''
@author    : B35385
@name      : mbedBoardDrv
@copyright : Copyright (c) 2016, NXP Semiconductors.

********************************************************************************
This is the MBED/DAPLINK Board handler file.
It defines APIs used for handling operations on the DUT and SIM boards.
********************************************************************************
'''

import testConfiguration
import serial.tools.list_ports
from pyOCD.board import MbedBoard


SIM_BOARD_HANDLE  = None
DUT_BOARD_HANDLE  = None
SIM_COM_PORT_NAME = None
DUT_COM_PORT_NAME = None


def loadDutImage(flashFile):
    """
    This method copies the DUT flash file to the Board MSD and then Resets it.
    The arg 'flashFile' has to be the relative path to the file from directory of execution.
    """
    print "Loading DUT Image %s." %  flashFile
    DUT_BOARD_HANDLE.flash.flashBinary(flashFile)
    DUT_BOARD_HANDLE.target.reset()

def loadSimImage():
    """
    This method copies the SIM flash file to the Board MSD and then Resets it.
    """
    print "Loading SIM Image %s." %  testConfiguration.K64F_SIM_FLASH_FILE
    SIM_BOARD_HANDLE.target.reset()
    SIM_BOARD_HANDLE.flash.flashBinary(testConfiguration.K64F_SIM_FLASH_FILE)
    SIM_BOARD_HANDLE.target.reset()

def UninitDutBoard():
    """
    This method uninitializes the DUT Board.
    """
    DUT_BOARD_HANDLE.uninit()

def UninitBoards():
    """
    This method uninitializes the DUT and SIM Boards.
    """
    SIM_BOARD_HANDLE.uninit()
    DUT_BOARD_HANDLE.uninit()

def select_mbed_board_com_port(sequence=0):
    """
    Returns COM Port ID for 1 MBED Board.
    sequence : sequence number of board to be selected as DUT.
    """
    mbedPorts = []
    mbedPortName = 'mbed Serial Port'
    dapLinkPortName = 'USB Serial Device'
    # In list of COM Ports look for mbed boards.
    for port in serial.tools.list_ports.comports():
        if port[1].startswith(mbedPortName) or port[1].startswith(dapLinkPortName):
            mbedPorts.append(port[0])

    if len(mbedPorts) < 1:
        print("The framework needs 1 MBED/DAPLINK Board COM Port to be connected.")
        return None

    return mbedPorts[sequence]

def select_mbed_board_com_ports(sequenceSim=0, sequenceDut=1):
    """
    Returns COM Port IDs for 2 MBED Boards.
    sequenceSim : sequence number of board to be selected as SIM.
    sequenceDut : sequence number of board to be selected as DUT.
    """
    mbedPorts = []
    mbedPortName = 'mbed Serial Port'
    dapLinkPortName = 'USB Serial Device'    
    # In list of COM Ports look for mbed boards.
    for port in serial.tools.list_ports.comports():
        if port[1].startswith(mbedPortName) or port[1].startswith(dapLinkPortName):
            mbedPorts.append(port[0])

    if len(mbedPorts) < 2:
        print("The framework needs 2 MBED/DAPLINK Board COM Ports to be connected.")
        return None

    return (mbedPorts[sequenceSim], mbedPorts[sequenceDut])

class MbedBoardDrv(MbedBoard):
    '''
    MBED Driver used to select 1, 2 or 3 Boards for DUT and Simulator(s) (parent driver does not support 1+ boards at a time) .
    '''

    @staticmethod
    def InitBoards(sequenceSim=0, sequenceDut=1):
        """
        Allows you to choose 2 boards among all boards connected.
        sequenceSim : sequence number of board to be selected as SIM.
        sequenceDut : sequence number of board to be selected as DUT.
        """
        all_mbeds = MbedBoard.getAllConnectedBoards()

        # Return if no or less than 2 boards are connected.
        if all_mbeds == None or len(all_mbeds) < 2:
            print("ERROR: This framework requires 2 MBED boards in MSD Mode to be connected.")
            return None

        # Select first 2 boards and close others if any.
        for i in range(0, len(all_mbeds)):
            if sequenceSim != i and sequenceDut != i:
                all_mbeds[i].interface.close()

        mbedSIM = all_mbeds[sequenceSim]
        mbedDUT = all_mbeds[sequenceDut]

        # Initialize the boards.
        try:
            mbedSIM.init()
            mbedDUT.init()
        except:
            mbedSIM.interface.close()
            mbedDUT.interface.close()
            raise

        return (mbedSIM, mbedDUT)

    @staticmethod
    def InitBoard(sequence=0):
        """
        Allow you to choose 1 boards among all boards connected.
        sequence : sequence number of board to be selected as DUT.
        """
        all_mbeds = MbedBoard.getAllConnectedBoards()

        # Return if less than 1 board is connected.
        if all_mbeds == None or len(all_mbeds) < 1:
            print("ERROR: This framework requires 1 MBED board in MSD Mode to be connected.")
            return None

        # Select matching board and close others if any.
        for i in range(0, len(all_mbeds)):
            if sequence != i:
                all_mbeds[i].interface.close()

        mbedDUT = all_mbeds[sequence]

        # Initialize the board.
        try:
            mbedDUT.init()
        except:
            mbedDUT.interface.close()
            raise

        return mbedDUT

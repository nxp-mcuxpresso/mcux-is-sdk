'''
@author    : B35385
@name      : pneBoardDrv
@copyright : Copyright (c) 2016, NXP Semiconductors.

********************************************************************************
This is the PE Micro Board handler file.
It defines APIs used for handling operations on the DUT board.
********************************************************************************
'''

import os
import string
import ctypes
import serial.tools.list_ports


DUT_BOARD_DRIVE   = None
DUT_COM_PORT_NAME = None


def loadDutImage(flashFile):
    """
    This method copies the DUT flash file to the Board MSD.
    The arg 'flashFile' has to be the relative path to the file from directory of execution.
    """
    print "Loading DUT Image %s." %  flashFile
    os.system("copy " + flashFile + DUT_BOARD_DRIVE)


def select_pne_board_com_port(sequence=0):
    """
    Returns COM Port ID for 1 PE Micro Board.
    """
    pnePorts = []

    # In list of COM Ports look for mbed boards.
    for port in serial.tools.list_ports.comports():
        if port[1].startswith('OpenSDA - CDC Serial Port'):
            pnePorts.append(port[0])

    if len(pnePorts) < 1:
        print("The framework needs 1 PE Micro Board COM Port to be connected.")
        return None

    return pnePorts[sequence]

def select_pne_board_drive(sequence=0):
    """
    Returns Drive Letters for 1 PE Micro Board.
    """
    drives = []
    volumeNameBuffer = ctypes.create_unicode_buffer(1024)
    fileSystemNameBuffer = ctypes.create_unicode_buffer(1024)
    serial_number = None
    max_component_length = None
    file_system_flags = None
    kernel32 = ctypes.windll.kernel32

    bitmask = kernel32.GetLogicalDrives()

    for letter in string.uppercase:
        if bitmask & 1:
            kernel32.GetVolumeInformationW(
                ctypes.c_wchar_p("%s:\\" % letter),
                volumeNameBuffer,
                ctypes.sizeof(volumeNameBuffer),
                serial_number,
                max_component_length,
                file_system_flags,
                fileSystemNameBuffer,
                ctypes.sizeof(fileSystemNameBuffer)
                )

            if volumeNameBuffer.value.startswith('FRDM'):
                drives.append(' ' + letter + ':')

        bitmask >>= 1

    if len(drives) < 1:
        print("The framework needs 1 PE Micro Board in Mass Storage Mode to be connected.")
        return None

    return drives[sequence]


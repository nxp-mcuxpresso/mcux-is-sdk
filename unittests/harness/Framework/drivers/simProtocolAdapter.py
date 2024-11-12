'''
@author    : B35385
@name      : simProtocolAdapter
@copyright : Copyright (c) 2016, NXP Semiconductors.

********************************************************************************
This is the Simulator Protocol Adapter file.
It defines APIs used for encoding commands to be sent to the Simulator over UART.
The Simulator accepts only 10-Byte padded messages over UART.

The currently list of commands are:
    1) Configure Command
    2) Add Sample Command

    Note: The Response from the SIM for these Commands is handled directly in the Serial Communication Driver.
********************************************************************************
'''

import re
import os
import testConfiguration


def encodeConfigureCommand(sampleDepth, slaveAddress):
    """
    This method encodes the Configure Command to be sent to the Simulator.
    ----------------------------------------------------
    | TAG | SlaveAddress | SampleDepth | Padding | EOL |
    ----------------------------------------------------
    | 1B  |      1B      |     2B      |    4B   |  2B |
    ----------------------------------------------------
    """
    strSampleDepth = "%04X" % sampleDepth
    strPadding     = ''.zfill(2*4)

    return bytearray.fromhex(testConfiguration.SIM_CFG_SLAVE_CMD_TAG + slaveAddress + strSampleDepth + strPadding + testConfiguration.SIM_WINDOWS_EOL_BYTES)

def encodeAddSampleCommands(sampleWidth, samplesFile):
    """
    This method encodes the Add Sample Commands to be sent to the Simulator.
    It returns the number of samples found and a list of formatted commands.
    The arg 'samplesFile' has to be the relative path to the file from directory of execution.
    Note: The length of a sample can me MAX 6 Bytes (with no-padding required).
          For length < 6 Bytes padding is to added at the end of the sample to make it 6 Bytes.
    ------------------------------------------------------
    | TAG |   Reserved   | Sample in HEX | Padding | EOL |
    ----------------------------------------------------
    | 1B  |      1B      |     xB        |    yB   |  2B |
    ------------------------------------------------------
    """
    offset = 0
    sampleList = []

    if(sampleWidth > testConfiguration.SIM_MAX_SAMPLE_WIDTH):
        print("ERROR: Sample Width [%d] greater than MAX Sample Width[%d]." % (sampleWidth, testConfiguration.SIM_MAX_SAMPLE_WIDTH))
        return None

    with open(os.getcwd() + samplesFile, 'r') as sampleFD:
        for line in sampleFD:
            sample = re.sub('[\s+]', '', line) # Remove Spaces.
            if len(sample) == 0 or sample == '' or sample[0] == '#': # Ignore blank lines and Comments.
                continue
            if len(sample) != 2*sampleWidth:
                print("ERROR: Unexpected sample in sample file at line[%d]." % (offset+1))
                return None
            strReserved = "%02X" % 0
            strPadding  = ''.zfill(2*(testConfiguration.SIM_MAX_SAMPLE_WIDTH - sampleWidth))
            addSamplePacket = bytearray.fromhex(testConfiguration.SIM_ADD_SAMPLE_CMD_TAG + strReserved + sample + strPadding + testConfiguration.SIM_WINDOWS_EOL_BYTES)
            sampleList.append(addSamplePacket)
            offset += 1

    print("Found [%d] Sample(s) in Sample file.\n" % offset)
    sampleFD.close()

    return (offset, sampleList)

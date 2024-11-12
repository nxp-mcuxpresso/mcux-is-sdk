'''
@author    : B35385
@name      : serialComDrv
@copyright : Copyright (c) 2016, NXP Semiconductors.

********************************************************************************
This is the serial simPort communication handler file.
It defines APIs used for initializing, reading and writing to and form the UART.
********************************************************************************
'''

import re
import time
import serial.tools.list_ports

def full_port_name(portname):
    """ Given a Port-name (of the form COM7,
        COM12, CNCA0, etc.) returns a full
        name suitable for opening with the
        Serial class.
    """
    m = re.match('^COM(\d+)$', portname)
    if m and int(m.group(1)) < 10:
        return portname
    return '\\\\.\\' + portname

def waitForDisconnection(portname):
    while True:
        connected = False
        for port_no, _, _ in list(serial.tools.list_ports.comports()):
            if port_no == portname:
                connected = True
        if not connected:
            break
        print("\nWaiting for DUT Port %s to be disconnected...\n" % portname)
        time.sleep(1)

    print("DUT Disconnected!\n\n")

class SerialComDrv(object):
    '''
    Serial Driver used to communicate with DUT/Simulator Boards
    '''
    sfsPort = serial.Serial()
    simPort = serial.Serial()
    dutPort = serial.Serial()

    DEBUG        = False
    MAX_TIMEOUT  = 1       # Max timeout of 1 seconds
    BAUDRATE     = 115200
    COMM_ERROR   = -1
    COMM_SUCCESS = 0
    SIM_RSP_TAG  = '\x06'  # ASCII for ACK is 0x06

    def __init__(self):
        '''
        Constructor
        '''

    def closeConnection(self):
        '''
        Closes connection to serial dutPort
        '''
        self.dutPort.close()
        print("\nConnection to DUT Board closed.\n")

    def closeConnections(self):
        '''
        Closes connection to serial simPort and dutPort
        '''
        self.simPort.close()
        self.dutPort.close()
        print("\nConnection to SIM and DUT Boards closed.\n")

    def openConnection(self, boardDrv):
        '''
        Establishes communication on a fixed COM dutPort
        '''

        self.dutPort.baudrate = self.BAUDRATE

        status = self.COMM_SUCCESS

        try:
            self.dutPort.port = full_port_name(boardDrv.DUT_COM_PORT_NAME)
            self.dutPort.timeout = self.MAX_TIMEOUT
            self.dutPort.open()
            print("\nConnection to DUT established successfully.\n")

        except:
            print("Connection Error. Unable to establish connection to board.\n")
            raise

        return status

    def openConnections(self, boardDrv):
        '''
        Establishes communication on fixed COM dutPort and simPort
        '''

        self.simPort.baudrate = self.BAUDRATE
        self.dutPort.baudrate = self.BAUDRATE

        status = self.COMM_SUCCESS

        try:
            self.simPort.port = full_port_name(boardDrv.SIM_COM_PORT_NAME)
            self.simPort.timeout = self.MAX_TIMEOUT
            self.simPort.open()
            print("\nConnection to Simulator established successfully.\n")

            self.dutPort.port = full_port_name(boardDrv.DUT_COM_PORT_NAME)
            self.dutPort.timeout = self.MAX_TIMEOUT
            self.dutPort.open()
            print("\nConnection to DUT established successfully.\n")

        except:
            print("Connection Error. Unable to establish connection to boards.\n")
            raise

        return status

    def simPortFlush(self):
        self.simPort.flushInput()
        self.simPort.flushOutput()

    def dutPortFlush(self):
        self.dutPort.flushInput()

    def simPortWrite(self, buf, waitForResponse=True):
        '''
        Writes specified number of bytes to the slave simPort and waits for ACK.
        byteCount  -- number of bytes
        Ack response Format:
            -------
            | TAG |
            -------
            | 1B  |
            -------
        '''
        if (len(buf) > 0):
            bytesWritten = self.simPort.write(buf)
            if (waitForResponse):
                ackByte = self.simPort.read(1) # Wait for 1 Byte response.
                if(ackByte != self.SIM_RSP_TAG):
                    print ("Failed to receive ACK from SIM.")
                    return self.COMM_ERROR
            return bytesWritten
        else:
            return 0

    def simPortRead(self, length=0):
        '''
        Reads the next packet up length/available size and removes non-ascii characters.
        '''
        if (length > 0):  # Blocking read for length bytes.
            readBytes = self.simPort.read(length)
        else:  # Non-Blocking read for received Bytes.
            readBytes = self.simPort.read(self.simPort.inWaiting())

        # Skip non-ascii characters in DUT output.
        for character in readBytes:
            if ord(character) > 127:
                readBytes = readBytes.replace(character, '')

        return readBytes

    def dutPortRead(self):
        '''
        Reads the next packet up available size and removes non-ascii characters.
        '''
        readBytes = self.dutPort.read(self.dutPort.inWaiting()) # Non-Blocking read for pending Bytes.
        # Skip non-ascii characters in DUT output.
        for character in readBytes:
            if ord(character) > 127:
                readBytes = readBytes.replace(character, '')

        return readBytes

    def dutPortWrite(self, buf):
        '''
        Writes the specified buffer to the dutPort.
        '''
        self.dutPort.write(buf)

    def dutPortWriteResetSF(self):
        '''
        Writes the Reset Command for SF to the slave dutPort.
        Command Format:
            -------------------------------------
           s| '~' | 'R' | 'S' | 'T' | ' ' | '~' |
            -------------------------------------
          0x|  7E |  52 |  53 |  54 |  20 |  7E |
            -------------------------------------
        '''
        self.sfsPort.write(bytearray.fromhex('7E 52 53 54 20 7E'))

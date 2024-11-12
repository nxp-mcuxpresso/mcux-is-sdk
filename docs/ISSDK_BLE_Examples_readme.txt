Feature List:-
------------------------------
1) The application implements a custom GATT based Wireless UART Profile that emulates UART over BLE.
2) The application also implements a Sensor Driver Interface for the Sensors connected to the Board. 
3) The example provides an interface through the Wireless UART Application to Read Sensor Data form Sensors.
4) The example provides an interface through the Wireless UART Application to Read and Write Registers of the Sensors.
5) For a list of supported Commands which can be entered through the Client Application, refer to the list below.
6) The Client Application "NXP IoT Toolbox" is available for iOS and Android (use Wireless UART Example from the APP).
7) For more information regarding the Wireless UART Application, please consult the BLE Demo Applications User's Guide.

How to Run:-
----------
01) Open the Project
02) Build and Load the Firmware on the Target Board by using the green DEBUG button
03) When Launched, the LEDs on the Board will start blinking
04) Press SW2 on the Board to Start Advertising, the LEDs will change pattern 
05) Launch the 'NXP IoT Toolbox' APP on your Handheld device and open 'Wireless UART Example'
06) When the APP detects and shows the Board, select it to connect.
07) By default system is in Standby in Data Format will be Converted.
08) Enter any Command from the List below to change the output as desired.
09) All the Operating Modes in the list below may not be supported by the every Firmware.
10) To build Firmware for a given example (source/issdk_xxx), include the appropriate example directory and exclude the others from build.   

List of User Interface Commands and their functions:-
---------------------------------------------------
1) Sensor Operation Mode Commands:
   Format = <TAG><D><MODE>
   TAG    = 'CMD'           (3 Fixed Characters)
   MODE   = 'STANDBY'       (7 Fixed Characters to Stop All Sensor Data)
   MODE   = 'ACCELEROMETER' (13 Fixed Characters to Start Accelerometer Data)
   MODE   = 'MAGNETOMETER'  (12 Fixed Characters to Start Magnetometer Data)
   MODE   = 'GYROSCOPE'     (9 Fixed Characters to Start Gyroscope Data)
   MODE   = 'BAROMETER'     (9 Fixed Characters to Start Barometer Data)
   MODE   = 'ALTIMETER'     (9 Fixed Characters to Start Altimeter Data)
   MODE   = 'THERMOMETER'   (11 Fixed Characters to Start Temperature Data)
   MODE   = 'THRESHOLD'     (9 Fixed Characters to Start Threshold detection)
   MODE   = 'FREEFALL'      (8 Fixed Characters to Start Freefall detection)
   MODE   = 'PEDOMETER'     (9 Fixed Characters to Pedometer Data capture)
   MODE   = 'ORIENTATION'   (11 Fixed Characters to Start Orientation detection)
   MODE   = 'CUSTOM'        (6 Fixed Characters to Start Custom detection)
   D      = ' '             (Delimiter : Any Single Character like ' ' Space or ',' Comma or ':' Colon...)

2) Sensor Data Format Commands:
   Format = <TAG><D><MODE>
   TAG    = 'CMD'           (3 Fixed Characters)
   MODE   = 'NORMAL'        (6 Fixed Characters to view Converted Sensor Data)
   MODE   = 'RAW'           (3 Fixed Characters to view Raw Sensor Data)
   MODE   = 'STREAM'        (6 Fixed Characters to Start Streaming Data)
   D      = ' '             (Delimiter : Any Single Character like ' ' Space or ',' Comma or ':' Colon...)

3) Sensor Register Access Commands:
   Format = <TAG><D><DEVICE><D><ACTION><D><OFFSET><D><BYTES>
   TAG    = 'RLI'      (3 Fixed Characters)
   DEVICE = 'A'        ('A' = ACCEL or 'M' = MAG or 'G' = GYRO or 'P' = PRESSURE)
   ACTION = 'R'        ('R' = READ or 'W' = WRITE)
   OFFSET = 'XX'       (00-7F in HEX for Register Start OFFSET)
   BYTES  = 'YY'       (00-7F in HEX for Bytes to READ)
   BYTES  = 'ZZZZZZZZ' (00-7F in HEX and up to 4 Bytes MAX to WRITE)
   D      = ' '        (Delimiter : Any Single Character like ' ' Space or ',' Comma or ':' Colon...)

/*
* Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*! \file DecodeCommandBytes.c
    \brief Command interpreter which interfaces to the Sensor Fusion Toolbox
*/

#include "anomaly_detection.h"
#include "control.h"
#include "fusion.h"
#include "calibration_storage.h"
// All commands for the command interpreter are exactly 4 characters long.
// The command interpeter converts the incoming packet to a 32-bit integer, which is then
// the index into a large switch statement for processing commands.
// The following block of #define statements are responsible for the conversion from 4-characters
// into an easier to use integer format.
#define cmd_VGplus      (((((('V' << 8) | 'G') << 8) | '+') << 8) | ' ') // "VG+ " = enable angular velocity packet transmission
#define cmd_VGminus     (((((('V' << 8) | 'G') << 8) | '-') << 8) | ' ') // "VG- " = disable angular velocity packet transmission
#define cmd_DBplus      (((((('D' << 8) | 'B') << 8) | '+') << 8) | ' ') // "DB+ " = enable debug packet transmission
#define cmd_DBminus     (((((('D' << 8) | 'B') << 8) | '-') << 8) | ' ') // "DB- " = disable debug packet transmission
#define cmd_Q3          (((((('Q' << 8) | '3') << 8) | ' ') << 8) | ' ') // "Q3  " = transmit 3-axis accelerometer quaternion in standard packet
#define cmd_Q3M         (((((('Q' << 8) | '3') << 8) | 'M') << 8) | ' ') // "Q3M " = transmit 3-axis magnetic quaternion in standard packet
#define cmd_Q3G         (((((('Q' << 8) | '3') << 8) | 'G') << 8) | ' ') // "Q3G " = transmit 3-axis gyro quaternion in standard packet
#define cmd_Q6MA        (((((('Q' << 8) | '6') << 8) | 'M') << 8) | 'A') // "Q6MA" = transmit 6-axis mag/accel quaternion in standard packet
#define cmd_Q6AG        (((((('Q' << 8) | '6') << 8) | 'A') << 8) | 'G') // "Q6AG" = transmit 6-axis accel/gyro quaternion in standard packet
#define cmd_Q9          (((((('Q' << 8) | '9') << 8) | ' ') << 8) | ' ') // "Q9  " = transmit 9-axis quaternion in standard packet (default)
#define cmd_RPCplus     (((((('R' << 8) | 'P') << 8) | 'C') << 8) | '+') // "RPC+" = Roll/Pitch/Compass on
#define cmd_RPCminus    (((((('R' << 8) | 'P') << 8) | 'C') << 8) | '-') // "RPC-" = Roll/Pitch/Compass off
#define cmd_ALTplus     (((((('A' << 8) | 'L') << 8) | 'T') << 8) | '+') // "ALT+" = Altitude packet on
#define cmd_ALTminus    (((((('A' << 8) | 'L') << 8) | 'T') << 8) | '-') // "ALT-" = Altitude packet off
#define cmd_RST         (((((('R' << 8) | 'S') << 8) | 'T') << 8) | ' ') // "RST " = Soft reset
#define cmd_RINS        (((((('R' << 8) | 'I') << 8) | 'N') << 8) | 'S') // "RINS" = Reset INS inertial navigation velocity and position
#define cmd_SVAC        (((((('S' << 8) | 'V') << 8) | 'A') << 8) | 'C') // "SVAC" = save all calibrations to Kinetis flash
#define cmd_SVMC        (((((('S' << 8) | 'V') << 8) | 'M') << 8) | 'C') // "SVMC" = save magnetic calibration to Kinetis flash
#define cmd_SVYC        (((((('S' << 8) | 'V') << 8) | 'Y') << 8) | 'C') // "SVYC" = save gyroscope calibration to Kinetis flash
#define cmd_SVGC        (((((('S' << 8) | 'V') << 8) | 'G') << 8) | 'C') // "SVGC" = save precision accelerometer calibration to Kinetis flash
#define cmd_ERAC        (((((('E' << 8) | 'R') << 8) | 'A') << 8) | 'C') // "ERAC" = erase all calibrations from Kinetis flash
#define cmd_ERMC        (((((('E' << 8) | 'R') << 8) | 'M') << 8) | 'C') // "ERMC" = erase magnetic calibration from Kinetis flash
#define cmd_ERYC        (((((('E' << 8) | 'R') << 8) | 'Y') << 8) | 'C') // "ERYC" = erase gyro offset calibration from Kinetis flash
#define cmd_ERGC        (((((('E' << 8) | 'R') << 8) | 'G') << 8) | 'C') // "ERGC" = erase precision accelerometer calibration from Kinetis flash
#define cmd_180X        (((((('1' << 8) | '8') << 8) | '0') << 8) | 'X') // "180X" perturbation
#define cmd_180Y        (((((('1' << 8) | '8') << 8) | '0') << 8) | 'Y') // "180Y" perturbation
#define cmd_180Z        (((((('1' << 8) | '8') << 8) | '0') << 8) | 'Z') // "180Z" perturbation
#define cmd_M90X        (((((('M' << 8) | '9') << 8) | '0') << 8) | 'X') // "M90X" perturbation
#define cmd_P90X        (((((('P' << 8) | '9') << 8) | '0') << 8) | 'X') // "P90X" perturbation
#define cmd_M90Y        (((((('M' << 8) | '9') << 8) | '0') << 8) | 'Y') // "M90Y" perturbation
#define cmd_P90Y        (((((('P' << 8) | '9') << 8) | '0') << 8) | 'Y') // "P90Y" perturbation
#define cmd_M90Z        (((((('M' << 8) | '9') << 8) | '0') << 8) | 'Z') // "M90Z" perturbation
#define cmd_P90Z        (((((('P' << 8) | '9') << 8) | '0') << 8) | 'Z') // "P90Z" perturbation
#define cmd_PA00        (((((('P' << 8) | 'A') << 8) | '0') << 8) | '0') // "PA00" average precision accelerometer location 0
#define cmd_PA01        (((((('P' << 8) | 'A') << 8) | '0') << 8) | '1') // "PA01" average precision accelerometer location 1
#define cmd_PA02        (((((('P' << 8) | 'A') << 8) | '0') << 8) | '2') // "PA02" average precision accelerometer location 2
#define cmd_PA03        (((((('P' << 8) | 'A') << 8) | '0') << 8) | '3') // "PA03" average precision accelerometer location 3
#define cmd_PA04        (((((('P' << 8) | 'A') << 8) | '0') << 8) | '4') // "PA04" average precision accelerometer location 4
#define cmd_PA05        (((((('P' << 8) | 'A') << 8) | '0') << 8) | '5') // "PA05" average precision accelerometer location 5
#define cmd_PA06        (((((('P' << 8) | 'A') << 8) | '0') << 8) | '6') // "PA06" average precision accelerometer location 6
#define cmd_PA07        (((((('P' << 8) | 'A') << 8) | '0') << 8) | '7') // "PA07" average precision accelerometer location 7
#define cmd_PA08        (((((('P' << 8) | 'A') << 8) | '0') << 8) | '8') // "PA08" average precision accelerometer location 8
#define cmd_PA09        (((((('P' << 8) | 'A') << 8) | '0') << 8) | '9') // "PA09" average precision accelerometer location 9
#define cmd_PA10        (((((('P' << 8) | 'A') << 8) | '1') << 8) | '0') // "PA10" average precision accelerometer location 10
#define cmd_PA11        (((((('P' << 8) | 'A') << 8) | '1') << 8) | '1') // "PA11" average precision accelerometer location 11

void DecodeCommandBytes(Globals *gbls, char iCommandBuffer[], uint8 sUART_InputBuffer[], uint16 nbytes)
{
	int32 isum;		// 32 bit command identifier
	int16 i, j;		// loop counters

	// parse all received bytes in sUARTInputBuf into the iCommandBuffer delay line
	for (i = 0; i < nbytes; i++) {
		// shuffle the iCommandBuffer delay line and add the new command byte
		for (j = 0; j < 3; j++)
			iCommandBuffer[j] = iCommandBuffer[j + 1];
		iCommandBuffer[3] = sUART_InputBuffer[i];

		// check if we have a valid command yet
		isum = ((((((int32)iCommandBuffer[0] << 8) | iCommandBuffer[1]) << 8) | iCommandBuffer[2]) << 8) | iCommandBuffer[3];
		switch (isum) 		{
		case cmd_VGplus: // "VG+ " = enable angular velocity packet transmission
                    gbls->pControlSubsystem->AngularVelocityPacketOn = true;
                    iCommandBuffer[3] = '~';
		break;

		case cmd_VGminus: // "VG- " = disable angular velocity packet transmission
                    gbls->pControlSubsystem->AngularVelocityPacketOn = false;
                    iCommandBuffer[3] = '~';
		break;

		case cmd_DBplus: // "DB+ " = enable debug packet transmission
                    gbls->pControlSubsystem->DebugPacketOn = true;
                    iCommandBuffer[3] = '~';
		break;

		case cmd_DBminus: // "DB- " = disable debug packet transmission
                    gbls->pControlSubsystem->DebugPacketOn = false;
                    iCommandBuffer[3] = '~';
		break;

		case cmd_RPCplus: // "RPC+" = Roll/Pitch/Compass on
                    gbls->pControlSubsystem->RPCPacketOn = true;
                    iCommandBuffer[3] = '~';
		break;

		case cmd_RPCminus: // "RPC-" = Roll/Pitch/Compass off
                    gbls->pControlSubsystem->RPCPacketOn = false;
                    iCommandBuffer[3] = '~';
		break;

		case cmd_ALTplus: // "ALT+" = Altitude packet on
                    gbls->pControlSubsystem->AltPacketOn = true;
                    iCommandBuffer[3] = '~';
		break;

		case cmd_ALTminus: // "ALT-" = Altitude packet off
                    gbls->pControlSubsystem->AltPacketOn = false;
                    iCommandBuffer[3] = '~';
		break;

		case cmd_RST: // "RST " = Soft reset
                    // reset sensor fusion

                    // reset magnetic calibration and magnetometer data buffer
                    iCommandBuffer[3] = '~';
		break;


		case cmd_SVAC: // "SVAC" = save all calibrations to Kinetis flash
                    SaveMagCalibrationToNVM(gbls);
                    SaveGyroCalibrationToNVM(gbls);
                    SaveAccelCalibrationToNVM(gbls);
                    iCommandBuffer[3] = '~';
		break;

		case cmd_SVMC: // "SVMC" = save magnetic calibration to Kinetis flash
                    SaveMagCalibrationToNVM(gbls);
                    iCommandBuffer[3] = '~';
		break;

		case cmd_SVYC: // "SVYC" = save gyroscope calibration to Kinetis flash
                    SaveGyroCalibrationToNVM(gbls);
                    iCommandBuffer[3] = '~';
		break;

		case cmd_SVGC: // "SVGC" = save precision accelerometer calibration to Kinetis flash
                    SaveAccelCalibrationToNVM(gbls);
                    iCommandBuffer[3] = '~';
		break;

		case cmd_ERAC: // "ERAC" = erase all calibrations from Kinetis flash
                    EraseMagCalibrationFromNVM();
                    EraseGyroCalibrationFromNVM();
                    EraseAccelCalibrationFromNVM();
                    iCommandBuffer[3] = '~';
		break;

		case cmd_ERMC: // "ERMC" = erase magnetic calibration offset 0 bytes from Kinetis flash
                    EraseMagCalibrationFromNVM();
                    iCommandBuffer[3] = '~';
		break;

		case cmd_ERYC: // "ERYC" = erase gyro offset calibrationoffset 128 bytes from Kinetis flash
                    EraseGyroCalibrationFromNVM();
                    iCommandBuffer[3] = '~';
		break;

		case cmd_ERGC: // "ERGC" = erase precision accelerometer calibration offset 192 bytesfrom Kinetis flash
                    EraseAccelCalibrationFromNVM();
                    iCommandBuffer[3] = '~';
		break;

		default:
			// no action
			break;
		}
	} // end of loop over received characters

	return;
}

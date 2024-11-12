/*
* Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/


#ifndef CALSTORE_H
#define CALSTORE_H

  
/*! \file calibration_storage.h
    \brief Provides functions to store calibration to NVM
   
    Users who are not using NXP hardware will need to supply their own drivers
    in place of those defined here.
*/

void SaveMagCalibrationToNVM(Globals *gbls);
void SaveGyroCalibrationToNVM(Globals *gbls);
void SaveAccelCalibrationToNVM(Globals *gbls);
void EraseMagCalibrationFromNVM(void);
void EraseGyroCalibrationFromNVM(void);
void EraseAccelCalibrationFromNVM(void);
#endif

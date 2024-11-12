/*
 * Copyright (c) 2013-2016 ARM Limited. All rights reserved.
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*! File: Driver_I2C_SDK2.h
* @brief The \b Driver_I2C_SDK2.h file declares instances of I2C drivers for I2C0 and I2C1.
*/

#ifndef __DRIVER_I2C_SDK2_H__
#define __DRIVER_I2C_SDK2_H__
#include "Driver_I2C.h"

extern ARM_DRIVER_I2C Driver_I2C0_KSDK2_Blocking;
extern ARM_DRIVER_I2C Driver_I2C0_KSDK2_NonBlocking;

extern ARM_DRIVER_I2C Driver_I2C1_KSDK2_Blocking;
extern ARM_DRIVER_I2C Driver_I2C1_KSDK2_NonBlocking;

#endif // __DRIVER_I2C_SDK2_H__

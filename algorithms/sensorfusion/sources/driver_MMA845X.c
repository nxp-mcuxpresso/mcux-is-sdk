/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file driver_MMA845x.c
    \brief Provides init() and read() functions for the MMA845x 3-axis accel family.

    Supports MMA8451Q, MMA8452Q and MMA8453Q.  Key differences which are applicable
    to this driver are shown in the table below.

    Feature | MMA8451Q | MMA8452Q | MMA8453Q
    --------|----------|----------|---------
    # Bits  | 14       | 12       | 10
    FIFO    | 32-deep  | NONE     | NONE

    All three have the MSB of the result registers located in the same location,
    so if we read as one 16-bit value, the only thing that should change g/count
    will be which range (+/- 2/4/8g) we are on.
*/

#include "board.h"                      // generated by Kinetis Expert.  Long term - merge sensor_board.h into this file
#include "sensor_fusion.h"              // Sensor fusion structures and types
#include "sensor_drv.h"
#include "sensor_io_i2c.h"              // Required for registerreadlist_t / registerwritelist_t declarations
#include "MMA845x.h"                    // describes the MMA845x register definition and its bit mask
#include "drivers.h"                    // Device specific drivers supplied by NXP (can be replaced with user drivers)
#define MMA845x_COUNTSPERG     8192.0
#define MMA8451_ACCEL_FIFO_SIZE 32

#if F_USING_ACCEL

// Command definition to read the WHO_AM_I value.
const registerreadlist_t    MMA845x_WHO_AM_I_READ[] =
{
    { .readFrom = MMA845x_WHO_AM_I, .numBytes = 1 }, __END_READ_DATA__
};

// Command definition to read the number of entries in the accel FIFO.
const registerreadlist_t    MMA845x_F_STATUS_READ[] =
{
    { .readFrom = MMA845x_STATUS, .numBytes = 1 }, __END_READ_DATA__
};

// Command definition to read the number of entries in the accel FIFO.
registerreadlist_t          MMA845x_DATA_READ[] =
{
    { .readFrom = MMA845x_OUT_X_MSB, .numBytes = 6 }, __END_READ_DATA__
};

// Each entry in a RegisterWriteList is composed of: register address, value to write, bit-mask to apply to write (0 enables)
const registerwritelist_t   MMA8451_Initialization[] =
{
    // OK: write 0000 0000 = 0x00 to CTRL_REG1 to place MMA845x into standby
    // [7-1] = 0000 000
    // [0]: active=0
    { MMA845x_CTRL_REG1, 0x00, 0x00 },

    // Not applicable to the 8452/3.
    // OK: write 0100 0000 = 0x40 to F_SETUP to enable FIFO in continuous (circular) mode
    // [7-6]: F_MODE[1-0]=01 for FIFO continuous mode
    // [5-0]: F_WMRK[5-0]=000000 for no FIFO watermark
    { MMA845x_F_SETUP, 0x40, 0x00 },

    // OK: write 0000 0001 = 0x01 to XYZ_DATA_CFG register to set g range
    // [7-5]: reserved=000
    // [4]: HPF_OUT=0
    // [3-2]: reserved=00
    // [1-0]: FS=01 for +/-4g: 512 counts / g = 8192 counts / g after 4 bit left shift
    { MMA845x_XYZ_DATA_CFG, 0x01, 0x00 },

    // OK: write 0000 0010 = 0x02 to CTRL_REG2 to set MODS bits
    // [7]: ST=0: self test disabled
    // [6]: RST=0: reset disabled
    // [5]: unused
    // [4-3]: SMODS=00
    // [2]: SLPE=0: auto sleep disabled
    // [1-0]: mods=10 for high resolution (maximum over sampling)
    { MMA845x_CTRL_REG2, 0x02, 0x00 },

    // write 00XX X001 to CTRL_REG1 to set data rate and exit standby
    // [7-6]: aslp_rate=00
    // [5-3]: dr=111 for 1.56Hz data rate giving 0x39
    // [5-3]: dr=110 for 6.25Hz data rate giving 0x31
    // [5-3]: dr=101 for 12.5Hz data rate giving 0x29
    // [5-3]: dr=100 for 50Hz data rate giving 0x21
    // [5-3]: dr=011 for 100Hz data rate giving 0x19
    // [5-3]: dr=010 for 200Hz data rate giving 0x11
    // [5-3]: dr=001 for 400Hz data rate giving 0x09
    // [5-3]: dr=000 for 800Hz data rate giving 0x01
    // [2]: unused=0
    // [1]: f_read=0 for normal 16 bit reads
    // [0]: active=1 to take the part out of standby and enable sampling
#if (ACCEL_ODR_HZ <= 1)
    { MMA845x_CTRL_REG1, 0x39, 0x00 }, // select 1.5625Hz ODR,
#elif (ACCEL_ODR_HZ <= 6)
    { MMA845x_CTRL_REG1, 0x31, 0x00 }, // select 6.25Hz ODR
#elif (ACCEL_ODR_HZ <= 12)
    { MMA845x_CTRL_REG1, 0x29, 0x00 }, // select 12.5Hz ODR
#elif (ACCEL_ODR_HZ <= 50)
    { MMA845x_CTRL_REG1, 0x21, 0x00 }, // select 50Hz ODR
#elif (ACCEL_ODR_HZ <= 100)
    { MMA845x_CTRL_REG1, 0x19, 0x00 }, // select 100Hz ODR
#elif (ACCEL_ODR_HZ <= 200)
    { MMA845x_CTRL_REG1, 0x11, 0x00 }, // select 200Hz ODR
#elif (ACCEL_ODR_HZ <= 400)
    { MMA845x_CTRL_REG1, 0x09, 0x00 }, // select 400Hz ODR
#else
    { MMA845x_CTRL_REG1, 0x01, 0x00 }, // select 800Hz ODR
#endif
    __END_WRITE_DATA__
};

// MMA845x_Initialization is exactly the same as the above, but without
// the FIFO initialization.
const registerwritelist_t   MMA845x_Initialization[] =
{
    { MMA845x_CTRL_REG1, 0x00, 0x00 },
    { MMA845x_XYZ_DATA_CFG, 0x01, 0x00 },
    { MMA845x_CTRL_REG2, 0x02, 0x00 },
#if (ACCEL_ODR_HZ <= 1)
    { MMA845x_CTRL_REG1, 0x39, 0x00 }, // select 1.5625Hz ODR,
#elif (ACCEL_ODR_HZ <= 6)
    { MMA845x_CTRL_REG1, 0x31, 0x00 }, // select 6.25Hz ODR
#elif (ACCEL_ODR_HZ <= 12)
    { MMA845x_CTRL_REG1, 0x29, 0x00 }, // select 12.5Hz ODR
#elif (ACCEL_ODR_HZ <= 50)
    { MMA845x_CTRL_REG1, 0x21, 0x00 }, // select 50Hz ODR
#elif (ACCEL_ODR_HZ <= 100)
    { MMA845x_CTRL_REG1, 0x19, 0x00 }, // select 100Hz ODR
#elif (ACCEL_ODR_HZ <= 200)
    { MMA845x_CTRL_REG1, 0x11, 0x00 }, // select 200Hz ODR
#elif (ACCEL_ODR_HZ <= 400)
    { MMA845x_CTRL_REG1, 0x09, 0x00 }, // select 400Hz ODR
#else
    { MMA845x_CTRL_REG1, 0x01, 0x00 }, // select 800Hz ODR
#endif
    __END_WRITE_DATA__
};

// All sensor drivers and initialization functions have the same prototype.
// sfg is a pointer to the master "global" sensor fusion structure.
// sensor = pointer to linked list element used by the sensor fusion subsystem to specify required sensors

// sfg = pointer to top level (generally global) data structure for sensor fusion
int8_t MMA845x_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg)
{
    int32_t status;
    uint8_t reg;
    status = Register_I2C_Read(sensor->bus_driver, &sensor->deviceInfo, sensor->addr, MMA845x_WHO_AM_I, 1, &reg);
    if (status==SENSOR_ERROR_NONE) {
        sfg->Accel.iWhoAmI = reg;
        switch (reg) {
        case MMA8451_WHO_AM_I_WHOAMI_VALUE:
        case MMA8452_WHO_AM_I_WHOAMI_VALUE:
        case MMA8453_WHO_AM_I_WHOAMI_VALUE: break;
        default: return(SENSOR_ERROR_INIT);
        }
    } else {
       return(status);
    }

    // Configure and start the MMA845x sensor.  This does multiple register writes
    // (see MMA845x_Initialization definition above)
    if (reg==MMA8451_WHO_AM_I_WHOAMI_VALUE) {
        status = Sensor_I2C_Write(sensor->bus_driver, &sensor->deviceInfo, sensor->addr, MMA8451_Initialization );
    } else {
        status = Sensor_I2C_Write(sensor->bus_driver, &sensor->deviceInfo, sensor->addr, MMA845x_Initialization );
    }

    // Stash some needed constants in the SF data structure for this sensor
    sfg->Accel.iCountsPerg = MMA845x_COUNTSPERG;
    sfg->Accel.fgPerCount = 1.0F / MMA845x_COUNTSPERG;
    sfg->Accel.fgPerCount = 1.0F / MMA845x_COUNTSPERG;

    sensor->isInitialized = F_USING_ACCEL;
    sfg->Accel.isEnabled = true;

    return (status);
}

int8_t MMA845x_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg)
{
    uint8_t                     I2C_Buffer[6 * MMA8451_ACCEL_FIFO_SIZE];    // I2C read buffer
    int8_t                      status;         // I2C transaction status
    int8_t                      j;              // scratch
    uint8_t                     sensor_fifo_count = 1;
    int16_t                     sample[3];

    if(sensor->isInitialized != F_USING_ACCEL)
    {
        return SENSOR_ERROR_INIT;
    }

    // read the F_STATUS register (mapped to STATUS) and extract number of measurements available (lower 6 bits)
    status =  Sensor_I2C_Read(sensor->bus_driver, &sensor->deviceInfo, sensor->addr, MMA845x_F_STATUS_READ, I2C_Buffer );
    if (status==SENSOR_ERROR_NONE) {
        if (sfg->Accel.iWhoAmI==MMA8451_WHO_AM_I_WHOAMI_VALUE) {
            sensor_fifo_count = I2C_Buffer[0] & 0x3F;
        } else {
            sensor_fifo_count = (I2C_Buffer[0] & 0x08)>>3;
        }
        // return if there are no measurements in the sensor FIFO.
        // this will only occur when the FAST_LOOP_HZ equals or exceeds ACCEL_ODR_HZ
        if  (sensor_fifo_count == 0) return SENSOR_ERROR_READ;
    } else {
        return(status);
    }

    MMA845x_DATA_READ[0].readFrom = MMA845x_OUT_X_MSB;
    MMA845x_DATA_READ[0].numBytes = 6 * sensor_fifo_count;
    status =  Sensor_I2C_Read(sensor->bus_driver, &sensor->deviceInfo, sensor->addr, MMA845x_DATA_READ, I2C_Buffer );

    if (status==SENSOR_ERROR_NONE) {
        for (j = 0; j < sensor_fifo_count; j++)
        {
            sample[CHX] = (I2C_Buffer[6 * j    ] << 8) | I2C_Buffer[6 * j + 1];
            sample[CHY] = (I2C_Buffer[6 * j + 2] << 8) | I2C_Buffer[6 * j + 3];
            sample[CHZ] = (I2C_Buffer[6 * j + 4] << 8) | I2C_Buffer[6 * j + 5];
            conditionSample(sample);  // truncate negative values to -32767
            addToFifo((union FifoSensor*) &(sfg->Accel), ACCEL_FIFO_SIZE, sample);
        }
    }

    return (status);
}


// Each entry in a RegisterWriteList is composed of: register address, value to write, bit-mask to apply to write (0 enables)
const registerwritelist_t   MMA845x_IDLE[] =
{
  // Set ACTIVE = other bits unchanged
  { MMA845x_CTRL_REG1, 0x00, 0x01 },
    __END_WRITE_DATA__
};

// MMA845x_Idle places the sensor into Standby mode (wakeup time = 2/ODR+1ms)
int8_t MMA845x_Idle(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg)
{
    int32_t     status;
    if(sensor->isInitialized == F_USING_ACCEL) {
        status = Sensor_I2C_Write(sensor->bus_driver, &sensor->deviceInfo, sensor->addr, MMA845x_IDLE );
        sensor->isInitialized = 0;
        sfg->Accel.isEnabled = false;
    } else {
      return SENSOR_ERROR_INIT;
    }
    return status;
}
#endif  // F_USING_ACCEL

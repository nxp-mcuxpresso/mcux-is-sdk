Sensor Drivers & Utility Functions {#drivers}
=======
Utility Functions
-----------------
* [addToFifo] (@ref addToFifo())

Sensor Drivers
--------------
All sensor driver functions utilize the same function prototype:

    int8_t <sensor>_[Init|Read|Idle](PhysicalSensor *sensor, SensorFusionGlobals *sfg)

FXAS21002 Gyroscope Driver Functions
* [FXAS21002_Init] (@ref FXAS21002_Init())
* [FXAS21002_Read] (@ref FXAS21002_Read())
* [FXAS21002_Idle] (@ref FXAS21002_Idle())

FXLS8471Q and MMA845x Accelerometer Driver Functions
* [FXLS8471Q_Init] (@ref FXLS8471Q_Init())
* [FXLS8471Q_Read] (@ref FXLS8471Q_Read())
* [FXLS8471Q_Idle] (@ref FXLS8471Q_Idle())

FXOS8700 Accel/Mag Combo Sensor Driver Functions
* [FXOS8700_Init] (@ref FXOS8700_Init())
* [FXOS8700_Read] (@ref FXOS8700_Read())
* [FXOS8700_Idle] (@ref FXOS8700_Idle())

MAG3110 Magnetometer Driver Functions
* [MAG3110_Init] (@ref MAG3110_Init())
* [MAG3110_Read] (@ref MAG3110_Read())
* [MAG3110_Idle] (@ref MAG3110_Idle())

MMA8652 Accelerometer Driver Functions
* [MMA8652_Init] (@ref MMA8652_Init())
* [MMA8652_Read] (@ref MMA8652_Read())
* [MMA8652_Idle] (@ref MMA8652_Idle())

MPL3115 Pressure Sensor / Altimeter Driver Functions
* [MPL3115_Init] (@ref MPL3115_Init())
* [MPL3115_Read] (@ref MPL3115_Read())
* [MPL3115_Idle] (@ref MPL3115_Idle())

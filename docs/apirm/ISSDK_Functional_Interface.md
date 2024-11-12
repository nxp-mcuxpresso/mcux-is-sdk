ISSDK Functional Interface (Sensor Driver API)         {#functionalapis}
==============================================
<P>Each Sensor Driver is unique to that driver. There is no abstraction, just a simple set of up to four functions for each sensor: Initialize, Configure, ReadData, and Deinit.
ISSDK support following Functional Interface (Sensor Driver API).

FXLS8964AF Functional Interface
-------------------------------
Function | Description
---------|------------
[FXLS896x_I2C_Initialize](@ref FXLS896x_I2C_Initialize(fxls896x_i2c_sensorhandle_t *pSensorHandle, ARM_DRIVER_I2C *pBus, uint8_t index, uint16_t sAddress, uint8_t *whoAmi)) | The interface function to initialize the FXLS8964AF sensor
[FXLS896x_I2C_Configure](@ref FXLS896x_I2C_Configure(fxls896x_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the FXLS8964AF sensor
[FXLS896x_I2C_ReadData](@ref FXLS896x_I2C_ReadData(fxls896x_i2c_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer)) | The interface function to read the FXLS8964AF sensor data
[FXLS896x_I2C_DeInit](@ref FXLS896x_I2C_DeInit(fxls896x_i2c_sensorhandle_t *pSensorHandle)) | The interface function to stop the FXLS8964AF sensor.
[FXLS896x_SPI_Initialize](@ref FXLS896x_SPI_Initialize(fxls896x_spi_sensorhandle_t *pSensorHandle, ARM_DRIVER_SPI *pBus, uint8_t index, void *pSlaveSelect, uint8_t *whoAmi)) | The interface function to initialize the FXLS8964AF sensor
[FXLS896x_SPI_Configure](@ref FXLS896x_SPI_Configure(fxls896x_spi_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the FXLS8964AF sensor
[FXLS896x_SPI_ReadData](@ref FXLS896x_SPI_ReadData(fxls896x_spi_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer)) | The interface function to read the FXLS8964AF sensor data
[FXLS896x_SPI_Deinit](@ref FXLS896x_SPI_Deinit(fxls896x_spi_sensorhandle_t *pSensorHandle)) | The interface function to stop the FXLS8964AF sensor.

FXLS8967AF Functional Interface
-------------------------------
Function | Description
---------|------------
[FXLS896x_I2C_Initialize](@ref FXLS896x_I2C_Initialize(fxls896x_i2c_sensorhandle_t *pSensorHandle, ARM_DRIVER_I2C *pBus, uint8_t index, uint16_t sAddress, uint8_t *whoAmi)) | The interface function to initialize the FXLS8967AF sensor
[FXLS896x_I2C_Configure](@ref FXLS896x_I2C_Configure(fxls896x_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the FXLS8967AF sensor
[FXLS896x_I2C_ReadData](@ref FXLS896x_I2C_ReadData(fxls896x_i2c_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer)) | The interface function to read the FXLS8967AF sensor data
[FXLS896x_I2C_DeInit](@ref FXLS896x_I2C_DeInit(fxls896x_i2c_sensorhandle_t *pSensorHandle)) | The interface function to stop the FXLS8967AF sensor.
[FXLS896x_SPI_Initialize](@ref FXLS896x_SPI_Initialize(fxls896x_spi_sensorhandle_t *pSensorHandle, ARM_DRIVER_SPI *pBus, uint8_t index, void *pSlaveSelect, uint8_t *whoAmi)) | The interface function to initialize the FXLS8967AF sensor
[FXLS896x_SPI_Configure](@ref FXLS896x_SPI_Configure(fxls896x_spi_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the FXLS8967AF sensor
[FXLS896x_SPI_ReadData](@ref FXLS896x_SPI_ReadData(fxls896x_spi_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer)) | The interface function to read the FXLS8967AF sensor data
[FXLS896x_SPI_Deinit](@ref FXLS896x_SPI_Deinit(fxls896x_spi_sensorhandle_t *pSensorHandle)) | The interface function to stop the FXLS8967AF sensor.

FXLS8974CF Functional Interface
-------------------------------
Function | Description
---------|------------
[FXLS8974_I2C_Initialize](@ref FXLS8974_I2C_Initialize(fxls8974_i2c_sensorhandle_t *pSensorHandle, ARM_DRIVER_I2C *pBus, uint8_t index, uint16_t sAddress, uint8_t *whoAmi)) | The interface function to initialize the FXLS8974CF sensor
[FXLS8974_I2C_Configure](@ref FXLS8974_I2C_Configure(fxls8974_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the FXLS8974CF sensor
[FXLS8974_I2C_ReadData](@ref FXLS8974_I2C_ReadData(fxls8974_i2c_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer)) | The interface function to read the FXLS8974CF sensor data
[FXLS8974_I2C_DeInit](@ref FXLS8974_I2C_DeInit(fxls8974_i2c_sensorhandle_t *pSensorHandle)) | The interface function to stop the FXLS8974CF sensor.
[FXLS8974_SPI_Initialize](@ref FXLS8974_SPI_Initialize(fxls8974_spi_sensorhandle_t *pSensorHandle, ARM_DRIVER_SPI *pBus, uint8_t index, void *pSlaveSelect, uint8_t *whoAmi)) | The interface function to initialize the FXLS8974CF sensor
[FXLS8974_SPI_Configure](@ref FXLS8974_SPI_Configure(fxls8974_spi_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the FXLS8974CF sensor
[FXLS8974_SPI_ReadData](@ref FXLS8974_SPI_ReadData(fxls8974_spi_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer)) | The interface function to read the FXLS8974CF sensor data
[FXLS8974_SPI_Deinit](@ref FXLS8974_SPI_Deinit(fxls8974_spi_sensorhandle_t *pSensorHandle)) | The interface function to stop the FXLS8974CF sensor.


FXAS21002 Functional Interface
-------------------------------
Function | Description
---------|------------
[FXAS21002_I2C_Initialize](@ref FXAS21002_I2C_Initialize(fxas21002_i2c_sensorhandle_t *pSensorHandle, ARM_DRIVER_I2C *pBus, uint8_t index, uint16_t sAddress, uint8_t whoAmi)) | The interface function to initialize the FXAS21002 sensor
[FXAS21002_I2C_Configure](@ref FXAS21002_I2C_Configure(fxas21002_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the FXAS21002 sensor
[FXAS21002_I2C_ReadData](@ref FXAS21002_I2C_ReadData(fxas21002_i2c_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer )) | The interface function to read the FXAS21002 sensor data
[FXAS21002_I2C_Deinit](@ref FXAS21002_I2C_Deinit(fxas21002_i2c_sensorhandle_t *pSensorHandle )) | The interface function to stop the FXAS21002 sensor.

FXLS8962 Functional Interface
-------------------------------
Function | Description
---------|------------
[FXLS8962_I2C_Initialize](@ref FXLS8962_I2C_Initialize(fxls8962_i2c_sensorhandle_t *pSensorHandle, ARM_DRIVER_I2C *pBus, uint8_t index, uint16_t sAddress, uint8_t *whoAmi)) | The interface function to initialize the FXLS8962 sensor
[FXLS8962_I2C_Configure](@ref FXLS8962_I2C_Configure(fxls8962_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the FXLS8962 sensor
[FXLS8962_I2C_ReadData](@ref FXLS8962_I2C_ReadData(fxls8962_i2c_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer )) | The interface function to read the FXLS8962 sensor data
[FXLS8962_I2C_DeInit](@ref FXLS8962_I2C_DeInit(fxls8962_i2c_sensorhandle_t *pSensorHandle )) | The interface function to stop the FXLS8962 sensor.
[FXLS8962_SPI_Initialize](@ref FXLS8962_SPI_Initialize(fxls8962_spi_sensorhandle_t *pSensorHandle, ARM_DRIVER_SPI *pBus, uint8_t index, void *pSlaveSelect, uint8_t *whoAmi)) | The interface function to initialize the FXLS8962 sensor
[FXLS8962_SPI_Configure](@ref FXLS8962_SPI_Configure(fxls8962_spi_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the FXLS8962 sensor
[FXLS8962_SPI_ReadData](@ref FXLS8962_SPI_ReadData(fxls8962_spi_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer )) | The interface function to read the FXLS8962 sensor data
[FXLS8962_SPI_Deinit](@ref FXLS8962_SPI_Deinit(fxls8962_spi_sensorhandle_t *pSensorHandle)) | The interface function to stop the FXLS8962 sensor.

FXOS8700 Functional Interface
-------------------------------
Function | Description
---------|------------
[FXOS8700_I2C_Initialize](@ref FXOS8700_I2C_Initialize(fxos8700_i2c_sensorhandle_t *pSensorHandle, ARM_DRIVER_I2C *pBus, uint8_t index, uint16_t sAddress, uint8_t whoAmi)) | The interface function to initialize the FXOS8700 sensor
[FXOS8700_I2C_Configure](@ref FXOS8700_I2C_Configure(fxos8700_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the FXOS8700 sensor
[FXOS8700_I2C_ReadData](@ref FXOS8700_I2C_ReadData(fxos8700_i2c_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer )) | The interface function to read the FXOS8700 sensor data
[FXOS8700_I2C_Deinit](@ref FXOS8700_I2C_Deinit(fxos8700_i2c_sensorhandle_t *pSensorHandle )) | The interface function to stop the FXOS8700 sensor.

MPL3115 Functional Interface
-------------------------------
Function | Description
---------|------------
[MPL3115_I2C_Initialize](@ref MPL3115_I2C_Initialize(mpl3115_i2c_sensorhandle_t *pSensorHandle, ARM_DRIVER_I2C *pBus, uint8_t index, uint16_t sAddress, uint8_t whoAmi)) | The interface function to initialize the MPL3115 sensor
[MPL3115_I2C_Configure](@ref MPL3115_I2C_Configure(mpl3115_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the MPL3115 sensor
[MPL3115_I2C_ReadData](@ref MPL3115_I2C_ReadData(mpl3115_i2c_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer )) | The interface function to read the MPL3115 sensor data
[MPL3115_I2C_DeInit](@ref MPL3115_I2C_DeInit(mpl3115_i2c_sensorhandle_t *pSensorHandle )) | The interface function to stop the MPL3115 sensor.

MAG3110 Functional Interface
-------------------------------
Function | Description
---------|------------
[MAG3110_I2C_Initialize](@ref MAG3110_I2C_Initialize(mag3110_i2c_sensorhandle_t *pSensorHandle, ARM_DRIVER_I2C *pBus, uint8_t index, uint16_t sAddress, uint8_t whoAmi)) | The interface function to initialize the MAG3110 sensor
[MAG3110_I2C_Configure](@ref MAG3110_I2C_Configure(mag3110_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the MAG3110 sensor
[MAG3110_I2C_ReadData](@ref MAG3110_I2C_ReadData(mag3110_i2c_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer )) | The interface function to read the MAG3110 sensor data
[MAG3110_I2C_DeInit](@ref MAG3110_I2C_DeInit(mag3110_i2c_sensorhandle_t *pSensorHandle )) | The interface function to stop the MAG3110 sensor.

MMA865x Functional Interface
-------------------------------
Function | Description
---------|------------
[MMA865x_I2C_Initialize](@ref MMA865x_I2C_Initialize(mma865x_i2c_sensorhandle_t *pSensorHandle, ARM_DRIVER_I2C *pBus, uint8_t index, uint16_t sAddress, uint8_t whoAmi)) | The interface function to initialize the MMA865X sensor
[MMA865x_I2C_Configure](@ref MMA865x_I2C_Configure(mma865x_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the MMA865X sensor
[MMA865x_I2C_ReadData](@ref MMA865x_I2C_ReadData(mma865x_i2c_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer )) | The interface function to read the MMA865X sensor data
[MMA865x_I2C_DeInit](@ref MMA865x_I2C_DeInit(mma865x_i2c_sensorhandle_t *pSensorHandle )) | The interface function to stop the MMA865X sensor.

MMA8491Q Functional Interface
-------------------------------
Function | Description
---------|------------
[MMA8491Q_I2C_Initialize](@ref MMA8491Q_I2C_Initialize(mma8491q_i2c_sensorhandle_t *pSensorHandle, ARM_DRIVER_I2C* pBus, uint8_t index, uint16_t sAddress)) | The interface function to initialize the MMA8491Q sensor
[MMA8491Q_I2C_ReadData](@ref MMA8491Q_I2C_ReadData(mma8491q_i2c_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer )) | The interface function to read the MMA8491Q sensor data

FXLS8471Q Functional Interface
-------------------------------
Function | Description
---------|------------
[FXLS8471Q_SPI_Initialize](@ref FXLS8471Q_SPI_Initialize(fxls8471q_spi_sensorhandle_t *pSensorHandle, ARM_DRIVER_SPI *pBus, uint8_t index, void *pSlaveSelect, uint8_t whoAmi)) | The interface function to initialize the FXLS8471Q sensor
[FXLS8471Q_SPI_Configure](@ref FXLS8471Q_SPI_Configure(fxls8471q_spi_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the FXLS8471Q sensor
[FXLS8471Q_SPI_ReadData](@ref FXLS8471Q_SPI_ReadData(fxls8471q_spi_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer )) | The interface function to read the FXLS8471Q sensor data
[FXLS8471Q_SPI_Deinit](@ref FXLS8471Q_SPI_Deinit(fxls8471q_spi_sensorhandle_t *pSensorHandle)) | The interface function to stop the FXLS8471Q sensor.

FXLC95000 Functional Interface
-------------------------------
Function | Description
---------|------------
[FXLC95000_SPI_Initialize](@ref FXLC95000_SPI_Initialize(fxlc95000_spi_sensorhandle_t *pSensorHandle, ARM_DRIVER_SPI *pBus, uint8_t index, void *pSpiSelect, void *pSlaveSelect, void *pReset, uint16_t buildId)) | The interface function to initialize the FXLC95000 sensor in SPI mode.
[FXLC95000_SPI_CommandResponse](@ref FXLC95000_SPI_CommandResponse(fxlc95000_spi_sensorhandle_t *pSensorHandle, const registercommandlist_t *pCommandList, const registerreadlist_t *pResponseList, uint8_t *pBuffer)) | The interface function to read the FXLC95000 sensor data.
[FXLC95000_I2C_Initialize](@ref FXLC95000_I2C_Initialize(fxlc95000_i2c_sensorhandle_t *pSensorHandle, ARM_DRIVER_I2C *pBus, uint8_t index, uint16_t sAddress, uint16_t buildId)) | The I2C interface function to initialize the FXLC95000 sensor in I2C mode.
[FXLC95000_I2C_CommandResponse](@ref FXLC95000_I2C_CommandResponse(fxlc95000_i2c_sensorhandle_t *pSensorHandle, const registercommandlist_t *pCommandList, const registerreadlist_t *pResponseList, uint8_t *pBuffer)) | The interface function to read the FXLC95000 sensor data.

MMA9553 Functional Interface
-------------------------------
Function | Description
---------|------------
[MMA9553_I2C_Initialize](@ref MMA9553_I2C_Initialize(mma9553_i2c_sensorhandle_t *pSensorHandle, ARM_DRIVER_I2C* pBus, uint8_t index, uint16_t sAddress)) | The interface function to initialize the MMA9553 sensor
[MMA9553_I2C_Configure](@ref MMA9553_I2C_Configure(mma9553_i2c_sensorhandle_t *pSensorHandle, const registercommandlist_t *pCommandList)) | The interface function to configure the MMA9553 sensor
[MMA9553_I2C_CommandResponse](@ref MMA9553_I2C_CommandResponse(mma9553_i2c_sensorhandle_t *pSensorHandle, const registercommandlist_t *pCommandList, const registerreadlist_t *pReadList, uint8_t *pBuffer)) | The interface function to read the MMA9553 sensor data
[MMA9553_I2C_DeInit](@ref MMA9553_I2C_DeInit(mma9553_i2c_sensorhandle_t *pSensorHandle)) | The interface function to stop the MMA9553 sensor.

MMA845x Functional Interface
-------------------------------
Function | Description
---------|------------
[MMA845x_I2C_Initialize](@ref MMA845x_I2C_Initialize(mma845x_i2c_sensorhandle_t *pSensorHandle, ARM_DRIVER_I2C *pBus, uint8_t index, uint16_t sAddress, uint8_t whoAmi)) | The interface function to initialize the MMA845x sensor
[MMA845x_I2C_Configure](@ref MMA845x_I2C_Configure(mma845x_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the MMA845x sensor
[MMA845x_I2C_ReadData](@ref MMA845x_I2C_ReadData(mma845x_i2c_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer )) | The interface function to read the MMA845x sensor data
[MMA845x_I2C_Deinit](@ref MMA845x_I2C_Deinit(mma845x_i2c_sensorhandle_t *pSensorHandle )) | The interface function to stop the MMA845x sensor.

FXPQ3115 Functional Interface
-------------------------------
Function | Description
---------|------------
[FXPQ3115_I2C_Initialize](@ref FXPQ3115_I2C_Initialize(fxpq3115_i2c_sensorhandle_t *pSensorHandle, ARM_DRIVER_I2C *pBus, uint8_t index, uint16_t sAddress, uint8_t whoAmi)) | The interface function to initialize the FXPQ3115 sensor
[FXPQ3115_I2C_Configure](@ref FXPQ3115_I2C_Configure(fxpq3115_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the FXPQ3115 sensor
[FXPQ3115_I2C_ReadData](@ref FXPQ3115_I2C_ReadData(fxpq3115_i2c_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer )) | The interface function to read the FXPQ3115 sensor data
[FXPQ3115_I2C_DeInit](@ref FXPQ3115_I2C_DeInit(fxpq3115_i2c_sensorhandle_t *pSensorHandle )) | The interface function to stop the FXPQ3115 sensor.

FXPS7250D4 Functional Interface
-------------------------------
Function | Description
---------|------------
[DBAP_I2C_Initialize](@ref DBAP_I2C_Initialize(dbap_i2c_sensorhandle_t *pSensorHandle, ARM_DRIVER_I2C *pBus, uint8_t index, uint16_t sAddress, uint8_t whoAmi)) | The interface function to initialize the FXPS7250D4 sensor
[DBAP_I2C_Configure](@ref DBAP_I2C_Configure(dbap_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)) | The interface function to configure the FXPS7250D4 sensor
[DBAP_I2C_ReadData](@ref DBAP_I2C_ReadData(dbap_i2c_sensorhandle_t *pSensorHandle, const registerreadlist_t *pReadList, uint8_t *pBuffer )) | The interface function to read the FXPS7250D4 sensor data

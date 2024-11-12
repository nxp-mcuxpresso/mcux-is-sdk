ISSDK Low Level Interface         {#lowlevelapis}
=========================
<P>ISSDK support direct low level interface to the sensor and register I2C or SPI interfaces.
 
Register I/O Interface
-----------------------
Function | Description 
---------|------------
[Register_I2C_Write](@ref Register_I2C_Write(ARM_DRIVER_I2C *pCommDrv, registerDeviceInfo_t *devInfo, uint16_t slaveAddress, uint8_t offset, uint8_t value, uint8_t mask, bool repeatedStart)) | The low lever interface function to write to a sensor register over I2C
[Register_I2C_Read](@ref Register_I2C_Read(ARM_DRIVER_I2C *pCommDrv, registerDeviceInfo_t *devInfo, uint16_t slaveAddress, uint8_t offset, uint8_t length, uint8_t *pOutBuffer)) | The low lever interface function to read a sensor register over I2C
[Register_SPI_Write](@ref Register_SPI_Write(ARM_DRIVER_SPI *pCommDrv, registerDeviceInfo_t *devInfo, void *pWriteParams, uint8_t offset, uint8_t value, uint8_t mask)) | The low lever interface function to write to a sensor register over SPI
[Register_SPI_Read](@ref Register_SPI_Read(ARM_DRIVER_SPI *pCommDrv, registerDeviceInfo_t *devInfo, void *pReadParams, uint8_t offset, uint8_t length, uint8_t *pOutBuffer)) | The low lever interface function to write to a sensor register over SPI

Sensor I/O Interface
-----------------------
Function | Description 
---------|------------
[Sensor_I2C_Write](@ref Sensor_I2C_Write(ARM_DRIVER_I2C *pCommDrv, registerDeviceInfo_t *devInfo, uint16_t slaveAddress, const registerwritelist_t* pRegWriteList )) | The low lever interface function to write register data to a sensor over I2C
[Sensor_I2C_Read](@ref Sensor_I2C_Read(ARM_DRIVER_I2C *pCommDrv, registerDeviceInfo_t *devInfo, uint16_t slaveAddress, const registerreadlist_t *pReadList, uint8_t *pOutBuffer)) | The low lever interface function to read register data from a sensor over I2C
[Sensor_SPI_Write](@ref Sensor_SPI_Write(ARM_DRIVER_SPI *pCommDrv, registerDeviceInfo_t *devInfo, void *pWriteParams, const registerwritelist_t* pRegWriteList)) | The low lever interface function to write register data to a sensor register over SPI
[Sensor_SPI_Read](@ref Sensor_SPI_Read(ARM_DRIVER_SPI *pCommDrv, registerDeviceInfo_t *devInfo, void *pReadParams, const registerreadlist_t *pReadList, uint8_t *pOutBuffer)) | The low lever interface function to read register data from a sensor over SPI

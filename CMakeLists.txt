
if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.interface.common)
    mcux_add_source(
        SOURCES sensors/register_io_i2c.c
                sensors/register_io_spi.c
                sensors/sensor_io_spi.c
                sensors/sensor_io_i2c.c
                sensors/register_io_i2c.h
                sensors/register_io_spi.h
                sensors/sensor_io_spi.h
                sensors/sensor_io_i2c.h
                sensors/sensor_drv.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()


if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.gpio_kinetis)
    mcux_add_source(
        SOURCES drivers/gpio/kinetis/gpio_driver_irq.c
                drivers/gpio/kinetis/gpio_driver.c
                drivers/gpio/kinetis/gpio_driver.h
                drivers/gpio/Driver_GPIO.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/gpio/kinetis
                 drivers/gpio
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.gpio_kinetis_kl27z)
    mcux_add_source(
        SOURCES drivers/gpio/kinetis/gpio_driver_irq_kl27z.c
                drivers/gpio/kinetis/gpio_driver.c
                drivers/gpio/kinetis/gpio_driver.h
                drivers/gpio/Driver_GPIO.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/gpio/kinetis
                 drivers/gpio
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.gpio_kinetis_ke15z)
    mcux_add_source(
        SOURCES drivers/gpio/kinetis/gpio_driver_irq_ke15z.c
                drivers/gpio/kinetis/gpio_driver.c
                drivers/gpio/kinetis/gpio_driver.h
                drivers/gpio/Driver_GPIO.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/gpio/kinetis
                 drivers/gpio
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.gpio_kinetis_kw45b41z)
    mcux_add_source(
        SOURCES drivers/gpio/kinetis/kw45b41z-evk/gpio_driver_irq.c
                drivers/gpio/kinetis/kw45b41z-evk/gpio_driver.c
                drivers/gpio/kinetis/kw45b41z-evk/gpio_driver.h
                drivers/gpio/Driver_GPIO.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/gpio/kinetis/kw45b41z-evk
                 drivers/gpio
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.gpio_kinetis_k32w148)
    mcux_add_source(
        SOURCES drivers/gpio/kinetis/k32w148-evk/gpio_driver_irq.c
                drivers/gpio/kinetis/k32w148-evk/gpio_driver.c
                drivers/gpio/kinetis/k32w148-evk/gpio_driver.h
                drivers/gpio/Driver_GPIO.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/gpio/kinetis/k32w148-evk
                 drivers/gpio
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.gpio_kinetis_kw41z)
    mcux_add_source(
        SOURCES drivers/gpio/kinetis/gpio_driver_irq_kw41z.c
                drivers/gpio/kinetis/gpio_driver.c
                drivers/gpio/kinetis/gpio_driver.h
                drivers/gpio/Driver_GPIO.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/gpio/kinetis
                 drivers/gpio
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.gpio_lpc)
    mcux_add_source(
        SOURCES drivers/gpio/lpc/gpio_driver_irq.c
                drivers/gpio/lpc/gpio_driver.c
                drivers/gpio/lpc/gpio_driver.h
                drivers/gpio/Driver_GPIO.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/gpio/lpc
                 drivers/gpio
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.gpio_imx)
    mcux_add_source(
        SOURCES drivers/gpio/i.mx/gpio_driver_irq.c
                drivers/gpio/i.mx/gpio_driver.c
                drivers/gpio/i.mx/gpio_driver.h
                drivers/gpio/Driver_GPIO.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/gpio/i.mx
                 drivers/gpio
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.gpio_imx8)
    mcux_add_source(
        SOURCES drivers/gpio/i.mx/gpio_driver.c
                drivers/gpio/i.mx/gpio_driver.h
                drivers/gpio/Driver_GPIO.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/gpio/i.mx
                 drivers/gpio
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.gpio_imxrt600)
    mcux_add_source(
        SOURCES drivers/gpio/i.mx/i.mxrt600/gpio_driver.c
                drivers/gpio/i.mx/i.mxrt600/gpio_driver.h
                drivers/gpio/Driver_GPIO.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/gpio/i.mx/i.mxrt600
                 drivers/gpio
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.gpio_imxrt700)
    mcux_add_source(
        SOURCES drivers/gpio/i.mx/i.mxrt700/gpio_driver.c
                drivers/gpio/i.mx/i.mxrt700/gpio_driver.h
                drivers/gpio/Driver_GPIO.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/gpio/i.mx/i.mxrt700
                 drivers/gpio
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.gpio_imxrt1180)
    mcux_add_source(
        SOURCES drivers/gpio/i.mx/i.mxrt1180/gpio_driver.c
                drivers/gpio/i.mx/i.mxrt1180/gpio_driver.h
                drivers/gpio/Driver_GPIO.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/gpio/i.mx/i.mxrt1180
                 drivers/gpio
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.gpio_imxrt)
    mcux_add_source(
        SOURCES drivers/gpio/i.mx/gpio_driver.c
                drivers/gpio/i.mx/gpio_driver.h
                drivers/gpio/Driver_GPIO.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/gpio/i.mx
                 drivers/gpio
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.gpio_mcx)
    mcux_add_source(
        SOURCES drivers/gpio/mcx/gpio_driver_irq.c
                drivers/gpio/mcx/gpio_driver.c
                drivers/gpio/mcx/gpio_driver.h
                drivers/gpio/Driver_GPIO.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/gpio/mcx
                 drivers/gpio
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.gpio_mcxw71)
    mcux_add_source(
        SOURCES drivers/gpio/mcx/frdm_mcxw71/gpio_driver_irq.c
                drivers/gpio/mcx/frdm_mcxw71/gpio_driver.c
                drivers/gpio/mcx/frdm_mcxw71/gpio_driver.h
                drivers/gpio/Driver_GPIO.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/gpio/mcx/frdm_mcxw71
                 drivers/gpio
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.drivers.systick_utils)
    mcux_add_source(
        SOURCES drivers/systick/systick_utils.c
                drivers/systick/systick_utils.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES drivers/systick
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.allregdefs)
    mcux_add_source(
        SOURCES sensors/fxas21002.h
                sensors/fxos8700.h
                sensors/mpl3115.h
                sensors/fxlc95000.h
                sensors/fxls8471q.h
                sensors/fxls8962.h
                sensors/fxpq3115.h
                sensors/mag3110.h
                sensors/mma845x.h
                sensors/mma865x.h
                sensors/mma8491q.h
                sensors/mma9553.h
                sensors/diff_p.h
                sensors/dbap.h
                sensors/fxls896x.h
                sensors/fxls8974.h
                sensors/fxls8961.h
                sensors/fxls8971.h
                sensors/nmh1000.h
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.fxas21002)
    mcux_add_source(
        SOURCES sensors/fxas21002.h
                sensors/fxas21002_drv.h
                sensors/fxas21002_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.fxos8700)
    mcux_add_source(
        SOURCES sensors/fxos8700.h
                sensors/fxos8700_drv.h
                sensors/fxos8700_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.mpl3115)
    mcux_add_source(
        SOURCES sensors/mpl3115.h
                sensors/mpl3115_drv.h
                sensors/mpl3115_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.fxlc95000)
    mcux_add_source(
        SOURCES sensors/fxlc95000.h
                sensors/fxlc95000_drv.h
                sensors/fxlc95000_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.mma8491q)
    mcux_add_source(
        SOURCES sensors/mma8491q.h
                sensors/mma8491q_drv.h
                sensors/mma8491q_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.fxls8471q)
    mcux_add_source(
        SOURCES sensors/fxls8471q.h
                sensors/fxls8471q_drv.h
                sensors/fxls8471q_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.mag3110)
    mcux_add_source(
        SOURCES sensors/mag3110.h
                sensors/mag3110_drv.h
                sensors/mag3110_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.mma845x)
    mcux_add_source(
        SOURCES sensors/mma845x.h
                sensors/mma845x_drv.h
                sensors/mma845x_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.mma865x)
    mcux_add_source(
        SOURCES sensors/mma865x.h
                sensors/mma865x_drv.h
                sensors/mma865x_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.fxpq3115)
    mcux_add_source(
        SOURCES sensors/fxpq3115.h
                sensors/fxpq3115_drv.h
                sensors/fxpq3115_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.fxls8962)
    mcux_add_source(
        SOURCES sensors/fxls8962.h
                sensors/fxls8962_drv.h
                sensors/fxls8962_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.mma9553)
    mcux_add_source(
        SOURCES sensors/mma9553.h
                sensors/mma9553_drv.h
                sensors/mma9553_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.nps300x)
    mcux_add_source(
        SOURCES sensors/diff_p.h
                sensors/diff_p_drv.h
                sensors/diff_p_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.isl29023)
    mcux_add_source(
        SOURCES sensors/isl29023.h
                sensors/isl29023_drv.h
                sensors/isl29023_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.fxps7250d4)
    mcux_add_source(
        SOURCES sensors/dbap.h
                sensors/dbap_drv.h
                sensors/dbap_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.fxls896xaf)
    mcux_add_source(
        SOURCES sensors/fxls896x.h
                sensors/fxls896x_drv.h
                sensors/fxls896x_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.fxls8974cf)
    mcux_add_source(
        SOURCES sensors/fxls8974.h
                sensors/fxls8974_drv.h
                sensors/fxls8974_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.fxls8961af)
    mcux_add_source(
        SOURCES sensors/fxls8961.h
                sensors/fxls8961_drv.h
                sensors/fxls8961_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.fxls8971cf)
    mcux_add_source(
        SOURCES sensors/fxls8971.h
                sensors/fxls8971_drv.h
                sensors/fxls8971_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

if(CONFIG_MCUX_COMPONENT_middleware.issdk.sensor.nmh1000)
    mcux_add_source(
        SOURCES sensors/nmh1000.h
                sensors/nmh1000_drv.h
                sensors/nmh1000_drv.c
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
    mcux_add_include(
        INCLUDES sensors
        BASE_PATH ${SdkRootDirPath}/middleware/issdk/
    )
endif()

/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v3.0
processor: QN908XC
package_id: QN9080C
mcu_data: ksdk2_0
processor_version: 0.0.0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_iocon.h"
#include "pin_mux.h"

#define PIN16_IDX                       16u   /*!< Pin number for pin 16 in a port */
#define PIN17_IDX                       17u   /*!< Pin number for pin 17 in a port */
#define PIN22_IDX                       22u   /*!< Pin number for pin 22 in a port */
#define PORTA_IDX                        0u   /*!< Port index */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm4}
- pin_list:
  - {pin_num: '29', peripheral: FLEXCOMM0, signal: TXD, pin_signal: GPIOA16/CS2/SCT0_OUT1/CTIMER2_MAT0/FC0_TXD/FC3_MOSI/QDEC0_A, pull_control: High_Z, drive_strength: low}
  - {pin_num: '28', peripheral: FLEXCOMM0, signal: RXD, pin_signal: GPIOA17/CS3/SD_DAC/CTIMER2_MAT1/FC0_RXD/FC3_MISO/QDEC0_B, pull_control: High_Z, drive_strength: low}
  - {pin_num: '23', peripheral: SWD, signal: SWCLK, pin_signal: SWCLK/GPIOA22/SCT0_IN2/CTIMER3_MAT0/FC2_SDA_SSEL0/FC3_SSEL3/QDEC1_A, pull_control: Pull_down, drive_strength: low}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitPins(void) {
  const uint32_t portA_pin16_config = (
    IOCON_FUNC4 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN16_IDX, portA_pin16_config); /* PORTA PIN16 (coords: 29) is configured as FC0_TXD */
  const uint32_t portA_pin17_config = (
    IOCON_FUNC4 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN17_IDX, portA_pin17_config); /* PORTA PIN17 (coords: 28) is configured as FC0_RXD */
  const uint32_t portA_pin22_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 0 */
    IOCON_MODE_PULLDOWN |                                    /* Selects pull-down function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN22_IDX, portA_pin22_config); /* PORTA PIN22 (coords: 23) is configured as SWCLK */
}

#define PIN16_IDX                       16u   /*!< Pin number for pin 16 in a port */
#define PIN17_IDX                       17u   /*!< Pin number for pin 17 in a port */
#define PORTA_IDX                        0u   /*!< Port index */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
USART0_InitPins:
- options: {coreID: cm4}
- pin_list:
  - {pin_num: '29', peripheral: FLEXCOMM0, signal: TXD, pin_signal: GPIOA16/CS2/SCT0_OUT1/CTIMER2_MAT0/FC0_TXD/FC3_MOSI/QDEC0_A, pull_control: High_Z, drive_strength: low}
  - {pin_num: '28', peripheral: FLEXCOMM0, signal: RXD, pin_signal: GPIOA17/CS3/SD_DAC/CTIMER2_MAT1/FC0_RXD/FC3_MISO/QDEC0_B, pull_control: High_Z, drive_strength: low}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : USART0_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void USART0_InitPins(void) {
  const uint32_t portA_pin16_config = (
    IOCON_FUNC4 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN16_IDX, portA_pin16_config); /* PORTA PIN16 (coords: 29) is configured as FC0_TXD */
  const uint32_t portA_pin17_config = (
    IOCON_FUNC4 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN17_IDX, portA_pin17_config); /* PORTA PIN17 (coords: 28) is configured as FC0_RXD */
}


#define PIN16_IDX                       16u   /*!< Pin number for pin 16 in a port */
#define PIN17_IDX                       17u   /*!< Pin number for pin 17 in a port */
#define PORTA_IDX                        0u   /*!< Port index */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
USART0_DeinitPins:
- options: {coreID: cm4}
- pin_list:
  - {pin_num: '29', peripheral: GPIOA, signal: 'GPIOA, 16', pin_signal: GPIOA16/CS2/SCT0_OUT1/CTIMER2_MAT0/FC0_TXD/FC3_MOSI/QDEC0_A, pull_control: High_Z, drive_strength: low}
  - {pin_num: '28', peripheral: GPIOA, signal: 'GPIOA, 17', pin_signal: GPIOA17/CS3/SD_DAC/CTIMER2_MAT1/FC0_RXD/FC3_MISO/QDEC0_B, pull_control: High_Z, drive_strength: low}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : USART0_DeinitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void USART0_DeinitPins(void) {
  const uint32_t portA_pin16_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 0 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN16_IDX, portA_pin16_config); /* PORTA PIN16 (coords: 29) is configured as GPIOA16 */
  const uint32_t portA_pin17_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 0 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN17_IDX, portA_pin17_config); /* PORTA PIN17 (coords: 28) is configured as GPIOA17 */
}

#define PIN4_IDX                         4u   /*!< Pin number for pin 4 in a port */
#define PIN5_IDX                         5u   /*!< Pin number for pin 5 in a port */
#define PIN30_IDX                       30u   /*!< Pin number for pin 30 in a port */
#define PORTA_IDX                        0u   /*!< Port index */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
SPI0_InitPins:
- options: {coreID: cm4}
- pin_list:
  - {pin_num: '15', peripheral: FLEXCOMM2, signal: SCK, pin_signal: GPIOA30/ACMP1P/ETM_TRACEDAT3/CTIMER3_MAT1/FC2_SCK/FC3_MOSI/SPIFI_IO3, pull_control: High_Z, drive_strength: low}
  - {pin_num: '6', peripheral: FLEXCOMM2, signal: MISO, pin_signal: GPIOA5/ADC3/SCT0_OUT5/CTIMER0_MAT1/FC0_RXD/FC2_SCL_MISO/SPIFI_IO1, pull_control: High_Z, drive_strength: low}
  - {pin_num: '7', peripheral: FLEXCOMM2, signal: MOSI, pin_signal: GPIOA4/ADC2/SCT0_OUT4/CTIMER0_MAT0/FC0_TXD/FC2_SDA_MOSI/SPIFI_IO0, pull_control: High_Z, drive_strength: low}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI0_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void SPI0_InitPins(void) {
  const uint32_t portA_pin4_config = (
    IOCON_FUNC5 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN4_IDX, portA_pin4_config); /* PORTA PIN4 (coords: 7) is configured as FC2_SDA_MOSI */
  const uint32_t portA_pin5_config = (
    IOCON_FUNC5 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN5_IDX, portA_pin5_config); /* PORTA PIN5 (coords: 6) is configured as FC2_SCL_MISO */
  const uint32_t portA_pin30_config = (
    IOCON_FUNC4 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN30_IDX, portA_pin30_config); /* PORTA PIN30 (coords: 15) is configured as FC2_SCK */
}


#define PIN4_IDX                         4u   /*!< Pin number for pin 4 in a port */
#define PIN5_IDX                         5u   /*!< Pin number for pin 5 in a port */
#define PIN30_IDX                       30u   /*!< Pin number for pin 30 in a port */
#define PORTA_IDX                        0u   /*!< Port index */


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
SPI0_DeinitPins:
- options: {coreID: cm4}
- pin_list:
  - {pin_num: '15', peripheral: FLEXCOMM2, signal: SCK, pin_signal: GPIOA30/ACMP1P/ETM_TRACEDAT3/CTIMER3_MAT1/FC2_SCK/FC3_MOSI/SPIFI_IO3, pull_control: High_Z, drive_strength: low}
  - {pin_num: '6', peripheral: FLEXCOMM2, signal: MISO, pin_signal: GPIOA5/ADC3/SCT0_OUT5/CTIMER0_MAT1/FC0_RXD/FC2_SCL_MISO/SPIFI_IO1, pull_control: High_Z, drive_strength: low}
  - {pin_num: '7', peripheral: FLEXCOMM2, signal: MOSI, pin_signal: GPIOA4/ADC2/SCT0_OUT4/CTIMER0_MAT0/FC0_TXD/FC2_SDA_MOSI/SPIFI_IO0, pull_control: High_Z, drive_strength: low}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI0_DeinitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void SPI0_DeinitPins(void) {
  const uint32_t portA_pin4_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN4_IDX, portA_pin4_config); /* PORTA PIN4 (coords: 7) is configured as GPIOA4 */
  const uint32_t portA_pin5_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN5_IDX, portA_pin5_config); /* PORTA PIN5 (coords: 6) is configured as GPIOA5 */
  const uint32_t portA_pin30_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN30_IDX, portA_pin30_config); /* PORTA PIN30 (coords: 15) is configured as GPIOA30 */
}


#define PIN13_IDX                       13u   /*!< Pin number for pin 13 in a port */
#define PIN15_IDX                       15u   /*!< Pin number for pin 15 in a port */
#define PIN16_IDX                       16u   /*!< Pin number for pin 16 in a port */
#define PIN17_IDX                       17u   /*!< Pin number for pin 17 in a port */
#define PORTA_IDX                        0u   /*!< Port index */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
SPI1_InitPins:
- options: {coreID: cm4}
- pin_list:
  - {pin_num: '30', peripheral: FLEXCOMM3, signal: SCK, pin_signal: GPIOA15/CS1/SCT0_OUT0/CTIMER2_CAP1/FC0_CTS/FC3_SCK/QDEC1_B, pull_control: High_Z, drive_strength: low}
  - {pin_num: '29', peripheral: FLEXCOMM3, signal: MOSI, pin_signal: GPIOA16/CS2/SCT0_OUT1/CTIMER2_MAT0/FC0_TXD/FC3_MOSI/QDEC0_A, pull_control: High_Z, drive_strength: low}
  - {pin_num: '28', peripheral: FLEXCOMM3, signal: MISO, pin_signal: GPIOA17/CS3/SD_DAC/CTIMER2_MAT1/FC0_RXD/FC3_MISO/QDEC0_B, pull_control: High_Z, drive_strength: low}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI1_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void SPI1_InitPins(void) {
  const uint32_t portA_pin15_config = (
    IOCON_FUNC5 |                                            /* Selects pin function 5 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN15_IDX, portA_pin15_config); /* PORTA PIN15 (coords: 30) is configured as FC3_SCK */
  const uint32_t portA_pin16_config = (
    IOCON_FUNC5 |                                            /* Selects pin function 5 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN16_IDX, portA_pin16_config); /* PORTA PIN16 (coords: 29) is configured as FC3_MOSI */
  const uint32_t portA_pin17_config = (
    IOCON_FUNC5 |                                            /* Selects pin function 5 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN17_IDX, portA_pin17_config); /* PORTA PIN17 (coords: 28) is configured as FC3_MISO */
}


#define PIN15_IDX                       15u   /*!< Pin number for pin 15 in a port */
#define PIN16_IDX                       16u   /*!< Pin number for pin 16 in a port */
#define PIN17_IDX                       17u   /*!< Pin number for pin 17 in a port */
#define PORTA_IDX                        0u   /*!< Port index */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
SPI1_DeinitPins:
- options: {coreID: cm4}
- pin_list:
  - {pin_num: '30', peripheral: GPIOA, signal: 'GPIOA, 15', pin_signal: GPIOA15/CS1/SCT0_OUT0/CTIMER2_CAP1/FC0_CTS/FC3_SCK/QDEC1_B, pull_control: High_Z, drive_strength: low}
  - {pin_num: '29', peripheral: GPIOA, signal: 'GPIOA, 16', pin_signal: GPIOA16/CS2/SCT0_OUT1/CTIMER2_MAT0/FC0_TXD/FC3_MOSI/QDEC0_A, pull_control: High_Z, drive_strength: low}
  - {pin_num: '28', peripheral: GPIOA, signal: 'GPIOA, 17', pin_signal: GPIOA17/CS3/SD_DAC/CTIMER2_MAT1/FC0_RXD/FC3_MISO/QDEC0_B, pull_control: High_Z, drive_strength: low}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : SPI1_DeinitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void SPI1_DeinitPins(void) {
  const uint32_t portA_pin15_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 0 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN15_IDX, portA_pin15_config); /* PORTA PIN15 (coords: 30) is configured as GPIOA15 */
  const uint32_t portA_pin16_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 0 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN16_IDX, portA_pin16_config); /* PORTA PIN16 (coords: 29) is configured as GPIOA16 */
  const uint32_t portA_pin17_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 0 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN17_IDX, portA_pin17_config); /* PORTA PIN17 (coords: 28) is configured as GPIOA17 */
}


#define PIN6_IDX                         6u   /*!< Pin number for pin 6 in a port */
#define PIN7_IDX                         7u   /*!< Pin number for pin 7 in a port */
#define PIN12_IDX                        12u  /*!< Pin number for pin 12 in a port */
#define PIN13_IDX                        13u  /*!< Pin number for pin 13 in a port */
#define PORTA_IDX                        0u   /*!< Port index */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
I2C0_InitPins:
- options: {coreID: cm4}
- pin_list:
  - {pin_num: '5', peripheral: FLEXCOMM1, signal: RTS_SCL, pin_signal: GPIOA6/SCT0_OUT3/CTIMER0_MAT2/FC1_RTS_SCL/BLE_PTI0/SPIFI_CLK, pull_control: High_Z, drive_strength: low,
    drive_extra: disabled}
  - {pin_num: '4', peripheral: FLEXCOMM1, signal: CTS_SDA, pin_signal: GPIOA7/ADC_VREFI/SCT0_OUT2/CTIMER1_CAP0/FC1_CTS_SDA/BLE_PTI1/SPIFI_CSN, pull_control: High_Z,
    drive_strength: low}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C0_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void I2C0_InitPins(void) {
#ifndef GO_X_BOARD
  const uint32_t portA_pin6_config = (
    IOCON_FUNC4 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN6_IDX, portA_pin6_config); /* PORTA PIN6 (coords: 5) is configured as FC1_RTS_SCL */
  const uint32_t portA_pin7_config = (
    IOCON_FUNC4 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN7_IDX, portA_pin7_config); /* PORTA PIN7 (coords: 4) is configured as FC1_CTS_SDA */
#else
  const uint32_t portA_pin12_config = (
    IOCON_FUNC4 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN12_IDX, portA_pin12_config); /* PORTA PIN12 (coords: 5) is configured as FC1_RTS_SCL */
  const uint32_t portA_pin13_config = (
    IOCON_FUNC4 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN13_IDX, portA_pin13_config); /* PORTA PIN13 (coords: 4) is configured as FC1_CTS_SDA */
#endif
}


#define PIN6_IDX                         6u   /*!< Pin number for pin 6 in a port */
#define PIN7_IDX                         7u   /*!< Pin number for pin 7 in a port */
#define PIN12_IDX                        12u  /*!< Pin number for pin 12 in a port */
#define PIN13_IDX                        13u  /*!< Pin number for pin 13 in a port */
#define PORTA_IDX                        0u   /*!< Port index */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
I2C0_DeinitPins:
- options: {coreID: cm4}
- pin_list:
  - {pin_num: '4', peripheral: GPIOA, signal: 'GPIOA, 7', pin_signal: GPIOA7/ADC_VREFI/SCT0_OUT2/CTIMER1_CAP0/FC1_CTS_SDA/BLE_PTI1/SPIFI_CSN, pull_control: High_Z,
    drive_strength: low}
  - {pin_num: '5', peripheral: GPIOA, signal: 'GPIOA, 6', pin_signal: GPIOA6/SCT0_OUT3/CTIMER0_MAT2/FC1_RTS_SCL/BLE_PTI0/SPIFI_CLK, pull_control: High_Z, drive_strength: low,
    drive_extra: disabled}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C0_DeinitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void I2C0_DeinitPins(void) {
#ifndef GO_X_BOARD    
  const uint32_t portA_pin6_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 0 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN6_IDX, portA_pin6_config); /* PORTA PIN6 (coords: 5) is configured as GPIOA6 */
  const uint32_t portA_pin7_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 0 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN7_IDX, portA_pin7_config); /* PORTA PIN7 (coords: 4) is configured as GPIOA7 */
#else
  const uint32_t portA_pin12_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN12_IDX, portA_pin12_config); /* PORTA PIN12 (coords: 5) is configured as FC1_RTS_SCL */
  const uint32_t portA_pin13_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 4 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN13_IDX, portA_pin13_config); /* PORTA PIN13 (coords: 4) is configured as FC1_CTS_SDA */
#endif  
}


#define PIN4_IDX                         4u   /*!< Pin number for pin 4 in a port */
#define PIN5_IDX                         5u   /*!< Pin number for pin 5 in a port */
#define PORTA_IDX                        0u   /*!< Port index */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
I2C1_InitPins:
- options: {coreID: cm4}
- pin_list:
  - {pin_num: '6', peripheral: FLEXCOMM2, signal: SCL_MISO, pin_signal: GPIOA5/ADC3/SCT0_OUT5/CTIMER0_MAT1/FC0_RXD/FC2_SCL_MISO/SPIFI_IO1, pull_control: High_Z, drive_strength: low}
  - {pin_num: '7', peripheral: FLEXCOMM2, signal: SDA_MOSI, pin_signal: GPIOA4/ADC2/SCT0_OUT4/CTIMER0_MAT0/FC0_TXD/FC2_SDA_MOSI/SPIFI_IO0, pull_control: High_Z, drive_strength: low}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C1_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void I2C1_InitPins(void) {
  const uint32_t portA_pin4_config = (
    IOCON_FUNC5 |                                            /* Selects pin function 5 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN4_IDX, portA_pin4_config); /* PORTA PIN4 (coords: 7) is configured as FC2_SDA_MOSI */
  const uint32_t portA_pin5_config = (
    IOCON_FUNC5 |                                            /* Selects pin function 5 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN5_IDX, portA_pin5_config); /* PORTA PIN5 (coords: 6) is configured as FC2_SCL_MISO */
}


#define PIN4_IDX                         4u   /*!< Pin number for pin 4 in a port */
#define PIN5_IDX                         5u   /*!< Pin number for pin 5 in a port */
#define PORTA_IDX                        0u   /*!< Port index */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
I2C1_DeinitPins:
- options: {coreID: cm4}
- pin_list:
  - {pin_num: '6', peripheral: GPIOA, signal: 'GPIOA, 5', pin_signal: GPIOA5/ADC3/SCT0_OUT5/CTIMER0_MAT1/FC0_RXD/FC2_SCL_MISO/SPIFI_IO1, pull_control: High_Z, drive_strength: low}
  - {pin_num: '7', peripheral: GPIOA, signal: 'GPIOA, 4', pin_signal: GPIOA4/ADC2/SCT0_OUT4/CTIMER0_MAT0/FC0_TXD/FC2_SDA_MOSI/SPIFI_IO0, pull_control: High_Z, drive_strength: low}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : I2C1_DeinitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void I2C1_DeinitPins(void) {
  const uint32_t portA_pin4_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 0 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN4_IDX, portA_pin4_config); /* PORTA PIN4 (coords: 7) is configured as GPIOA4 */
  const uint32_t portA_pin5_config = (
    IOCON_FUNC0 |                                            /* Selects pin function 0 */
    IOCON_MODE_HIGHZ |                                       /* Selects High-Z function */
    IOCON_DRIVE_LOW                                          /* Enable low drive strength */
  );
  IOCON_PinMuxSet(IOCON, PORTA_IDX, PIN5_IDX, portA_pin5_config); /* PORTA PIN5 (coords: 6) is configured as GPIOA5 */
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

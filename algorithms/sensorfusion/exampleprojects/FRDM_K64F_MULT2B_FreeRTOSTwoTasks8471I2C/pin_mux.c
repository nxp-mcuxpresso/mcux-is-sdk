/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
*** TEXT BELOW IS USED AS SETTING FOR THE PINS TOOL ***
PinsProfile:
- !!Product 'Pins,v 1.0'
- !!Processor 'MK64FN1M0xxx12'
- !!Variant 'MK64FN1M0VLQ12'
- !!SWtoolDataID 'ksdk2_0'
- !!ProcessorDBVersion '0.1.0'
*** BE CAREFUL MODIFYING ABOVE TEXT AS USED AS SETTING FOR THE PINS TOOL ***
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

/*FUNCTION*********************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
*** TEXT BELOW IS USED AS SETTING FOR THE PINS TOOL ***
BOARD_InitPins:
- options: {coreID: singlecore, enableClock: 'true'}
- pin_list:
  - {peripheral: I2C0, signal: SCL, pin_signal: '[45] ADC0_SE17/PTE24/UART4_TX/I2C0_SCL/EWM_OUT_b', open_drain: enable, pull_select: up}
  - {peripheral: I2C0, signal: SDA, pin_signal: '[46] ADC0_SE18/PTE25/UART4_RX/I2C0_SDA/EWM_IN', open_drain: enable, pull_select: up}
  - {peripheral: I2C1, signal: SCL, pin_signal: '[115] ADC1_SE6b/PTC10/I2C1_SCL/FTM3_CH6/I2S0_RX_FS/FB_AD5', open_drain: enable, pull_select: up}
  - {peripheral: I2C1, signal: SDA, pin_signal: '[116] ADC1_SE7b/PTC11/LLWU_P11/I2C1_SDA/FTM3_CH7/I2S0_RXD1/FB_RW_b', open_drain: enable}
  - {peripheral: UART0, signal: RX, pin_signal: '[95] PTB16/SPI1_SOUT/UART0_RX/FTM_CLKIN0/FB_AD17/EWM_IN'}
  - {peripheral: UART0, signal: TX, pin_signal: '[96] PTB17/SPI1_SIN/UART0_TX/FTM_CLKIN1/FB_AD16/EWM_OUT_b'}
  - {peripheral: GPIOB, signal: 'GPIO, 9', pin_signal: '[90] PTB9/SPI1_PCS1/UART3_CTS_b/FB_AD20'}
  - {peripheral: GPIOB, signal: 'GPIO, 10', pin_signal: '[91] ADC1_SE14/PTB10/SPI1_PCS0/UART3_RX/FB_AD19/FTM0_FLT1'}
  - {peripheral: GPIOB, signal: 'GPIO, 23', pin_signal: '[102] PTB23/SPI2_SIN/SPI0_PCS5/FB_AD28'}
  - {peripheral: GPIOA, signal: 'GPIO, 0', pin_signal: '[50] PTA0/UART0_CTS_b/UART0_COL_b/FTM0_CH5/JTAG_TCLK/SWD_CLK/EZP_CLK'}
  - {peripheral: GPIOA, signal: 'GPIO, 2', pin_signal: '[52] PTA2/UART0_TX/FTM0_CH7/JTAG_TDO/TRACE_SWO/EZP_DO'}
  - {peripheral: GPIOB, signal: 'GPIO, 21', pin_signal: '[100] PTB21/SPI2_SCK/FB_AD30/CMP1_OUT'}
  - {peripheral: GPIOB, signal: 'GPIO, 22', pin_signal: '[101] PTB22/SPI2_SOUT/FB_AD29/CMP2_OUT'}
  - {peripheral: GPIOC, signal: 'GPIO, 6', pin_signal: '[111] CMP0_IN0/PTC6/LLWU_P10/SPI0_SOUT/PDB0_EXTRG/I2S0_RX_BCLK/FB_AD9/I2S0_MCLK'}
  - {peripheral: GPIOD, signal: 'GPIO, 0', pin_signal: '[127] PTD0/LLWU_P12/SPI0_PCS0/UART2_RTS_b/FTM3_CH0/FB_ALE/FB_CS1_b/FB_TS_b'}
  - {peripheral: GPIOE, signal: 'GPIO, 26', pin_signal: '[47] PTE26/ENET_1588_CLKIN/UART4_CTS_b/RTC_CLKOUT/USB_CLKIN'}
*** BE CAREFUL MODIFYING ABOVE TEXT AS USED AS SETTING FOR THE PINS TOOL ***
 *END*************************************************************************/
void BOARD_InitPins(void) {

    /* Initialize UART0 pins below */
    /* Ungate the port clock */
    CLOCK_EnableClock(kCLOCK_PortB);                 /* Port B Clock Gate Control: Clock enabled */
    /* Initialize UART0 pins below */
    PORT_SetPinMux(PORTB, PIN16_IDX, kPORT_MuxAlt3); /* PORTB16 (pin 62) is configured as UART0_RX */
    PORT_SetPinMux(PORTB, PIN17_IDX, kPORT_MuxAlt3); /* PORTB17 (pin 63) is configured as UART0_TX */

    /* Ungate the I2C0 port clock */
    CLOCK_EnableClock(kCLOCK_PortE);
    /* I2C0 PIN_MUX Configuration and pull up resister setting */
    PORT_SetPinMux(PORTE, PIN24_IDX, kPORT_MuxAlt5); /* PORTE24 (pin 45) is configured as I2C0_SCL */
    PORTE->PCR[24] = ((PORTE->PCR[24] &
      (~(PORT_PCR_PS_MASK | PORT_PCR_ODE_MASK)))     /* Mask bits to zero which are setting */
        | PORT_PCR_PS(PCR_PS_UP)                     /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
        | PORT_PCR_ODE(PCR_ODE_ENABLED)              /* Open Drain Enable: Open drain output is enabled on the corresponding pin, if the pin is configured as a digital output. */
       );
    PORT_SetPinMux(PORTE, PIN25_IDX, kPORT_MuxAlt5); /* PORTE25 (pin 46) is configured as I2C0_SDA */
    PORTE->PCR[25] = ((PORTE->PCR[25] &
      (~(PORT_PCR_PS_MASK | PORT_PCR_ODE_MASK)))     /* Mask bits to zero which are setting */
        | PORT_PCR_PS(PCR_PS_UP)                     /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
        | PORT_PCR_ODE(PCR_ODE_ENABLED)              /* Open Drain Enable: Open drain output is enabled on the corresponding pin, if the pin is configured as a digital output. */
      );

    /* Ungate the I2C1 port clock */
    CLOCK_EnableClock(kCLOCK_PortC);                 /* Port C Clock Gate Control: Clock enabled */
    /* I2C1 PIN_MUX Configuration and pull up resister setting */
    PORT_SetPinMux(PORTC, PIN10_IDX, kPORT_MuxAlt2); /* PORTC10 (pin 115) is configured as I2C1_SCL */
    PORTC->PCR[10] = ((PORTC->PCR[10] &
      (~(PORT_PCR_PS_MASK | PORT_PCR_ODE_MASK)))     /* Mask bits to zero which are setting */
        | PORT_PCR_PS(PCR_PS_UP)                     /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
        | PORT_PCR_ODE(PCR_ODE_ENABLED)              /* Open Drain Enable: Open drain output is enabled on the corresponding pin, if the pin is configured as a digital output. */
      );
    PORT_SetPinMux(PORTC, PIN11_IDX, kPORT_MuxAlt2); /* PORTC11 (pin 116) is configured as I2C1_SDA */
    PORTC->PCR[11] = ((PORTC->PCR[11] &
      (~(PORT_PCR_ODE_MASK)))                        /* Mask bits to zero which are setting */
        | PORT_PCR_PS(PCR_PS_UP)                     /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
        | PORT_PCR_ODE(PCR_ODE_ENABLED)              /* Open Drain Enable: Open drain output is enabled on the corresponding pin, if the pin is configured as a digital output. */
      );

    /* Initialize UART3 pins below */
    PORT_SetPinMux(PORTC, PIN16_IDX, kPORT_MuxAlt3); /* PORTC16 (pin 90) is configured as UART3_RX */
    PORT_SetPinMux(PORTC, PIN17_IDX, kPORT_MuxAlt3); /* PORTC17 (pin 91) is configured as UART3_TX */

    /* Ungate the SPI port clock */
    CLOCK_EnableClock(kCLOCK_PortD);                 /* Port D Clock Gate Control: Clock enabled */
    /* SPI1 PIN_MUX Configuration */
    PORT_SetPinMux(PORTD, PIN5_IDX, kPORT_MuxAlt7);
    PORT_SetPinMux(PORTD, PIN6_IDX, kPORT_MuxAlt7);
    PORT_SetPinMux(PORTD, PIN7_IDX, kPORT_MuxAlt7);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

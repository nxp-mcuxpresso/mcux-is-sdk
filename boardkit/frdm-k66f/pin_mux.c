/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*
 * TEXT BELOW IS USED AS SETTING FOR THE PINS TOOL *****************************
PinsProfile:
- !!product 'Pins v1.0'
- !!processor 'MK66FN2M0xxx18'
- !!package 'MK66FN2M0VMD18'
- !!mcu_data 'ksdk2_0'
- !!processor_version '1.0.0'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE PINS TOOL ***
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"


/*
 * TEXT BELOW IS USED AS SETTING FOR THE PINS TOOL *****************************
BOARD_InitPins:
- options: {coreID: singlecore, enableClock: 'true'}
- pin_list:
  - {pin_num: C9, peripheral: I2C0, signal: SCL, pin_signal: PTD8/LLWU_P24/I2C0_SCL/LPUART0_RX/FB_A16, open_drain: enable, pull_select: up, pull_enable: enable}
  - {pin_num: B9, peripheral: I2C0, signal: SDA, pin_signal: PTD9/I2C0_SDA/LPUART0_TX/FB_A17, open_drain: enable, pull_select: up, pull_enable: enable}
  - {pin_num: C7, peripheral: I2C1, signal: SCL, pin_signal: ADC1_SE6b/PTC10/I2C1_SCL/FTM3_CH6/I2S0_RX_FS/FB_AD5/SDRAM_A13, slew_rate: fast, open_drain: enable, pull_select: up,
    pull_enable: enable}
  - {pin_num: B7, peripheral: I2C1, signal: SDA, pin_signal: ADC1_SE7b/PTC11/LLWU_P11/I2C1_SDA/FTM3_CH7/I2S0_RXD1/FB_RW_b, slew_rate: fast, open_drain: enable, pull_select: up,
    pull_enable: enable}
  - {pin_num: C8, peripheral: I2S0, signal: MCLK, pin_signal: CMP0_IN0/PTC6/LLWU_P10/SPI0_SOUT/PDB0_EXTRG/I2S0_RX_BCLK/FB_AD9/SDRAM_A17/I2S0_MCLK}
  - {pin_num: G3, peripheral: I2S0, signal: TX_BCLK, pin_signal: PTE12/I2S0_TX_BCLK/FTM3_CH7}
  - {pin_num: G4, peripheral: I2S0, signal: TX_FS, pin_signal: PTE11/I2C3_SCL/I2S0_TX_FS/LPUART0_RTS_b/FTM3_CH6}
  - {pin_num: B11, peripheral: I2S0, signal: TXD0, pin_signal: ADC0_SE15/TSI0_CH14/PTC1/LLWU_P6/SPI0_PCS3/UART1_RTS_b/FTM0_CH0/FB_AD13/SDRAM_A21/I2S0_TXD0}
  - {pin_num: F4, peripheral: I2S0, signal: RXD0, pin_signal: UART3_RTS/I2S0_RXD0/FTM3_CH2}
  - {pin_num: L9, peripheral: GPIOA, signal: 'GPIO, 11', pin_signal: PTA11/LLWU_P23/FTM2_CH1/MII0_RXCLK/I2C2_SDA/FTM2_QD_PHB/TPM2_CH1, direction: OUTPUT}
  - {pin_num: D7, peripheral: GPIOC, signal: 'GPIO, 9', pin_signal: ADC1_SE5b/CMP0_IN3/PTC9/FTM3_CH5/I2S0_RX_BCLK/FB_AD6/SDRAM_A14/FTM2_FLT0, direction: OUTPUT}
  - {pin_num: E1, peripheral: GPIOE, signal: 'GPIO, 6', pin_signal: PTE6/LLWU_P16/SPI1_PCS3/UART3_CTS_b/I2S0_MCLK/FTM3_CH1/USB0_SOF_OUT, direction: OUTPUT}
  - {pin_num: M12, peripheral: OSC, signal: EXTAL0, pin_signal: EXTAL0/PTA18/FTM0_FLT2/FTM_CLKIN0/TPM_CLKIN0}
  - {pin_num: M11, peripheral: OSC, signal: XTAL0, pin_signal: XTAL0/PTA19/FTM1_FLT0/FTM_CLKIN1/LPTMR0_ALT1/TPM_CLKIN1}
  - {pin_num: E4, peripheral: SDHC, signal: CMD, pin_signal: ADC1_SE7a/PTE3/SPI1_SIN/UART1_RTS_b/SDHC0_CMD/TRACE_D1/SPI1_SOUT}
  - {pin_num: D2, peripheral: SDHC, signal: 'DATA, 0', pin_signal: ADC1_SE5a/PTE1/LLWU_P0/SPI1_SOUT/UART1_RX/SDHC0_D0/TRACE_D3/I2C1_SCL/SPI1_SIN}
  - {pin_num: D3, peripheral: SDHC, signal: 'DATA, 1', pin_signal: ADC1_SE4a/PTE0/SPI1_PCS1/UART1_TX/SDHC0_D1/TRACE_CLKOUT/I2C1_SDA/RTC_CLKOUT}
  - {pin_num: E2, peripheral: SDHC, signal: 'DATA, 2', pin_signal: PTE5/SPI1_PCS2/UART3_RX/SDHC0_D2/FTM3_CH0}
  - {pin_num: E3, peripheral: SDHC, signal: 'DATA, 3', pin_signal: PTE4/LLWU_P2/SPI1_PCS0/UART3_TX/SDHC0_D3/TRACE_D0}
  - {pin_num: D1, peripheral: SDHC, signal: DCLK, pin_signal: ADC1_SE6a/PTE2/LLWU_P1/SPI1_SCK/UART1_CTS_b/SDHC0_DCLK/TRACE_D2}
  - {pin_num: B3, peripheral: GPIOD, signal: 'GPIO, 10', pin_signal: PTD10/LPUART0_RTS_b/FB_A18, direction: INPUT, pull_select: down}
  - {pin_num: E10, peripheral: UART0, signal: RX, pin_signal: TSI0_CH9/PTB16/SPI1_SOUT/UART0_RX/FTM_CLKIN0/FB_AD17/SDRAM_D17/EWM_IN/TPM_CLKIN0}
  - {pin_num: E9, peripheral: UART0, signal: TX, pin_signal: TSI0_CH10/PTB17/SPI1_SIN/UART0_TX/FTM_CLKIN1/FB_AD16/SDRAM_D16/EWM_OUT_b/TPM_CLKIN1}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE PINS TOOL ***
 */
/*FUNCTION**********************************************************************
 *
 * Function Name : I2C0_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void I2C0_InitPins(void)
{
}
void I2C0_DeinitPins(void)
{
}
void UART0_DeinitPins(void)
{
    PORT_SetPinMux(PORTB, PIN16_IDX, kPORT_PinDisabledOrAnalog); /* PORTB16 (pin 95) is disabled */
    PORT_SetPinMux(PORTB, PIN17_IDX, kPORT_PinDisabledOrAnalog); /* PORTB17 (pin 96) is disabled */
}

void UART0_InitPins(void)
{
    CLOCK_EnableClock(kCLOCK_PortB); /* Port B Clock Gate Control: Clock enabled */
    PORT_SetPinMux(PORTB, PIN16_IDX, kPORT_MuxAlt3);           /* PORTB16 (pin E10) is configured as UART0_RX */
    PORT_SetPinMux(PORTB, PIN17_IDX, kPORT_MuxAlt3);           /* PORTB17 (pin E9) is configured as UART0_TX */
}

void SAI_InitPins(void)
{
    PORT_SetPinMux(PORTC, PIN1_IDX, kPORT_MuxAlt6);            /* PORTC1 (pin B11) is configured as I2S0_TXD0 */
    PORT_SetPinMux(PORTE, PIN7_IDX, kPORT_MuxAlt4);            /* PORTE7 (pin F4) is configured as I2S0_RXD0 */
    const port_pin_config_t portc10_pinC7_config = {
        kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
        kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
        kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
        kPORT_OpenDrainEnable,                                   /* Open drain is enabled */
        kPORT_LowDriveStrength,                                  /* Low drive strength is configured */
        kPORT_MuxAlt2,                                           /* Pin is configured as I2C1_SCL */
        kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
    };
    PORT_SetPinConfig(PORTC, PIN10_IDX, &portc10_pinC7_config); /* PORTC10 (pin C7) is configured as I2C1_SCL */
    const port_pin_config_t portc11_pinB7_config = {
        kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
        kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
        kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
        kPORT_OpenDrainEnable,                                   /* Open drain is enabled */
        kPORT_LowDriveStrength,                                  /* Low drive strength is configured */
        kPORT_MuxAlt2,                                           /* Pin is configured as I2C1_SDA */
        kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
    };
    PORT_SetPinConfig(PORTC, PIN11_IDX, &portc11_pinB7_config); /* PORTC11 (pin B7) is configured as I2C1_SDA */
    PORT_SetPinMux(PORTC, PIN6_IDX, kPORT_MuxAlt6);            /* PORTC6 (pin C8) is configured as I2S0_MCLK */
    PORT_SetPinMux(PORTE, PIN11_IDX, kPORT_MuxAlt4);           /* PORTE11 (pin G4) is configured as I2S0_TX_FS */
    PORT_SetPinMux(PORTE, PIN12_IDX, kPORT_MuxAlt4);           /* PORTE12 (pin G3) is configured as I2S0_TX_BCLK */
}

void SAI_DeinitPins(void)
{
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitPins(void) {
  CLOCK_EnableClock(kCLOCK_PortA);                           /* Port A Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortB);                           /* Port B Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortC);                           /* Port C Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortD);                           /* Port D Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortE);                           /* Port E Clock Gate Control: Clock enabled */

  UART0_InitPins();
  SAI_InitPins();

  PORT_SetPinMux(PORTA, PIN11_IDX, kPORT_MuxAsGpio);         /* PORTA11 (pin L9) is configured as PTA11 */
  PORT_SetPinMux(PORTA, PIN18_IDX, kPORT_PinDisabledOrAnalog); /* PORTA18 (pin M12) is configured as EXTAL0 */
  PORT_SetPinMux(PORTA, PIN19_IDX, kPORT_PinDisabledOrAnalog); /* PORTA19 (pin M11) is configured as XTAL0 */
  PORT_SetPinMux(PORTC, PIN9_IDX, kPORT_MuxAsGpio);          /* PORTC9 (pin D7) is configured as PTC9 */
  PORT_SetPinMux(PORTD, PIN10_IDX, kPORT_MuxAsGpio);         /* PORTD10 (pin B3) is configured as PTD10 */
  PORTD->PCR[10] = ((PORTD->PCR[10] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_ISF_MASK)))               /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_DOWN)                             /* Pull Select: Internal pulldown resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
    );
  PORT_SetPinMux(PORTD, PIN8_IDX, kPORT_MuxAlt2);            /* PORTD8 (pin C9) is configured as I2C0_SCL */
  PORTD->PCR[8] = ((PORTD->PCR[8] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ODE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
      | PORT_PCR_ODE(PCR_ODE_ENABLED)                        /* Open Drain Enable: Open drain output is enabled on the corresponding pin, if the pin is configured as a digital output. */
    );
  PORT_SetPinMux(PORTD, PIN9_IDX, kPORT_MuxAlt2);            /* PORTD9 (pin B9) is configured as I2C0_SDA */
  PORTD->PCR[9] = ((PORTD->PCR[9] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ODE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
      | PORT_PCR_PE(PCR_PE_ENABLED)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
      | PORT_PCR_ODE(PCR_ODE_ENABLED)                        /* Open Drain Enable: Open drain output is enabled on the corresponding pin, if the pin is configured as a digital output. */
    );
  PORT_SetPinMux(PORTE, PIN0_IDX, kPORT_MuxAlt4);            /* PORTE0 (pin D3) is configured as SDHC0_D1 */
  PORT_SetPinMux(PORTE, PIN1_IDX, kPORT_MuxAlt4);            /* PORTE1 (pin D2) is configured as SDHC0_D0 */
  PORT_SetPinMux(PORTE, PIN2_IDX, kPORT_MuxAlt4);            /* PORTE2 (pin D1) is configured as SDHC0_DCLK */
  PORT_SetPinMux(PORTE, PIN3_IDX, kPORT_MuxAlt4);            /* PORTE3 (pin E4) is configured as SDHC0_CMD */
  PORT_SetPinMux(PORTE, PIN4_IDX, kPORT_MuxAlt4);            /* PORTE4 (pin E3) is configured as SDHC0_D3 */
  PORT_SetPinMux(PORTE, PIN5_IDX, kPORT_MuxAlt4);            /* PORTE5 (pin E2) is configured as SDHC0_D2 */
  PORT_SetPinMux(PORTE, PIN6_IDX, kPORT_MuxAsGpio);          /* PORTE6 (pin E1) is configured as PTE6 */
  SIM->SOPT5 = ((SIM->SOPT5 &
    (~(SIM_SOPT5_UART0TXSRC_MASK)))                          /* Mask bits to zero which are setting */
      | SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX)       /* UART 0 transmit data source select: UART0_TX pin */
    );
}



/*******************************************************************************
 * EOF
 ******************************************************************************/

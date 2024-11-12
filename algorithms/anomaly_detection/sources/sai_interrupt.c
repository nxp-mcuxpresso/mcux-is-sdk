/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
//#include "music.h"
#include "fsl_sai.h"
#include "fsl_debug_console.h"

#include "fsl_dialog7212.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "anomaly_detection.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* SAI and I2C instance and clock */
#define DEMO_CODEC_DA7212
#define DEMO_SAI I2S0
#define DEMO_SAI_CHANNEL (0)
#define DEMO_SAI_BITWIDTH (kSAI_WordWidth16bits)
#define DEMO_I2C I2C1
#define DEMO_SAI_CLKSRC kCLOCK_CoreSysClk
#define DEMO_SAI_CLK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)
#define DEMO_I2C_CLKSRC kCLOCK_BusClk
#define DEMO_I2C_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define DEMO_SAI_IRQ I2S0_Tx_IRQn
#define SAI_TxIRQHandler I2S0_Tx_IRQHandler
#define SAI_RxIRQHandler I2S0_Rx_IRQHandler
#define DEMO_SAI_IRQ_RX I2S0_Rx_IRQn


#define I2C_RELEASE_SDA_PORT PORTC
#define I2C_RELEASE_SCL_PORT PORTC
#define I2C_RELEASE_SDA_GPIO GPIOC
#define I2C_RELEASE_SDA_PIN 11U
#define I2C_RELEASE_SCL_GPIO GPIOC
#define I2C_RELEASE_SCL_PIN 10U
#define I2C_RELEASE_BUS_COUNT 100U
#define OVER_SAMPLE_RATE (384U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_I2C_ReleaseBus(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static size_t g_index = 0;
static volatile bool isTxFinished = false;
static volatile bool isRxFinished = false;
#if defined(DEMO_CODEC_WM8960)
wm8960_handle_t codecHandle = {0};
#elif defined (DEMO_CODEC_DA7212)
da7212_handle_t codecHandle = {0};
#else
sgtl_handle_t codecHandle = {0};
#endif

#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
lpi2c_master_handle_t i2cHandle;
#else
i2c_master_handle_t i2cHandle;
#endif

#define MUSIC_LEN (48000)
#define B_SIZE  (FSL_FEATURE_SAI_FIFO_COUNT * (DEMO_SAI_BITWIDTH / 8U))
uint8_t audio_test[MUSIC_LEN];

/*******************************************************************************
 * Code
 ******************************************************************************/

void SAI_TxIRQHandler(void)
{
    uint8_t i = 0;
    uint8_t j = 0;
    uint32_t data = 0;
    uint32_t temp = 0;

    /* Clear the FIFO error flag */
    if (SAI_TxGetStatusFlag(DEMO_SAI) & kSAI_FIFOErrorFlag)
    {
        SAI_TxClearStatusFlags(DEMO_SAI, kSAI_FIFOErrorFlag);
    }

    if (SAI_TxGetStatusFlag(DEMO_SAI) & kSAI_FIFOWarningFlag)
    {
        for (i = 0; i < FSL_FEATURE_SAI_FIFO_COUNT; i++)
        {
            data = 0;
            for (j = 0; j < DEMO_SAI_BITWIDTH / 8U; j++)
            {
                temp = (uint32_t)(audio_test[g_index]);
                data |= (temp << (8U * j));
                g_index++;
            }
            SAI_WriteData(DEMO_SAI, DEMO_SAI_CHANNEL, data);
        }
    }

        
    if (g_index >= MUSIC_LEN)
    {
        isTxFinished = true;
//        SAI_TxDisableInterrupts(DEMO_SAI, kSAI_FIFOWarningInterruptEnable | kSAI_FIFOErrorInterruptEnable);
//        SAI_TxEnable(DEMO_SAI, false);
    }
}

void SAI_RxIRQHandler(void)
{
    uint8_t i = 0;
    uint8_t j = 0;
    uint32_t data = 0;
    uint8_t buffer[B_SIZE];
    uint32_t buf_idx = 0;

    /* Clear the FIFO error flag */
    if (SAI_RxGetStatusFlag(DEMO_SAI) & kSAI_FIFOErrorFlag)
    {
        SAI_RxClearStatusFlags(DEMO_SAI, kSAI_FIFOErrorFlag);
    }

    if (SAI_RxGetStatusFlag(DEMO_SAI) & kSAI_FIFOWarningFlag)
    {
        for (i = 0; i < FSL_FEATURE_SAI_FIFO_COUNT; i++)
        {
            data = SAI_ReadData(DEMO_SAI, DEMO_SAI_CHANNEL);
            for (j = 0; j < DEMO_SAI_BITWIDTH / 8U; j++)
            {
                buffer[buf_idx] = (data >> (8U * j)) & 0xFF;
                audio_test[g_index] = buffer[buf_idx];           // this is for test.
                buf_idx++;
                g_index++;                                       // this is for test.
            }
            
        }
    }
    isRxFinished = true;
    g_index = 0;
    
//    /* this is for test. remove the audio_test array and save the buffer in the data structure of ADT project. */
//    if (g_index >= MUSIC_LEN)
//    {
//        isRxFinished = true;
////        SAI_RxDisableInterrupts(DEMO_SAI, kSAI_FIFOWarningInterruptEnable | kSAI_FIFOErrorInterruptEnable);
////        SAI_RxEnable(DEMO_SAI, false);
//        g_index = 0;
//    }
//    /*   end of test     */
    
}

/*!
 * @brief Main function
 */
//int main(void)
void initializeSai(Globals *gbls)
{
    sai_config_t config;
    uint32_t mclkSourceClockHz = 0U;
    sai_transfer_format_t format;
#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    lpi2c_master_config_t i2cConfig = {0};
#else
    i2c_master_config_t i2cConfig = {0};
#endif
    uint32_t i2cSourceClock;

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    PRINTF("SAI functional interrupt example started!\n\r");

    /*
     * config.masterSlave = kSAI_Master;
     * config.mclkSource = kSAI_MclkSourceSysclk;
     * config.protocol = kSAI_BusLeftJustified;
     * config.syncMode = kSAI_ModeAsync;
     * config.mclkOutputEnable = true;
     */
    SAI_TxGetDefaultConfig(&config);    
    SAI_RxGetDefaultConfig(&config);
    SAI_TxInit(DEMO_SAI, &config);
    SAI_RxInit(DEMO_SAI, &config);

    /* Configure the audio format */
    format.bitWidth = DEMO_SAI_BITWIDTH;
    format.channel = DEMO_SAI_CHANNEL;
//    format.sampleRate_Hz = kSAI_SampleRate16KHz;
    format.sampleRate_Hz = kSAI_SampleRate8KHz;
#if (defined FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER && FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER) || \
    (defined FSL_FEATURE_PCC_HAS_SAI_DIVIDER && FSL_FEATURE_PCC_HAS_SAI_DIVIDER)
    format.masterClockHz = OVER_SAMPLE_RATE * format.sampleRate_Hz;
#else
    format.masterClockHz = DEMO_SAI_CLK_FREQ;
#endif
    format.protocol = config.protocol;
    format.stereo = kSAI_Stereo;

    /* Configure Codec I2C */
    codecHandle.base = DEMO_I2C;
    codecHandle.i2cHandle = &i2cHandle;
    i2cSourceClock = DEMO_I2C_CLK_FREQ;

#if defined(FSL_FEATURE_SOC_LPI2C_COUNT) && (FSL_FEATURE_SOC_LPI2C_COUNT)
    /*
     * i2cConfig.debugEnable = false;
     * i2cConfig.ignoreAck = false;
     * i2cConfig.pinConfig = kLPI2C_2PinOpenDrain;
     * i2cConfig.baudRate_Hz = 100000U;
     * i2cConfig.busIdleTimeout_ns = 0;
     * i2cConfig.pinLowTimeout_ns = 0;
     * i2cConfig.sdaGlitchFilterWidth_ns = 0;
     * i2cConfig.sclGlitchFilterWidth_ns = 0;
     */
    LPI2C_MasterGetDefaultConfig(&i2cConfig);
    LPI2C_MasterInit(DEMO_I2C, &i2cConfig, i2cSourceClock);
    LPI2C_MasterTransferCreateHandle(DEMO_I2C, &i2cHandle, NULL, NULL);
#else
    /*
     * i2cConfig.baudRate_Bps = 100000U;
     * i2cConfig.enableStopHold = false;
     * i2cConfig.glitchFilterWidth = 0U;
     * i2cConfig.enableMaster = true;
     */
    I2C_MasterGetDefaultConfig(&i2cConfig);
    I2C_MasterInit(DEMO_I2C, &i2cConfig, i2cSourceClock);
    I2C_MasterTransferCreateHandle(DEMO_I2C, &i2cHandle, NULL, NULL);
#endif

#if defined(DEMO_CODEC_WM8960)
    WM8960_Init(&codecHandle, NULL);
    WM8960_ConfigDataFormat(&codecHandle, format.masterClockHz, format.sampleRate_Hz, format.bitWidth);
#elif defined (DEMO_CODEC_DA7212)
    DA7212_Init(&codecHandle, NULL);
    DA7212_ConfigAudioFormat(&codecHandle, format.sampleRate_Hz, format.masterClockHz, format.bitWidth);
    DA7212_ChangeInput(&codecHandle, kDA7212_Input_MIC1_Dig);
    DA7212_ChangeOutput(&codecHandle, kDA7212_Output_HP);
    DA7212_ChangeHPVolume(&codecHandle, kDA7212_DACGain0DB);
#else
    /* Use default settings for sgtl5000 */
    SGTL_Init(&codecHandle, NULL);
    /* Configure codec format */
    SGTL_ConfigDataFormat(&codecHandle, format.masterClockHz, format.sampleRate_Hz, format.bitWidth);
#endif
    mclkSourceClockHz = DEMO_SAI_CLK_FREQ;
    SAI_TxSetFormat(DEMO_SAI, &format, mclkSourceClockHz, format.masterClockHz);
    SAI_RxSetFormat(DEMO_SAI, &format, mclkSourceClockHz, format.masterClockHz);

    /* Enable Rx Interrupt */
    EnableIRQ(DEMO_SAI_IRQ_RX);
//    SAI_RxEnableInterrupts(DEMO_SAI, kSAI_FIFOWarningInterruptEnable | kSAI_FIFOErrorInterruptEnable);
//    SAI_RxEnable(DEMO_SAI, true);
//     /* Wait until finished */
//    while (isRxFinished != true)
//    {
//    }
    
    /*  Enable interrupt */
    EnableIRQ(DEMO_SAI_IRQ);
//    SAI_TxEnableInterrupts(DEMO_SAI, kSAI_FIFOWarningInterruptEnable | kSAI_FIFOErrorInterruptEnable);
//    SAI_TxEnable(DEMO_SAI, true);   
//    /* Wait until finished */
//    while (isTxFinished != true)
//    {
//    }

//    PRINTF("\n\r SAI functional interrupt example finished!\n\r ");
//    while (1)
//    {
//    }
}

void SAI_RxListen(uint8_t *adata)
{ 
    SAI_RxEnableInterrupts(DEMO_SAI, kSAI_FIFOWarningInterruptEnable | kSAI_FIFOErrorInterruptEnable);
    SAI_RxEnable(DEMO_SAI, true);
    while (isRxFinished != true)
    {
    } 
    memcpy(adata, &audio_test[0], B_SIZE);
//    SAI_RxDisableInterrupts(DEMO_SAI, kSAI_FIFOWarningInterruptEnable | kSAI_FIFOErrorInterruptEnable);
//    SAI_RxEnable(DEMO_SAI, false);
}

void SAI_RxDisable(void)
{
    SAI_RxDisableInterrupts(DEMO_SAI, kSAI_FIFOWarningInterruptEnable | kSAI_FIFOErrorInterruptEnable);
    SAI_RxEnable(DEMO_SAI, false);
}

void SAI_TxPlay(void)
{ 
    SAI_TxEnableInterrupts(DEMO_SAI, kSAI_FIFOWarningInterruptEnable | kSAI_FIFOErrorInterruptEnable);
    SAI_TxEnable(DEMO_SAI, true);
    while (isTxFinished != true)
    {
    }
    SAI_TxDisableInterrupts(DEMO_SAI, kSAI_FIFOWarningInterruptEnable | kSAI_FIFOErrorInterruptEnable);
    SAI_TxEnable(DEMO_SAI, false);
}




/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file gpio_driver_irq.c
 * @brief The gpio_driver_irq.c file containes the Generic IRQ implmentation for GPIO.
*/

#include "fsl_rgpio.h"
#include "gpio_driver.h"

extern void imxsdk_gpio_handle_interrupt(RGPIO_Type *base, port_number_t gpioPortNumber);

/*******************************************************************************
 * Functions - GPIOIRQ implementation
 ******************************************************************/

void GPIO1_0_IRQHandler (void)
{
    imxsdk_gpio_handle_interrupt(RGPIO1, GPIO1_NUM);
}

void GPIO1_1_IRQHandler (void)
{
    imxsdk_gpio_handle_interrupt(RGPIO1, GPIO1_NUM);
}

void GPIO2_0_IRQHandler (void)
{
    imxsdk_gpio_handle_interrupt(RGPIO2, GPIO2_NUM);
}

void GPIO2_1_IRQHandler (void)
{
    imxsdk_gpio_handle_interrupt(RGPIO2, GPIO2_NUM);
}

void GPIO3_0_IRQHandler (void)
{
    imxsdk_gpio_handle_interrupt(RGPIO3, GPIO3_NUM);
}

void GPIO3_1_IRQHandler (void)
{
    imxsdk_gpio_handle_interrupt(RGPIO3, GPIO3_NUM);
}

//#if !defined(CPU_MIMXRT1021DAG5A) && !defined(CPU_MIMXRT1024DAG5A)
void GPIO4_0_IRQHandler (void)
{
    imxsdk_gpio_handle_interrupt(RGPIO4, GPIO4_NUM);
}

void GPIO4_1_IRQHandler (void)
{
    imxsdk_gpio_handle_interrupt(RGPIO4, GPIO4_NUM);
}
//#endif

void GPIO5_0_IRQHandler (void)
{
    imxsdk_gpio_handle_interrupt(RGPIO5, GPIO5_NUM);
}

void GPIO5_1_IRQHandler (void)
{
    imxsdk_gpio_handle_interrupt(RGPIO5, GPIO5_NUM);
}
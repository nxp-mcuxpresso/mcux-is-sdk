/*
* Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/



#ifndef SAI_INTERRUPT_H
#define SAI_INTERRUPT_H

void initializeSai(Globals *gbls);
void SAI_RxListen(uint8_t *adata);
void SAI_RxDisable(void);
void SAI_TxPlay(void);


#endif  // SAI_INTERRUPT_H
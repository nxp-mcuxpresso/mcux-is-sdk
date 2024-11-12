/*
* Copyright (c) 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/* Anomaly Detection Headers */
#include "anomaly_detection.h"

#define MAX_FEATURE_INSTANCES 10

struct FEATURE_LIST {
  bool          in_use;
  sensor_t      sensor;
  axis_t        axis;
  feature_t     feature;
};
struct FEATURE_LIST feature_list[MAX_FEATURE_INSTANCES];
void initialize_feature_list() {
    for (int i=0; i< MAX_FEATURE_INSTANCES; i++) {
        feature_list[i].in_use=false;
        feature_list[i].sensor=ACCEL;   // Meaningless until in_use set to true
        feature_list[i].axis=CHX;       // Meaningless until in_use set to true
        feature_list[i].feature=MEAN;   // Meaningless until in_use set to true
    }
}

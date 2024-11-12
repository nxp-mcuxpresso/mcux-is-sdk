/*
* Copyright (c) 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/**
 * @file process_host_command.c
 * @brief The process_host_command.c file implements the embedded functional interfaces and host i/o interface.
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"

/* KSDK Headers */
#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_port.h"
#include "fsl_i2c.h"

/* CMSIS Headers */
#include "Driver_USART.h"
#include "fsl_i2c_cmsis.h"
#include "fsl_uart_cmsis.h"

/* ISSDK Headers */
#include "issdk_hal.h"
#include "fxas21002.h"
#include "fxos8700.h"
#include "register_io_i2c.h"
#include "host_io_uart.h"
#include "gpio_driver.h"

/* Sensor Fusion Headers */
#include "anomaly_detection.h"
#include "control.h"
#include "status.h"
#include "drivers.h"

#define APPLICATION_NAME "No Shield:Anomaly Detection:0.1"                     /* Orientation Application Name */

#define DECODE_DIM(X) ((X&0X10)>>4)
#define DECODE_DIM_SVM(X)       ((X&0x80)>>7)
#define DECODE_CLR(X) (X&0x01)
#define DECODE_STOP(X) ((X&0x02)>>1)
#define DECODE_RUN(X) ((X&0x04)>>2)
#define DECODE_TRAIN(X) ((X&0x08)>>3)
#define DECODE_DELETE(X) ((X&0x10)>>4)
#define DECODE_DELETE_ALL(X) ((X&0x20)>>5)
#define DECODE_TED(X)   ((X&0x40)>>6)
#define DECODE_STREAM_ENABLE(X) ((X&0x80)>>7)
#define UPPER_NIBBLE(X) (X>>4)
#define LOWER_NIBBLE(X) (X&0x0F)
#define SHORT(X,Y) ((X<<8) + Y)

extern Globals gbls;                ///< This is the primary sensor fusion data structure

int8_t current_model_id=0x00;

void getEncodedSampleRates(uint32_t *sampleRates) {
  sampleRates[0] = ACCEL_ODR_HZ;
  sampleRates[1] = GYRO_ODR_HZ;
  sampleRates[2] = TEMP_ODR_HZ;
  sampleRates[3] = PRESSURE_ODR_HZ;
  sampleRates[4] = MICROPHONE_ODR_HZ;
}

bool setclr_feature(sensor_t sensor, feature_t feature, axis_t axis, bool sts)
{
  uint8_t feature_number;
  if (sts) {
      return(add_feature(&feature_number, gbls.pFeatureCollection, sensor, axis, feature));
  } else {
      return(delete_feature_by_attributes(gbls.pFeatureCollection, sensor, axis, feature));
  }
}

bool setFeatureInterval(int32_t featureInterval) {
  // TBD: this function is currently undefined and unused. But keep this function for later use. It may be used to set feature interval.
    return true;
}

void setStatusLLModels(struct ModelCollection *modelCollection, uint8_t command)
{
    bool isEnabled;
    uint8_t model_idx;

    for (int i=0; i< modelCollection->num_LL_models; i++) {
        if (get_model_idx_instance(&model_idx, &isEnabled, modelCollection, modelCollection->model_IDs_ensemble[i])) {
            modelCollection->pModelInstance[model_idx]->iStage = command;
        } else {
            gbls.setStatus(&gbls, HARD_FAULT); // never returns
        }
    }
}

/// @brief This function is used to process model mode commands.
///
/// With the received packet, model type, model ID, and appropriate command are set to a model instance.
///
/// @param ModelType    Model types.
///     @param  ModelNumber     Model ID.
///     @param  StreamEnable    An indicator for enabling streaming.
///     @param  command The commands such as training and run.
///     @return true / false    True if success.
bool processSetModelModeCommand(uint8_t ModelType, uint8_t ModelNumber, bool StreamEnable, uint8_t command) {
    bool isEnabled; // not needed here, but required by get_model_idx_instance
    uint8_t model_idx;
    gbls.current_model_id = ModelNumber;
    struct ModelCollection *modelCollection = gbls.pModelCollection;
    bool TED = DECODE_TED(command);
    bool STOP = DECODE_STOP(command);
    bool DELETE = DECODE_DELETE(command);
    bool DELETE_ALL = DECODE_DELETE_ALL(command);

    if (STOP)  {
        gbls.controlCommandforML = command;
        modelCollection->iStage = command;
        setStatusLLModels(modelCollection, command);
        disableAllModels(gbls.pModelCollection);
        return true;
    }
    else if (DELETE_ALL)  {             // what is this?
        nullModelCollection(gbls.pModelCollection);
        return true;
    }
    // using model ID for filtering different modes.
    else if (inEnsembleRange(ModelNumber)) {   // Range of ensemble model IDs
        gbls.controlCommandforML = command;
        modelCollection->ensemble_modelID = ModelNumber;                // this will differentiate the model result packet between LL and ensemble models.
        modelCollection->iStage = command;
        setStatusLLModels(modelCollection, command);                    // set the status of lower level models with the command.
        if (TED)   modelCollection->isEnsembleEnabled = true;
        if (DELETE) deleteModel(ModelNumber);
        return true;
    }
    else if(get_model_idx_instance(&model_idx, &isEnabled, gbls.pModelCollection, ModelNumber)){
      gbls.controlCommandforML = command;
      gbls.pModelCollection->pModelInstance[model_idx]->iStage = command;
      if (TED)  gbls.setStatus(&gbls, HARD_FAULT); // TED should have worked with the ensemble model id.
      if (DELETE) deleteModel(ModelNumber);
      return true;
    } else {
      gbls.setStatus(&gbls, HARD_FAULT); // never returns
      return false;
    }
}

/// @brief      This function is called when GMM configuration is processed.
///
///     The configuration is recieved from GUI and set to each model instance.
///
///     @param ModelType        Model type
///     @param  ModelNumber     Model ID.
///     @param  DIM             1 or 2 dimension of feature samples.
///     @param  MaxGaussians    The maximum number of Gaussian components in GMM usage.
///     @param  BufSize         The size of moving window in feature buffer.
///     @param  ComputeInterval The rate of training. At every ComputeInterval, training is executed.
///     @param  ThresholdX100   The treshold value assigned by a user. 100 times larger values is transmitted to use integer type rather than float and save the packet size.
///     @param  FeatureCode1    The first dimension of feature type.
///     @param  FeatureCode2    The second dimension of feature type.
///     @param  SensorCode1     The first dimension of sensor type.
///     @param  SensorCode2     The second dimension of sensor type.
///     @param  AxisCode1       The axis of SensorCode1.
///     @param  AxisCode2       The axis of SensorCode2.
bool processGmmConfigurationCommand(uint8_t ModelType, uint8_t ModelNumber, bool DIM,
              uint8_t MaxGaussians, uint16_t BufSize, uint16_t ComputeInterval, uint8_t ThresholdX100,
              feature_t FeatureCode1, feature_t FeatureCode2, sensor_t SensorCode1, sensor_t SensorCode2,
              axis_t AxisCode1, axis_t AxisCode2){
  uint8_t feature1, feature2;
  int8_t model_idx;
  gbls.controlCommandforML = NO_COMMAND; // nothing to be scheduled
  disableAllModels(gbls.pModelCollection);  // TBD: Currently disable all but the current model
  if (get_feature_number(&feature1, gbls.pFeatureCollection, SensorCode1, AxisCode1, FeatureCode1)) {
    gbls.pFeatureCollection->pFeatureInstance[feature1]->doNormalization = true ;
    gbls.pFeatureCollection->pFeatureInstance[feature1]->isEnabled = true;

    if (DIM) {
        if (!get_feature_number(&feature2, gbls.pFeatureCollection, SensorCode2, AxisCode2, FeatureCode2)) return false;
        gbls.pFeatureCollection->pFeatureInstance[feature2]->doNormalization = true;
        gbls.pFeatureCollection->pFeatureInstance[feature2]->isEnabled = true;
    }
    model_idx = idxViaFindOrAssignModelID(gbls.pModelCollection, ModelNumber);
    if (model_idx >= 0) {
      gbls.pModelCollection->training_rate = ComputeInterval;
      gbls.pModelCollection->pModelInstance[model_idx]->model_type = (model_t) ModelType;
      gbls.pModelCollection->pModelInstance[model_idx]->modelID = ModelNumber;
      gbls.pModelCollection->pModelInstance[model_idx]->isEnabled = false;
      gbls.pModelCollection->pModelInstance[model_idx]->isTrained = false;
      gbls.pModelCollection->pModelInstance[model_idx]->pFeature[0] = gbls.pFeatureCollection->pFeatureInstance[feature1];
      gbls.pModelCollection->pModelInstance[model_idx]->feature_dimension = 1;
      gbls.pModelCollection->pModelInstance[model_idx]->pModel->gmm_model.init_maxGaussComponents = MaxGaussians + 1;
      gbls.pModelCollection->pModelInstance[model_idx]->threshold = ((float) ThresholdX100)/100;    // range from GUI is from 1 to 100. Threshold is tipically very small (probability density) for outlier judgement.
      gbls.pModelCollection->pModelInstance[model_idx]->featureBufferSize = BufSize;
//      gbls.pModelCollection->pModelInstance[model_idx]->enableTransmit = false;
      if (DIM) {
        gbls.pModelCollection->pModelInstance[model_idx]->pFeature[1] = gbls.pFeatureCollection->pFeatureInstance[feature2];
        gbls.pModelCollection->pModelInstance[model_idx]->feature_dimension = 2;
      }
    } else {
      gbls.setStatus(&gbls, SOFT_FAULT);
      return false;
    }
    return true;
  } else {
    gbls.setStatus(&gbls, SOFT_FAULT);
    return false;
  }
}


/// @brief      This function is called when SVM configuration is processed.
///
///     The configuration is recieved from GUI and set to each model instance.
///
///     @param  ModelType       Model type
///     @param  ModelNumber     Model ID.
///     @param  DIM             1 or 2 dimension of feature samples.
///     @param  MaxGaussians    The maximum number of Gaussian components in GMM usage.
///     @param  BufSize         The size of moving window in feature buffer.
///     @param  ComputeInterval The rate of training. At every ComputeInterval, training is executed.
///     @param  nuX100          The nu parameter of one class SVM. It controls the bounds of how many SVs and threshold level. 100 times larger values is transmitted to use integer type rather than float and save the packet size.
///     @param  gammaX100       The kernel size parameter assigned by a user. 100 times larger values is transmitted to use integer type rather than float and save the packet size.
///     @param  FeatureCode1    The first dimension of feature type.
///     @param  FeatureCode2    The second dimension of feature type.
///     @param  SensorCode1     The first dimension of sensor type.
///     @param  SensorCode2     The second dimension of sensor type.
///     @param  AxisCode1       The axis of SensorCode1.
///     @param  AxisCode2       The axis of SensorCode2.
bool processOcSvmConfigurationCommand(uint8_t ModelType, uint8_t ModelNumber, uint8_t KernelType, bool DIM,
              uint16_t BufSize, uint16_t ComputeInterval, uint8_t nuX100, uint8_t gammaX100,
              feature_t FeatureCode1, feature_t FeatureCode2, sensor_t SensorCode1, sensor_t SensorCode2,
              axis_t AxisCode1, axis_t AxisCode2){
  uint8_t feature1, feature2;
  int8_t model_idx;
  gbls.controlCommandforML = NO_COMMAND;
  disableAllModels(gbls.pModelCollection);  // TBD: Currently disable all but the current model
  if (get_feature_number(&feature1, gbls.pFeatureCollection, SensorCode1, AxisCode1, FeatureCode1)) {
    gbls.pFeatureCollection->pFeatureInstance[feature1]->doNormalization = true ;
    gbls.pFeatureCollection->pFeatureInstance[feature1]->isEnabled = true;

    if (DIM) {
        if (!get_feature_number(&feature2, gbls.pFeatureCollection, SensorCode2, AxisCode2, FeatureCode2)) return false;
        gbls.pFeatureCollection->pFeatureInstance[feature2]->doNormalization = true;
        gbls.pFeatureCollection->pFeatureInstance[feature2]->isEnabled = true;
    }
    model_idx = idxViaFindOrAssignModelID(gbls.pModelCollection, ModelNumber);
    if (model_idx >= 0) {
      gbls.pModelCollection->training_rate = ComputeInterval;
      gbls.pModelCollection->pModelInstance[model_idx]->model_type = (model_t) ModelType;
      gbls.pModelCollection->pModelInstance[model_idx]->modelID = ModelNumber;
      gbls.pModelCollection->pModelInstance[model_idx]->isEnabled = false;
      gbls.pModelCollection->pModelInstance[model_idx]->isTrained = false;
      gbls.pModelCollection->pModelInstance[model_idx]->pFeature[0] = gbls.pFeatureCollection->pFeatureInstance[feature1];
      gbls.pModelCollection->pModelInstance[model_idx]->feature_dimension = 1;
      gbls.pModelCollection->pModelInstance[model_idx]->pModel->ocsvm_model.nu = (float)nuX100/100;
      gbls.pModelCollection->pModelInstance[model_idx]->pModel->ocsvm_model.KernelType = KernelType;
      gbls.pModelCollection->pModelInstance[model_idx]->pModel->ocsvm_model.KernelSize = sqrt(0.5* 100 / (float) gammaX100);
      gbls.pModelCollection->pModelInstance[model_idx]->threshold = 0.01;    // this will be computed in SVM optimization. rho is the value. threshold is determined by SVM algorithm. But keep this variable because we may want to manually set the threshold in later use.
      gbls.pModelCollection->pModelInstance[model_idx]->featureBufferSize = BufSize;
//      gbls.pModelCollection->pModelInstance[model_idx]->enableTransmit = false;
      if (DIM) {
        gbls.pModelCollection->pModelInstance[model_idx]->pFeature[1] = gbls.pFeatureCollection->pFeatureInstance[feature2];
        gbls.pModelCollection->pModelInstance[model_idx]->feature_dimension = 2;
      }
    } else {
      gbls.setStatus(&gbls, SOFT_FAULT);
      return false;
    }
    return true;
  } else {
    gbls.setStatus(&gbls, SOFT_FAULT);
    return false;
  }
}

bool processEnsembleConfigurationCommand(uint8_t ModelType, uint8_t ModelNumber, uint8_t VotesRequired, uint8_t NumSubModels, uint8_t *SubModelSpecifier)
{
    int i;
//    int8_t model_idx;

    gbls.controlCommandforML = NO_COMMAND;
    disableAllModels(gbls.pModelCollection);  // TBD: Currently disable all but the current model
    gbls.pModelCollection->num_LL_models = NumSubModels;
    gbls.pModelCollection->votes_required = VotesRequired;
    for (i=0; i<NumSubModels; i++){
        if (SubModelSpecifier[i] == NULL) return false;
        else  gbls.pModelCollection->model_IDs_ensemble[i] = SubModelSpecifier[i];
    }
    return true;
}

bool setclr_features(sensor_t sensor, axis_t axis, byte *hostCommand, int idx)
{
    bool b1, b2, b3, b4, b5, b6, b7;
    uint16_t bitfield = (hostCommand[idx+1]<<8) + hostCommand[idx];
    b1=setclr_feature(sensor, MEAN,           axis, CI_MEAN           & bitfield);
    b2=setclr_feature(sensor, VARIANCE,       axis, CI_VARIANCE       & bitfield);
    b3=setclr_feature(sensor, SKEW_FACTOR,    axis, CI_SKEW_FACTOR    & bitfield);
    b4=setclr_feature(sensor, KURTOSIS,       axis, CI_KURTOSIS       & bitfield);
    // setclr_feature(sensor, FFT,            axis, CI_FFT            & bitfield);
    b5=setclr_feature(sensor, CROSSING_RATE,  axis, CI_CROSSING_RATE  & bitfield);
    b6=setclr_feature(sensor, STD,            axis, CI_STD            & bitfield);
    b7=setclr_feature(sensor, NORMALIZED_STD, axis, CI_NORMALIZED_STD & bitfield);
    return(b1&&b2&&b3&&b4&&b5&&b6&&b7);
}

feature_t featureEnumFromBitfield(uint16_t bitfield) {
    if (CI_MEAN           & bitfield) return MEAN;
    if (CI_VARIANCE       & bitfield) return VARIANCE;
    if (CI_SKEW_FACTOR    & bitfield) return SKEW_FACTOR;
    if (CI_KURTOSIS       & bitfield) return KURTOSIS;
    if (CI_CROSSING_RATE  & bitfield) return CROSSING_RATE;
    if (CI_STD            & bitfield) return STD;
    if (CI_NORMALIZED_STD & bitfield) return NORMALIZED_STD;
    return END_OF_FEATURES;
}



/* Handler for Device Info and Streaming Control Commands. */
bool process_host_command(
    uint8_t tag, uint8_t *hostCommand, uint8_t *hostResponse, size_t *hostMsgSize, size_t respBufferSize)
{
    bool success = true;
    bool DIM;
    bool StreamEnable;

    int idx;
    byte command=0x00;
    sensor_t SensorCode1, SensorCode2=ACCEL;
    axis_t  AxisCode1, AxisCode2=CHX;
    feature_t FeatureCode1, FeatureCode2=MEAN;
    uint8_t  tmpUInt8;
    uint8_t nuX100, gammaX100, ThresholdX100;
    uint8_t KernelType;
    uint8_t MaxGaussians;
    uint16_t BufSize, ComputeInterval;
    uint32_t tmpUInt32Array[HOST_CMD_RSP_LEN/4];
    uint32_t featureInterval;
    uint8_t NumSubModels;
    uint8_t ModelType, ModelNumber;
    uint8_t SubModelSpecifier[MAX_SUBMODELS];
    uint8_t VotesRequired;
    size_t sz;
    sz = 0;

    size_t appNameLen = sizeof(APPLICATION_NAME) - 1;
    size_t boardNameLen = sizeof(BOARD_NAME) - 1;
    size_t shieldNameLen = sizeof(SHIELD_NAME) - 1;
    gbls.setStatus(&gbls, RECEIVING);
    switch (tag) {
    case HOST_PRO_INT_DEV_TAG: /* IMMEDIATE COMMAND: Host is requesting Device Info, send Board Name and Shield Name. */

        if (respBufferSize >= boardNameLen + shieldNameLen + appNameLen + 3)
        { /* We have sufficient buffer. */
            *hostMsgSize = 0;
        }
        else
        {
            return false;
        }

        // frameTag and hostProtocolVersion will be added by calling function

        hostResponse[sz] = appNameLen;
        sz += 1;

        memcpy(hostResponse + sz, APPLICATION_NAME, appNameLen);
        sz += appNameLen;

        hostResponse[sz] = boardNameLen;
        sz += 1;

        memcpy(hostResponse + sz, BOARD_NAME, boardNameLen);
        sz += boardNameLen;

        hostResponse[sz] = shieldNameLen;
        sz += 1;

        memcpy(hostResponse + sz, SHIELD_NAME, shieldNameLen);
        sz += shieldNameLen;
        break;
    case 0x25: // IMMEDIATE COMMAND: Set feature enable/disable
        //tmpUInt16 = (hostCommand[1]<<8) + hostCommand[2];
        setclr_features(ACCEL, CHX,  hostCommand, 0);
        setclr_features(ACCEL, CHY,  hostCommand, 2);
        setclr_features(ACCEL, CHZ,  hostCommand, 4);
        setclr_features(ACCEL, VM,   hostCommand, 6);
        setclr_features(GYRO,  CHX,  hostCommand, 8);
        setclr_features(GYRO,  CHY,  hostCommand, 10);
        setclr_features(GYRO,  CHZ,  hostCommand, 12);
        setclr_features(GYRO,  VM,   hostCommand, 14);
        setclr_features(TEMPERATURE, SCALAR, hostCommand, 16);
        setclr_features(PRESSURE,    SCALAR, hostCommand, 18);
        setclr_features(MICROPHONE,  SCALAR, hostCommand, 20);
        featureInterval = (hostCommand[25]<<24) + (hostCommand[24]<<16) + (hostCommand[23]<<8) + hostCommand[22];
        setFeatureInterval(featureInterval); // currently only a stub function
        sz = 0;
        break;
    case 0x28: // IMMEDIATE COMMAND: Set GMM Configuration Command
        idx=0;
        ModelType = LOWER_NIBBLE(hostCommand[idx++]);
        ModelNumber = hostCommand[idx++]&0x3F;
        tmpUInt8 = hostCommand[idx++];
        DIM = DECODE_DIM(tmpUInt8);
        MaxGaussians = LOWER_NIBBLE(tmpUInt8);
        BufSize = SHORT(hostCommand[idx+1], hostCommand[idx]);
        idx = idx+2;
        ComputeInterval = SHORT(hostCommand[idx+1], hostCommand[idx]);
        idx = idx+2;
        ThresholdX100 = hostCommand[idx++];
        FeatureCode1 = (feature_t) hostCommand[idx++];
        tmpUInt8 = hostCommand[idx++];
        SensorCode1 = (sensor_t) UPPER_NIBBLE(tmpUInt8);
        AxisCode1 = (axis_t) LOWER_NIBBLE(tmpUInt8);
        if (DIM) {
          // It is a two feature model.
          // These three variables are defaulted to zero above, and should be
          // ignored when DIM==0;
          FeatureCode2 = (feature_t) hostCommand[idx++];
          tmpUInt8 = hostCommand[idx++];
          //SensorCode2 = tmpUInt8>>4;
          SensorCode2 = (sensor_t) UPPER_NIBBLE(tmpUInt8);
          AxisCode2 = (axis_t) LOWER_NIBBLE(tmpUInt8);
        }
        processGmmConfigurationCommand(ModelType, ModelNumber, DIM, MaxGaussians,
              BufSize, ComputeInterval, ThresholdX100, FeatureCode1,
              FeatureCode2, SensorCode1, SensorCode2, AxisCode1, AxisCode2);
        break;
    case 0x29: // IMMEDIATE COMMAND: Set SVM Configuration Command
        idx = 0;
        KernelType = UPPER_NIBBLE(hostCommand[idx]);
        ModelType = LOWER_NIBBLE(hostCommand[idx++]);
        ModelNumber = hostCommand[idx++]&0x3F;
        tmpUInt8 = hostCommand[idx++];
        DIM = DECODE_DIM_SVM(tmpUInt8);
        BufSize = SHORT(hostCommand[idx+1], hostCommand[idx]);
        idx = idx + 2;
        ComputeInterval = SHORT(hostCommand[idx+1], hostCommand[idx]);
        idx = idx+2;
        nuX100 = hostCommand[idx++];
        gammaX100 = hostCommand[idx++];
        FeatureCode1 = (feature_t) hostCommand[idx++];
        tmpUInt8 = hostCommand[idx++];
        SensorCode1 = (sensor_t) UPPER_NIBBLE(tmpUInt8);
        AxisCode1 = (axis_t) LOWER_NIBBLE(tmpUInt8);
        if (DIM) {
          // It is a two feature model.
          // These three variables are defaulted to zero above, and should be
          // ignored when DIM==0;
          FeatureCode2 = (feature_t) hostCommand[idx++];
          tmpUInt8 = hostCommand[idx++];
          //SensorCode2 = tmpUInt8>>4;
          SensorCode2 = (sensor_t) UPPER_NIBBLE(tmpUInt8);
          AxisCode2 = (axis_t) LOWER_NIBBLE(tmpUInt8);
        }
        processOcSvmConfigurationCommand(ModelType, ModelNumber, KernelType, DIM,
                                         BufSize, ComputeInterval, nuX100, gammaX100,
                                         FeatureCode1, FeatureCode2, SensorCode1, SensorCode2,
                                         AxisCode1, AxisCode2);
      break;

    case 0x2A: // MIXED DELAYED/IMMEDIATE COMMAND: Set Model Mode Command
        idx=0;
        ModelType = hostCommand[idx++]&0x0F;
        ModelNumber = hostCommand[idx++]&0x3F;
        command = hostCommand[idx++];
        StreamEnable = DECODE_STREAM_ENABLE(command);
        if (DECODE_CLR(command))  clearFeatureBuffers(gbls.pModelCollection);                   // trained model parameters are remained.
        processSetModelModeCommand(ModelType, ModelNumber, StreamEnable, command&0x7E);         // to set model modes except for the command "CLR".
        break;
    case 0x2B: // IMMEDIATE COMMAND: Read Sample Rates Command
        getEncodedSampleRates(tmpUInt32Array);
        memcpy(hostResponse, tmpUInt32Array, 20);
        sz = 20;
        break;
    case 0x2C: // IMMEDIATE COMMAND: Set Ensemble Configuration Command
        idx=0;
        NumSubModels = UPPER_NIBBLE(hostCommand[idx]);
        ModelType = LOWER_NIBBLE(hostCommand[idx++]);
        ModelNumber = hostCommand[idx++]&0x3F;
        VotesRequired = hostCommand[idx++]&0x0F;
        for (int i=0; i<NumSubModels; i++) {
            SubModelSpecifier[i] = hostCommand[idx++];
        }
        processEnsembleConfigurationCommand(ModelType, ModelNumber, VotesRequired, NumSubModels, &SubModelSpecifier[0]);
      break;
    default:
        sz = 0;
        success = false;
    }
    *hostMsgSize = sz;
    return success;
}

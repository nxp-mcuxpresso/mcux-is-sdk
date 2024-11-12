/*
* Copyright (c) 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/**
 * @file output_stream.c
 * @brief The output_stream.c file implements the streaming packets functionalities for ADT.
 *        Real-time features, anomaly detection results, trained models such GMM and OC-SVM information are transtmitted.
 */

/* FreeRTOS kernel includes. */
#include "stdlib.h"
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
extern Globals gbls;                ///< This is the primary sensor fusion data structure

#include "output_stream.h"

#define PASSFAIL 0x80
#define TRAINRUN 0x40

/// @brief This function is called when stream packets are transmitted to GUI with the current feature samples and detection restuls.
///
/// Details about the packet structure is available in ADT manual.
void send_model_result(bool passFail, bool trainRun, uint8_t modelNumber, uint16_t featureInterval, float feature1, float feature2){
      uint8_t buffer[16];
      buffer[0] = 0x40; // Frame Tag
      buffer[1]=  0x00; // Length MSB - Payload length is big endian.  Everything else is little endian
      buffer[2] = 0x0B; // Length LSB
      buffer[3] = (byte) modelNumber;
      if (passFail) buffer[3] = buffer[3] | PASSFAIL;
      if (trainRun) buffer[3] = buffer[3] | TRAINRUN;
      buffer[4] = (uint8_t) featureInterval & 0x00FF; // LSB
      buffer[5] = featureInterval >> 8;               // MSB
      
      if ((modelNumber > 0x10) && (modelNumber < 0x20))         // for ensemble model, model IDs are in the range from 0x11 to 0x1F.
          Host_IO_Send(buffer, 6, HOST_FORMAT_HDLC);            // send the ENSEMBLE model result packet - 6 bytes
      else {
          memcpy(buffer + 6, &feature1, 4);
          memcpy(buffer + 10, &feature2, 4);
          Host_IO_Send(buffer, 14, HOST_FORMAT_HDLC);           // if model ID is for lower level models, send this packet. - 6 bytes plus 8 bytes (features 1 & 2).
      }
}


/// @brief This function is called when stream packets are transmitted to GUI with the current GMM information.
///
/// Details about the packet structure is available in ADT manual.
void send_computed_gmm_model(ModelInstance *modelInstance, uint16_t featureInterval){
      GmmModel *model;
      model = &(modelInstance->pModel->gmm_model);
      uint8_t buffer[256];
      uint16_t idx=0;
      uint8_t feature_dimension = modelInstance->feature_dimension;
      GaussComponent *gc;

      buffer[idx++] = 0x41; // Frame Tag
      idx+=2; // ength is computed on the fly and will be inserted later
      buffer[idx++] = modelInstance->modelID; // Model Number
      buffer[idx++] = (uint8_t) featureInterval & 0x00FF;  // Feature Interval LSB
      buffer[idx++] = featureInterval >> 8;  // Feature Interval MSB
      if (feature_dimension == 2) buffer[idx++] = 0x01; else buffer[idx++] = 0x00;  // Control byte

      // Send the mean and stdev used to normalize features
      memcpy(buffer + idx, &(modelInstance->pFeature[0]->fBufferMean), 4); idx += 4;
      if (feature_dimension == 2) memcpy(buffer + idx, &(modelInstance->pFeature[1]->fBufferMean), 4); idx += 4;
      memcpy(buffer + idx, &(modelInstance->pFeature[0]->fBufferStd), 4); idx += 4;
      if (feature_dimension == 2) memcpy(buffer + idx, &(modelInstance->pFeature[1]->fBufferStd), 4); idx += 4;


      buffer[idx++] = model->opt_num_components; // Number of gaussian components in this model
      for (int i=0; i<model->opt_num_components; i++) {  // Let's interate through the gaussian components
          gc = &(model->gComponents[i]); // get the pointer to this gaussian
          memcpy(buffer + idx, &(gc->muMeans[0]), 4); idx+=4; // stream the 1st mean
          if (feature_dimension == 2) {
              memcpy(buffer + idx, &(gc->muMeans[1]), 4); idx+=4;  // stream the 2nd mean when required
          }
          memcpy(buffer + idx, &(gc->rCovariance[0][0]), 4); idx+=4; // stream the 1st covariance value
          if (feature_dimension == 2) {
              // stream additional covariances for models with 2 input featues
              memcpy(buffer + idx, &(gc->rCovariance[0][1]), 4); idx+=4;
              memcpy(buffer + idx, &(gc->rCovariance[1][0]), 4); idx+=4;
              memcpy(buffer + idx, &(gc->rCovariance[1][1]), 4); idx+=4;
          }
          memcpy(buffer + idx, &(gc->pi), 4); idx+=4;
     }
     buffer[1] = (idx-3)>>8;
     buffer[2] = (idx-3)&0xFF;  // this is the packet payload length
     Host_IO_Send(buffer, idx, HOST_FORMAT_HDLC);
}

/// @brief This function is called when stream packets are transmitted to GUI with the current OC-SVM information.
///
/// Details about the packet structure is available in ADT manual.
void send_computed_svm_model(ModelInstance *modelInstance, uint16_t featureInterval){
      OcsvmModel *model;
      model = &(modelInstance->pModel->ocsvm_model);
      uint8_t feature_dimension = modelInstance->feature_dimension;
      uint8_t *buffer;
      int SizeofSVs = model->opt_num_SVs * (feature_dimension + 1); // +1 is for coefficent for each SV
      buffer = Malloc(uint8_t, SizeofSVs + 28);
      uint16_t idx=0;
      struct svm_node_embedded *sv;
      float rho_float;
      float sv_coef;
      float sv_float;           // for type conversion from double to float


      buffer[idx++] = 0x42; // Frame Tag
      idx+=2; // Length is computed on the fly and will be inserted later
      buffer[idx++] = modelInstance->modelID; // Model Number
      buffer[idx++] = (uint8_t) featureInterval & 0x00FF;  // Feature Interval LSB
      buffer[idx++] = featureInterval >> 8;  // Feature Interval MSB
      if (feature_dimension == 2) buffer[idx++] = 0x01; else buffer[idx++] = 0x00;  // Control byte

      // Send the mean and stdev used to normalize features
      memcpy(buffer + idx, &(modelInstance->pFeature[0]->fBufferMean), 4); idx += 4;
      if (feature_dimension == 2) memcpy(buffer + idx, &(modelInstance->pFeature[1]->fBufferMean), 4); idx += 4;
      memcpy(buffer + idx, &(modelInstance->pFeature[0]->fBufferStd), 4); idx += 4;
      if (feature_dimension == 2) memcpy(buffer + idx, &(modelInstance->pFeature[1]->fBufferStd), 4); idx += 4;

      rho_float = (float) (model->rho);
      memcpy(buffer + idx, &rho_float, 4); idx += 4;          // float type
      buffer[idx++] = model->opt_num_SVs; // Number of SVs trained
      float nu = model->nu;
      memcpy(buffer + idx, &nu, 4); idx += 4;          // float type
      float kernelSize = model->KernelSize;
      memcpy(buffer + idx, &kernelSize, 4); idx += 4;          // float type

     for (int i=0; i<model->opt_num_SVs; i++) {         // iterate through the SVs
        sv_coef = (float) model->sv_coef[i];
        memcpy(buffer + idx, &sv_coef, 4); idx+=4;      // float type
        sv = &(model->SV[i][0]);                // get the pointer to this SV
        sv_float = (float) sv->value;
        memcpy(buffer + idx, &sv_float, 4); idx+=4;  // double type in svm_node, following LIBSVM code. TBD: change to float type?
        if (feature_dimension == 2) {
            sv = &(model->SV[i][1]);
            sv_float = (float) sv->value;
            memcpy(buffer + idx, &sv_float, 4); idx+=4;  // stream SV of the 2nd dimension
        }
     }

     buffer[1] = (idx-3)>>8;
     buffer[2] = (idx-3)&0xFF;  // this is the packet payload length
     Host_IO_Send(buffer, idx, HOST_FORMAT_HDLC);
     Free(buffer);
}

/// @brief This function is called when stream packets are transmitted to GUI with features and anomaly detection results.
///
/// Details about the packet structure is available in ADT manual.
void streamFeatures()
{
    ModelCollection *models = gbls.pModelCollection;
    ModelInstance *modelInstance;
    float feature1, feature2;
    bool passFail, trainRun, status;
    float likelihood;
    uint8_t modelID;
    uint8_t votes = 0;
  
    // the loop 1) searches Enabled model instances, 2) make decision by runModel(), 3) count the votes, and 3) send the result in a packet identified by modelID. 
    for (int i=0; i<MAX_MODEL_INSTANCES; i++) {
        modelInstance = models->pModelInstance[i];
        if (modelInstance->isEnabled) { 
            feature1 = getLastFeatureValueAdded(modelInstance->pFeature[0]);            // get the latest feature sample
            if (modelInstance->feature_dimension>1) feature2 = getLastFeatureValueAdded(modelInstance->pFeature[1]);    // for two dimensional features
            else feature2=0.0;
            if (modelInstance->isTrained) {                                     // if the model instance is trained,
                if (modelInstance->iStage & RUN_COMMAND)    trainRun = false;  // and only if RUN_COMMAND is enabled, trainRun = 0;
                else  trainRun = true;                                          // otherwise, trainRun = 1
            }
            else   trainRun = true;                                             // if the model was not trained, trainRun = 1.
            if (trainRun)  passFail = true;                                     // passFail = 1 if the model is being trained and even before being trained.
            else {                                                              // trainRun = 0 indicates that the model is trained and ready to go to RUN mode.
                status = runModel(modelInstance, &passFail, &likelihood);       // runModel() makes decisions for the model instance and upcoming features. pass (1) or fail (0).
                if (!status)  gbls.setStatus(&gbls, HARD_FAULT);
            }
            votes += passFail;
            modelID = modelInstance->modelID;           // lower level model id
            send_model_result(passFail, trainRun, modelID, feature_interval_number, feature1, feature2); // send model results.
        }
    }
    
    // if ensemble model is enabled, the modelID should be in the range from 0x11 to 0x1F. Conditioned on the counted votes, it makes a decision and send a packet. 
    if (inEnsembleRange(models->ensemble_modelID) && models->isEnsembleEnabled) {  // if Ensemble, modelID is in the range of 0x11 ~ 0x1F.
      if (gbls.controlCommandforML == RUN_COMMAND)   {
          trainRun = false;                     // if the ensemble model is trained and only if RUN_COMMAND is enabled, trainRun = 0. Otherwise, trainRun = 1;
          if (votes >= models->votes_required)    passFail = true;                // makes a decision - pass (1) or fail (0).
          else passFail = false; 
          modelID = models->ensemble_modelID;             // ensemble model id
          send_model_result(passFail, trainRun, modelID, feature_interval_number, feature1, feature2);     // send ensemble model results. here feature1 and feature2 are nominal.
      }
      else trainRun = true;        
    }
}

/// @brief This function is called when stream packets are transmitted to GUI with model information.
///
/// Details about the packet structure is available in ADT manual.
bool streamModels(struct ModelCollection *modelCollection, uint8_t modelID) 
{
    uint8_t     model_idx;
    model_t     model_type;
    bool        isEnabled;
    struct ModelInstance *modelInstance;
    
    if (!get_model_idx_instance(&model_idx, &isEnabled, modelCollection, modelID)) return false;
    if (isEnabled) {
        modelInstance = modelCollection->pModelInstance[model_idx];
        model_type = modelInstance->model_type;
        switch (model_type){
        case GMM:
            send_computed_gmm_model(modelInstance, feature_interval_number);
            break;
        case OCSVM:
            send_computed_svm_model(modelInstance, feature_interval_number);
            break;
        default:
            break;
        }
        return true;
    }
    else return false;
}

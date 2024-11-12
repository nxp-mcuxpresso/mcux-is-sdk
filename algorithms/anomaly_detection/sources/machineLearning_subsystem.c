/*
* Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/** @file machinelearning_subsystem.c
    @brief The machinelearning_subsystem.c file implements the top level programming interface.

    This contains subroutines of machine learning algorithms.
*/

/* Including needed modules to compile this module/procedure */
#include <stdio.h>
#include <math.h>
#include "anomaly_detection.h"
#include "status.h"
#include "magnetic.h"
#include "drivers.h"
#include "sensor_drv.h"
#include "status.h"
#include "control.h"
#include "fusion.h"
#include "fsl_debug_console.h"
#include "machineLearning_subsystem.h"
#include "gmm_utility.h"
extern Globals gbls;                ///< This is the primary sensor fusion data structure

/** @brief Initializes the FeatureCollection structure.
*
*       The FeatureCollection structure contains pointers of feature instances.
*       This function initializes the address of feature instances created, sensor type, sensor axis, feature type,
*       buffer counter, and indicators such as iBufferExceeded, isEnabled, and isNormalized.
*
*       @param  features        The pointer of FeatureCollection structure.
*       @param  featureInstance The first pointer of FeatureInstance array structure (if multiple feature instances are defined).
*       @return void.
*/
void initFeatureCollection(struct FeatureCollection *featureCollection, struct FeatureInstance *featureInstance)
{
    uint8_t i = 0;              // A variable for the count in for loop.

    for (i=0; i<MAX_FEATURE_INSTANCES; i++){
        /*
        * Feature instance initialization
        * feat_inst is the pointer that indicates the address of first feature instance in the array.
        */
        featureCollection->pFeatureInstance[i] = featureInstance + i;                  // pointers for feature instances
        featureCollection->pFeatureInstance[i]->sensor_type=END_OF_SENSORS;      // Meaningless until isEnabled set to true
        featureCollection->pFeatureInstance[i]->sensor_axis=END_OF_AXIS_TYPES;   // Meaningless until isEnabled set to true
        featureCollection->pFeatureInstance[i]->feature_type=END_OF_FEATURES;    // Meaningless until isEnabled set to true
        featureCollection->pFeatureInstance[i]->iBufferCount = -1;               // The counter of feature buffer.
        featureCollection->pFeatureInstance[i]->iBufferExceeded = false;         // This is set to true if the feature buffer is filled once.
        featureCollection->pFeatureInstance[i]->isEnabled = false;               // Indicator whether this instance is enabled or not.
        featureCollection->pFeatureInstance[i]->isNormalized = false;            // True if doNormalization is processed.
        featureCollection->pFeatureInstance[i]->doNormalization = true;
        featureCollection->pFeatureInstance[i]->fBufferMean = 0.0;
        featureCollection->pFeatureInstance[i]->fBufferStd = 0.0;
    }
}

void nullModelInstance(struct ModelInstance *modelInstance) {
        /* fill model instance with default values */
        modelInstance->feature_dimension = 2;
        modelInstance->modelID = 0;
        modelInstance->isEnabled = false;
        modelInstance->isTrained = false;
        modelInstance->startPositionBuffer = 0;
        modelInstance->endPositionBuffer = 0;
        modelInstance->decision_hard = NULL;
        modelInstance->decision_soft = 0.0;
#if DEBUG_ENABLE_TX_VARIABLE_IS_ON
        modelInstance->enableTransmit = true;
#endif
        for (int j=0; j<MAX_FEATURE_DIMENSION; j++){
            modelInstance->pFeature[j] = NULL;
        }
}

void nullEnsemble(struct ModelCollection *modelCollection) {
        modelCollection->ensemble_modelID = NULL;
        modelCollection->votes_required = NULL;
        modelCollection->isEnsembleEnabled = false;
}


/** @brief Initializes the ModelCollection structure.
*
*       The ModelCollection is initialized with the number of models that are going to process, training rate, and modelInstance structures.
*       The ModelInstance structure contains pointers of feature instances.
*       This function initializes the address of model instances created, feature_dimension, model ID, decision results (soft / hard),
*       and indicators such as isEnabled and isTrained.
*       Each modelInstance also contains startPositionBuffer and endPositionBuffer that are used for a moving window within a feature buffer.
*       That moving window is to pick a given number of latest features from the buffer in real time.
*       The ModelInstance structure also contains the pointers of feature instances and union of model types (GMM / SVM). Each modelInstance takes only one of GMM and SVM models.
*
*       @param  modelCollection        The pointer of ModelCollection structure.
*       @param  modelInstance          The pointer of ModelInstance structure.
*       @param  model                  The pointer of union Model that contains struct GmmModel and struct OcsvmModel.
*       @return void.
*/
void connectModelCollection(struct ModelCollection *modelCollection, struct ModelInstance *modelInstance, union Model *model)
{
    uint8_t i;                               // Variables for the count in for loop
    modelCollection->training_rate = 10;        // Training rate.

    for (i=0; i<MAX_MODEL_INSTANCES; i++){
        /* model instances initialization with defaults */
        modelCollection->pModelInstance[i] = modelInstance + i;
        modelCollection->pModelInstance[i]->pModel = model + i;
    }
}

bool deleteModel(uint8_t ModelNumber) {
    bool isEnabled; // not needed here, but required by get_model_idx_instance
    uint8_t model_idx;
    if (gbls.current_model_id == ModelNumber) gbls.current_model_id = -1;
    struct ModelCollection *modelCollection = gbls.pModelCollection;
    if (inEnsembleRange(ModelNumber))   {
      // Range of ensemble model IDs
      nullEnsemble(modelCollection);
    } else if(get_model_idx_instance(&model_idx, &isEnabled, gbls.pModelCollection, ModelNumber)){
      // we are in normal LL model range
      nullModelInstance(modelCollection->pModelInstance[model_idx]);
    } else {
      gbls.setStatus(&gbls, HARD_FAULT); // never returns
      return false;
    }
    return true;
}

void nullModelCollection(struct ModelCollection *modelCollection)
{
    uint8_t i;                               // Variables for the count in for loop
    modelCollection->training_rate = 10;        // Training rate.

    for (i=0; i<MAX_MODEL_INSTANCES; i++){
        /* model instances initialization with defaults */
        nullModelInstance(modelCollection->pModelInstance[i]);
    }
    nullEnsemble(modelCollection);  // only one Ensemble model is supported
}

void initModelCollection(struct ModelCollection *modelCollection, struct ModelInstance *modelInstance, union Model *model)
{
    connectModelCollection(modelCollection, modelInstance, model);
    nullModelCollection(modelCollection);
}

/** @brief Stops all the active models.
*
*       This function stops the active models by setting isEnabled to false at each model instance.
*
*       @param  modelCollection        The pointer of ModelCollection structure.
*       @return void.
*/
void disableAllModels(struct ModelCollection *models)
{
    uint8_t i = 0;

    for (i=0; i<MAX_MODEL_INSTANCES; i++){
        models->pModelInstance[i]->isEnabled = false; // disable LL models
    }
    models->isEnsembleEnabled = false; // disable Ensemble
}

bool inEnsembleRange(int ModelNumber) {
    return ((ModelNumber > MAX_LLMODEL_NUMBER) && (ModelNumber <= MAX_ENSEMBLE_MODEL_NUMBER));
}

bool inLLModelRange(int ModelNumber) {
    return ((ModelNumber > 0) && (ModelNumber <= MAX_LLMODEL_NUMBER));
}

int8_t idxViaFindOrAssignModelID(struct ModelCollection *models, uint8_t modelID)
{
    // Look for it first
    for (int i=0; i<MAX_MODEL_INSTANCES; i++){
        if (models->pModelInstance[i]->modelID == modelID) return i;
    }
    for (int i=0; i<MAX_MODEL_INSTANCES; i++){
        if (models->pModelInstance[i]->modelID == 0) {
            models->pModelInstance[i]->modelID = modelID;
            return(i);
        }
    }
    return(-1);  // returns -1 if we don't find or assign the modelID
}

/** @brief Adds the real-time features to the buffer in FeatureInstance.
*
*       This function is to add the current features computed in real time into the buffer of FeatureInstance.
*       It first finds which feature instance is enabled, and then calls addToFeatBuffer() function
*       in order to put the values into the enabled feature instance.
*
*       @param  gbls        The pointer of Globals structure.
*       @return void.
*/
void addToFeatureBuffers(struct Globals *gbls)
{
    struct FeatureInstance *featureInstance;
    struct FeatureCollection *featureCollection = gbls->pFeatureCollection;

    for (int i=0; i<MAX_FEATURE_INSTANCES; i++) {
        featureInstance = featureCollection->pFeatureInstance[i];
        if (featureInstance != NULL) {
            if (featureInstance->isEnabled) {
                addToFeatBuffer(gbls, featureInstance);
            }
        } else {
            gbls->setStatus(gbls, HARD_FAULT); // never returns
        }
    }
}

/** @brief Adds a feature sample to the fBuffer.
*
*       This function adds a feature sample to the feature buffer in feature instance. The Globals structure contains four different types sensors
*       where each feature instance takes feature samples from only one of the sensors.
*       Definitions - F_USING_ACCEL, F_USING_MAG, F_USING_GYRO, and F_1DOF_P_BASIC separate the sensor types.
*
*       @param  gbls            The pointer of Globals structure.
*       @param  featureInstance The pointer of FeatureInstance structure.
*       @return void
*/
void addToFeatBuffer(struct Globals *gbls, struct FeatureInstance *featureInstance)
{
    int16_t i = featureInstance->iBufferCount;

    if ((i<0)||(i>=MAX_FEATURE_SAMPLES))  gbls->setStatus(gbls, HARD_FAULT); // never returns
    switch(featureInstance->sensor_type){
#if F_USING_ACCEL
        case ACCEL:     // features computed from accelerometer
            featureInstance->fBuffer[i] = gbls->Accel.features[featureInstance->feature_type][featureInstance->sensor_axis];
            break;
#endif
 #if F_USING_MAG
        case MAG:       // features computed from magnetometer
            featureInstance->fBuffer[i] = gbls->Mag.features[featureInstance->feature_type][featureInstance->sensor_axis];
            break;
#endif
#if F_USING_GYRO
        case GYRO:      // features computed from gyro sensor
            featureInstance->fBuffer[i] = gbls->Gyro.features[featureInstance->feature_type][featureInstance->sensor_axis];
            break;
#endif
#if F_1DOF_P_BASIC
        case PRESSSURE: // features computed from pressure sensor
            featureInstance->fBuffer[i] = gbls->Pressure.features[featureInstance->feature_type];
            break;
#endif
        default:
            break;
    }
    /* If the feature instance already normalized fBuffer, and if the current mode is RUN, then calls zscoreFeatBuffer(). */
    if ((featureInstance->isNormalized) && (gbls->controlCommandforML & RUN_COMMAND))
        zscoreFeatBuffer(featureInstance);      // Given std and mean of the feature buffer, it computes zscore.
}

/** @brief Computes zscore (standardized by the given std and mean).
*
*       This function normalizes the feature sample in fBuffer and save it in fBufferNormalized. iBufferCount is the current index of the feature.
*
*       @param  featureInstance The pointer of FeatureInstance structure.
*       @return void
*/
void zscoreFeatBuffer(struct FeatureInstance *featureInstance)
{
    uint16_t i = featureInstance->iBufferCount;         // the current index within the buffer
    /* computes zscore by std and mean of the buffer. When RUN mode, the std and mean are fixed, whereas they are repeatedly updated in TRINING mode. */
    computeZscore(&featureInstance->fBufferNormalized[i], featureInstance->fBuffer[i], featureInstance->fBufferMean, featureInstance->fBufferStd);
}

/** @brief Computes zscore.
*
*       This function computes zscore instantly.
*
*       @param  xn      The pointer for the output of this function.
*       @param  x       The feature to be standardized.
*       @param  mu      Mean.
*       @param  sigma   Standard deviation.
*       @return void.
*/
void computeZscore(float *xn, float x, float mu, float sigma)
{
    *xn = (x - mu) / sigma;
}

/** @brief increases the featuer buffer indices.
*
*       This function increases the feature buffer indices, if the feature instance is enabled.
*       When the index exceeds MAX_FEATURE_SAMPLES which equivalently defines the maximum size of buffer,
*       the index is reset and creases again. iBufferExceeded indicates whether this occured or not.
*
*       @param  gbls            The pointer of Globals structure.
*       @return void
*/
void incrementFeatureBufferIndices(struct Globals *gbls)
{
    struct FeatureInstance *featureInstance;
    struct FeatureCollection *featureCollection = gbls->pFeatureCollection;
    int idx;

    for (int i=0; i<MAX_FEATURE_INSTANCES; i++) {
        featureInstance = featureCollection->pFeatureInstance[i];
        if (featureInstance->isEnabled) {
            idx = featureInstance->iBufferCount++;
            if (idx >= MAX_FEATURE_SAMPLES - 1){
                featureInstance->iBufferCount = 0;
                featureInstance->iBufferExceeded = true;
            }
        }
    }
}

/** @brief Increment Feature Buffer Indices of Models
*
*       This function increase the indices of feature buffers, if the model instance is enabled.
*
*       @param  gbls    The pointer of Globals structure.
*       @return void.
*/
void incrementFeatureBufferIndicesModels(struct Globals *gbls)
{
    struct ModelInstance *modelInstance = gbls->pModelCollection->pModelInstance[0];

    for (int i=0; i<MAX_MODEL_INSTANCES; i++) {
        if (modelInstance[i].isEnabled)
            incrementFeatureBufferIndicesStartEnd(&modelInstance[i]);
    }
}

/** @brief Increases feature buffer indices for a model instance
*
*       This function increases feature buffer indices for the given model instance.
*       It checks whether the indices are within a moving window that is reversed or not.
*
*       @param  modelInstance   The pointer of ModelInstance structure.
*       @return void.
*/
void incrementFeatureBufferIndicesStartEnd(struct ModelInstance *modelInstance)
{
    int startPosition;
    int endPosition;
    bool reversed ;

    if (modelInstance->pFeature[0]->isEnabled) {
        startPosition = modelInstance->startPositionBuffer;
        endPosition = modelInstance->endPositionBuffer;
        reversed = checkReversed(startPosition, endPosition);
        if (!reversed && (endPosition - startPosition < modelInstance->featureBufferSize - 1))
            modelInstance->endPositionBuffer = modelInstance->pFeature[0]->iBufferCount;
        else {
            modelInstance->endPositionBuffer = modelInstance->pFeature[0]->iBufferCount;
            modelInstance->startPositionBuffer = modelInstance->pFeature[0]->iBufferCount - modelInstance->featureBufferSize + 1;
            if (modelInstance->startPositionBuffer < 0)
                modelInstance->startPositionBuffer += MAX_FEATURE_SAMPLES;
        }
    }
}

/** @brief Checks whether a moving window is reversed or not.
*
*       This function is to check whether a moving window is reversed or not.
*       If the window is reversed, then the startPosition will be bigger than endPosition.
*       If not, startPosition will be smaller than endPosition.
*
*       @param  startPosition   The index that indicates the start position of a moving window.
*       @param  endPosition     The index that indicates the end position of a moving window.
*       @return true / false.
*/
bool checkReversed(int startPosition, int endPosition)
{
    if (startPosition > endPosition) return true;
    else return false;
}

/** @brief Finds a window position
*
*       This function finds the start and end positions of a moving window.
*
*       @param  modelInstance   The pointer of ModelInstance structure.
*       @param  startPos        The output that contains the start position of window.
*       @param  endPos          The output that contains the end position of window.
*       @return void.
*/
void inputPositions(struct ModelInstance *modelInstance, int *startPos, int *endPos)
{
    bool reversed = modelInstance->reversedPosition;

    if (!reversed) {
        *startPos = modelInstance->startPositionBuffer;
        *endPos = modelInstance->endPositionBuffer;
    }
    else {
        *startPos = modelInstance->startPositionBuffer;
        *endPos = modelInstance->endPositionBuffer + MAX_FEATURE_SAMPLES;
    }
}

float getLastFeatureValueAdded(FeatureInstance *featureInstance)
{
    int16_t idx = featureInstance->iBufferCount;

    if ((idx<0)||(idx>=MAX_FEATURE_SAMPLES))  gbls.setStatus(&gbls, HARD_FAULT); // never returns
    return(featureInstance->fBuffer[idx]);
}

/** @brief Normalizes the features within a moving window.
*
*       This function is to normalize the features within a moving window whose start and end positions
*       are maintained in ModelInstance structure. The function contains a subroutine for checking whether those positions are reversed or not.
*       normalizedFeatWindow() is referred to.
*
*       @param  modelInstance   The pointer of ModelInstance structure.
*       @return void.
*/
void normalizeFeatures (struct ModelInstance *modelInstance)
{
    struct FeatureInstance *featureInstance;
    featureInstance = modelInstance->pFeature[0];
    int sPos, ePos;
    bool reversed;

    sPos = modelInstance->startPositionBuffer;
    ePos = modelInstance->endPositionBuffer;
    modelInstance->reversedPosition = checkReversed(sPos, ePos);
    inputPositions(modelInstance, &sPos, &ePos);
    reversed = modelInstance->reversedPosition;
    if (featureInstance->doNormalization) {
        normalizeFeatWindow(featureInstance, sPos, ePos, reversed);
        if (modelInstance->feature_dimension == 2){
            featureInstance = modelInstance->pFeature[1];
            normalizeFeatWindow(featureInstance, sPos, ePos, reversed);
        }
    }
}

/** @breif Normalizes features within a moving window.
*
*       This function is to normalize the features within a moving window. The start and end poistions of the window are computed
*       and stored in FeatureInstance structure.
*       isNormalized is set to true at the end of this function.
*
*       @param  featureInstance The pointer of FeatureInstance structure.
*       @param  sPos    The start position of moving window.
*       @param  ePos    The end position of moving window.
*       @param  reversed        The indicator whether the moving window is reversed or not.
*       @return void.
*/
void normalizeFeatWindow (struct FeatureInstance *featureInstance, int sPos, int ePos, bool reversed)
{
      uint16_t i;
      float mean = 0, var = 0;
      uint16_t r;
      int windowSize = ePos - sPos + 1;
      if (featureInstance->iBufferExceeded){
          for (i=sPos; i<ePos; i++) {
              if (!reversed) r = i;
              else r = i % MAX_FEATURE_SAMPLES;
              mean += featureInstance->fBuffer[r];
          }
          mean /= windowSize;
          for (i=sPos; i<ePos; i++) {
              if (!reversed) r = i;
              else r = i % MAX_FEATURE_SAMPLES;
              var += (featureInstance->fBuffer[r] - mean) * (featureInstance->fBuffer[r] - mean);
          }
          var /= (windowSize - 1);            // sample variance to be unbiased
          featureInstance->fBufferMean = mean;
          featureInstance->fBufferStd = sqrt(var);
          for (i=sPos; i<ePos; i++) {
              if (!reversed) r = i;
              else r = i % MAX_FEATURE_SAMPLES;
              featureInstance->fBufferNormalized[r] = (featureInstance->fBuffer[r] - featureInstance->fBufferMean) / featureInstance->fBufferStd;
          }
      }
      else {
          for (i=sPos; i<ePos; i++) {
              mean += featureInstance->fBuffer[i];
          }
          mean /= featureInstance->iBufferCount;
          for (i=sPos; i<ePos; i++) {
              var += (featureInstance->fBuffer[i] - mean) * (featureInstance->fBuffer[i] - mean);
          }
          var /= (featureInstance->iBufferCount - 1);            // sample variance to be unbiased
          featureInstance->fBufferMean = mean;
          featureInstance->fBufferStd = sqrt(var);
          for (i=sPos; i<ePos; i++) {
              featureInstance->fBufferNormalized[i] = (featureInstance->fBuffer[i] - featureInstance->fBufferMean) / featureInstance->fBufferStd;
          }
      }
      featureInstance->isNormalized = true;
}

/** @brief Gets the model instance by model ID
*
*       This function searches model IDs of modelInstance structures and assigns the ID number and isEnabled.
*
*       @param  idx     The output that indicates modelInstance index.
*       @param  isEnabled       The output that indicates whether the modelInstance is enabled or not.
*       @param  modelCollection The pointer of ModelCollection structure
*       @param  modelID Model ID.
*       @return true / false    True if success.
*/
bool get_model_idx_instance(uint8_t *idx, bool *isEnabled, struct ModelCollection *modelCollection, uint8_t modelID)
{
    for (int i=0; i<MAX_MODEL_INSTANCES; i++) {
            if (modelCollection->pModelInstance[i]->modelID == modelID) {
                *idx = i;
                *isEnabled = modelCollection->pModelInstance[i]->isEnabled;
                return(true);
            }
    }
    return(false);
}


/** @brief Gets feature instance number
*
*       This function gets the index of FeatureInstance structure from the coordination of sensor_type, sensor_axis, and feature_type.
*
*       @param  idx     The output that indicates the resulting index of featureInstance.
*       @param  featureCollection       The pointer of FeatureCollection structure.
*       @param  sensor  Sensor type information.
*       @param  axis    Sensor axis information.
*       @param  feature Feature type information.
*       @return true / false.
*/
bool get_feature_number(uint8_t *idx, struct FeatureCollection *featureCollection, sensor_t sensor, axis_t axis, feature_t feature)
{
    // Check to see if the sensor is in the list
    for (int i=0; i< MAX_FEATURE_INSTANCES; i++) {
        if (featureCollection->pFeatureInstance[i]->isEnabled) {
            if ((featureCollection->pFeatureInstance[i]->sensor_type == sensor) &&
                (featureCollection->pFeatureInstance[i]->sensor_axis == axis) &&
                (featureCollection->pFeatureInstance[i]->feature_type == feature)) {
                *idx = i;
                return(true);
            }
        }
    }
    return false;
}

/** @brief Adds features by (sensor_t, axis_t, feature_t).
*
*       This function is to add and enable feature instance with coordination (sensor_t, axis_t, feature_t).
*
*       @param  idx     The output that indicates the resulting index of featureInstance.
*       @param  featureCollection       The pointer of FeatureCollection structure.
*       @param  sensor  Sensor type information.
*       @param  axis    Sensor axis information.
*       @param  feature Feature type information.
*       @return true / false.
*/
bool add_feature(uint8_t *idx, struct FeatureCollection *featureCollection, sensor_t sensor, axis_t axis, feature_t feature)
{
    // First check to see if the sensor is already in the list
    if (get_feature_number(idx, featureCollection, sensor, axis, feature)) return(true);
    // Was not found.  Let's see if we have space to add it.
    for (int i=0; i< MAX_FEATURE_INSTANCES; i++) {
        if (!featureCollection->pFeatureInstance[i]->isEnabled) {
            featureCollection->pFeatureInstance[i]->sensor_type = sensor;
            featureCollection->pFeatureInstance[i]->sensor_axis = axis;
            featureCollection->pFeatureInstance[i]->feature_type = feature;
            featureCollection->pFeatureInstance[i]->isEnabled = true;
            *idx = i;
            return(true);
        }
    }
    gbls.setStatus(&gbls, SOFT_FAULT);
    return(false);  // It doesn't exist and we don't have room to add it
}

bool delete_feature_by_index(struct FeatureCollection *features, uint8_t idx)
{
    if (idx<MAX_FEATURE_INSTANCES) {
        features->pFeatureInstance[idx]->isEnabled = false;
        return true;
    } else {
        gbls.setStatus(&gbls, HARD_FAULT); // never returns
        return false;
    }
}

bool delete_feature_by_attributes(struct FeatureCollection *features, sensor_t sensor, axis_t axis, feature_t feature)
{
    uint8_t idx;
    // First check to see if the sensor is already in the list

    if (get_feature_number(&idx, features, sensor, axis, feature)) {
        delete_feature_by_index(features, idx);
        return(true);
    }
    return(false);  // It doesn't exist
}

bool get_feature_attributes(struct FeatureCollection *features, uint8_t idx, sensor_t *sensor, axis_t *axis, feature_t *feature)
{
    if (idx<MAX_FEATURE_INSTANCES) {
        *sensor = features->pFeatureInstance[idx]->sensor_type;
        *axis   = features->pFeatureInstance[idx]->sensor_axis;
        *feature = features->pFeatureInstance[idx]->feature_type;
        return true;
    } else return false;
}

/** @brief Enbales a model with model ID to run
*
*       This function enables a model that is specified by model ID to run.
*       iStage of modelInstance is set to RUN_COMMAND, if the model instance has been trained.
*
*       @param  modelCollection The pointer of ModelCollection structure.
*       @param  modelID Model ID.
*       @return true / false    True if success.
*/
bool enableRunModel(struct ModelCollection *modelCollection, uint8_t modelID)
{
    uint8_t     model_idx;
    bool        isEnabled;

    if (!get_model_idx_instance(&model_idx, &isEnabled, modelCollection, modelID)) return false;
    else {
        if (modelCollection->pModelInstance[model_idx]->isTrained)
            modelCollection->pModelInstance[model_idx]->iStage = RUN_COMMAND;
    }
    return true;
}

/** @brief Enables model instances.
*
*       This function enables modelInstance with model ID.
*
*       @param  modelCollection The pointer of ModelCollection structure.
*       @param  modelID Model ID.
*       @return true / false    True if success.
*/
bool enableModel(struct ModelCollection *modelCollection, uint8_t modelID)
{
    uint8_t     model_idx;
    bool        isEnabled;

    if (!get_model_idx_instance(&model_idx, &isEnabled, modelCollection, modelID)) return false;
    else  modelCollection->pModelInstance[model_idx]->isEnabled = true;
    return true;
}

/** @brief Execute RUN mode of a model instance.
*
*       This function executes the run mode of a model instance. Depending on the model type (GMM / SVM),
*       given the trained model, detectionAnomalyInGMM(), detectionAnomalyInSVM(), or detectionAnomalyInEnsemble() is called.
*
*       @param  modelInstance   The pointer of ModelInstance structure.
*       @param  passFail        Hard decision restult. Pass: positive (normal). Fail: negative (anomaly).
*       @param  likelihood      Soft decition restult. The output of density function built by GMM or SVM.
*       @return true / false    True if success.
*/
bool runModel(struct ModelInstance *modelInstance, bool *passFail, float *likelihood)
{
    model_t     model_type;

    if (modelInstance->isEnabled) {
        model_type = modelInstance->model_type;
        switch (model_type){
        case GMM:
            if (modelInstance->isTrained){
                detectAnomalyInGMM(modelInstance);
                *likelihood = modelInstance->decision_soft;
                *passFail = modelInstance->decision_hard;
                return true;
            }
            else  return false;
            break;
        case OCSVM:
            if (modelInstance->isTrained){
                detectAnomalyInSVM(modelInstance);
                *likelihood = modelInstance->decision_soft;
                *passFail = modelInstance->decision_hard;
                return true;
            }
            else  return false;
            break;
//        case ENSEMBLE:
//              break;
        default:
            return false;
            break;
        }
    } else return false;
}

/** @brief Anomaly detection function for GMM
*
*       This function is for anomaly detection using GMM. Soft and hard decisions are made.
*
*       @param  modelInstance   The pointer of ModelInstance structure.
*       @return void.
*/
void detectAnomalyInGMM(struct ModelInstance *modelInstance)
{
    float likelihood = 0.0;
    float feat[2], feat_normalized[2];
    uint8_t i;
    uint16_t idx0 = modelInstance->pFeature[0]->iBufferCount;
    struct GmmModel *pGmmModel;

    feat[0] = modelInstance->pFeature[0]->fBuffer[idx0];
    computeZscore(&feat_normalized[0], feat[0], modelInstance->pFeature[0]->fBufferMean, modelInstance->pFeature[0]->fBufferStd);
    if (modelInstance->feature_dimension == 2) {
        uint16_t idx1 = modelInstance->pFeature[1]->iBufferCount;
        feat[1] = modelInstance->pFeature[1]->fBuffer[idx1];
        computeZscore(&feat_normalized[1], feat[1], modelInstance->pFeature[1]->fBufferMean, modelInstance->pFeature[1]->fBufferStd);
    }
    pGmmModel = &modelInstance->pModel->gmm_model;
    for(i=0; i<pGmmModel->opt_num_components; i++) {
        likelihood += pGmmModel->gComponents[i].pi * exp(loglike(&feat_normalized[0],&pGmmModel->gComponents[i],modelInstance->feature_dimension));
    }
    modelInstance->decision_soft = likelihood;
    modelInstance->decision_hard = likelihood > modelInstance->threshold;
}

/** @brief Anomaly detection function for SVM
*
*       This function is for anomaly detection using OC-SVM. Soft and hard decisions are made.
*
*       @param  modelInstance   The pointer of ModelInstance structure.
*       @return void.
*/
void detectAnomalyInSVM(struct ModelInstance *modelInstance)
{
    float likelihood = 0.0;
    float feat[2], feat_normalized[2], kSize;
    float x, y;
    uint8_t i;
    uint16_t idx0 = modelInstance->pFeature[0]->iBufferCount;
    struct OcsvmModel *pOcsvmModel;

    feat[0] = modelInstance->pFeature[0]->fBuffer[idx0];
    computeZscore(&feat_normalized[0], feat[0], modelInstance->pFeature[0]->fBufferMean, modelInstance->pFeature[0]->fBufferStd);
    if (modelInstance->feature_dimension == 2) {
        uint16_t idx1 = modelInstance->pFeature[1]->iBufferCount;
        feat[1] = modelInstance->pFeature[1]->fBuffer[idx1];
        computeZscore(&feat_normalized[1], feat[1], modelInstance->pFeature[1]->fBufferMean, modelInstance->pFeature[1]->fBufferStd);
    }
    pOcsvmModel = &modelInstance->pModel->ocsvm_model;
    if (pOcsvmModel->KernelType == RBF) {
        kSize = pOcsvmModel->KernelSize;
        for(i=0; i<pOcsvmModel->opt_num_SVs; i++) {
            if (modelInstance->feature_dimension == 2) {
                x = feat_normalized[0] - pOcsvmModel->SV[i][0].value;
                y = feat_normalized[1] - pOcsvmModel->SV[i][1].value;
                likelihood += pOcsvmModel->sv_coef[i] * exp(-0.5*(x*x + y*y)/(kSize*kSize));
            }
            else {
                x = feat_normalized[0] - pOcsvmModel->SV[i][0].value;
                likelihood += pOcsvmModel->sv_coef[i] * exp(-0.5*(x*x)/(kSize*kSize));
            }
        }
    }
    else     gbls.setStatus(&gbls, HARD_FAULT);             // kernel is not defined as RBF.
    modelInstance->decision_soft = likelihood;
    modelInstance->threshold = pOcsvmModel->rho;
    modelInstance->decision_hard = likelihood > pOcsvmModel->rho;
}

/** @brief Anomaly detection function for Ensemble Models
*
*       This function is for anomaly detection using ensemble models. Soft and hard decisions are made.
*
*       @param  modelInstance   The pointer of ModelInstance structure.
*       @return void.
*/
void detectAnomalyInEnsemble(struct ModelInstance *modelInstance)
{
}

/** @brief Executes TRAINING of model instances.
*
*       This function is for training a model instance. Each modelInstance contains a model type (GMM or SVM or Ensemble), and calls one of the training functions (trainGMM(), trainOCSVM(), and trainALL()).
*       Before calling those functions, it is assumed to normalize features because normalization can help models to avoid undesired situations (e.g., sigularity of covariance matrix in GMM).
*       Also, normalization scales down arbitrary features down to standardized ones by std and mean.
*
*       @param  modelCollection The pointer of ModelCollection structure.
*       @param  modelID Model ID.
*       @return true / false    True if success.
*/
bool trainModel(struct ModelCollection *modelCollection, uint8_t modelID)
{
    uint8_t     model_idx;
    model_t     model_type;
    bool        isEnabled;
    struct ModelInstance *modelInstance;

    if (!get_model_idx_instance(&model_idx, &isEnabled, modelCollection, modelID)) return false;
    if (isEnabled) {
        modelInstance = modelCollection->pModelInstance[model_idx];
        model_type = modelInstance->model_type;
        modelInstance->iStage = TRAIN_COMMAND;
//        modelInstance->enableTransmit = true;
        switch (model_type){
        case GMM:
            if (modelInstance->isEnabled){
                normalizeFeatures(modelInstance);
                trainGMM(modelCollection, modelID);
            }
            else {
                return false;
            }
            break;
        case OCSVM:
            if (modelInstance->isEnabled){
                normalizeFeatures(modelInstance);
                trainOCSVM(modelCollection, modelID);
            }
            else {
                return false;
            }
            break;
        case ENSEMBLE:
            trainEnsemble(modelCollection);             // not implemented yet.
            break;
        default:
            return false;
            break;
        }
        return true;
    } else return false;
}

void trainGMM_instance(struct ModelInstance *modelInstance)
{
    int sPos, ePos;

    sPos = modelInstance->startPositionBuffer;
    ePos = modelInstance->endPositionBuffer;
    modelInstance->reversedPosition = checkReversed(sPos, ePos);
    if (initGMM(modelInstance))  {
        runGMMEM(modelInstance);
    }
    modelInstance->isTrained = true;
}

/** @brief Executes TRAINING of GMM instance.
*
*       This function is for training a GMM instance. If the model instance is enabled and the features are normalized,
*       then Expectation-Maximization (EM) algorithm runs. initGMM() is called to control GMM parameters before EM is processed. runGMMEM() executes the EM algorithm.
*       Before calling those functions, it is assumed to check whether the moving window of feature buffer is reversed or not. Once TRAINING is done, isTrained is set to true.
*
*       @param  modelCollection The pointer of ModelCollection structure.
*       @param  modelID Model ID.
*       @return true / false    True if success.
*/
void trainGMM(struct ModelCollection *modelCollection, uint8_t modelID)
{
    uint8_t  model_idx;
    bool isEnabled;
    struct ModelInstance *modelInstance;

    if (!get_model_idx_instance(&model_idx, &isEnabled, modelCollection, modelID)) {
        gbls.setStatus(&gbls, HARD_FAULT); // never returns
    }
    else {
        modelInstance = modelCollection->pModelInstance[model_idx];
        if (isEnabled&&(modelInstance->pFeature[0]->isNormalized)) {
            trainGMM_instance(modelInstance);
        }
    }
}

float sumBuffer(float *pBuffer, int startPos, int endPos, bool reversed)
{
    int j, r;
    float sum = 0.0;

    for (j=startPos; j<=endPos; j++){
        if (!reversed)  r = j;
        else  r = j % MAX_FEATURE_SAMPLES;
        sum += pBuffer[r];
    }
    return sum;
}

float sumSquaredBuffer(float *pBuffer1, float *pBuffer2, int startPos, int endPos, bool reversed)
{
    int j, r;
    float sum = 0.0;

    for (j=startPos; j<=endPos; j++){
        if (!reversed)  r = j;
        else  r = j % MAX_FEATURE_SAMPLES;
        sum += (pBuffer1[r] * pBuffer2[r]);
    }
    return sum;
}

bool computeR(struct ModelInstance *modelInstance, float R[][2])
{
    int16_t i,k;
    float *pBuffer1, *pBuffer2;
    float mean[MAX_FEATURE_DIMENSION];
    int startPos, endPos;
    bool reversed = modelInstance->reversedPosition;
    float summedWeights = modelInstance->featureBufferSize;
    float Rmin;

    inputPositions(modelInstance, &startPos, &endPos);
    if (modelInstance->feature_dimension < 1) return false;
    else {
        /* initialize the mean vector */
        for (i=0; i<modelInstance->feature_dimension; i++) {
            if (modelInstance->pFeature[i]->isNormalized)
                pBuffer1 = &modelInstance->pFeature[i]->fBufferNormalized[0];
            else
                pBuffer1 = &modelInstance->pFeature[i]->fBuffer[0];
            mean[i] = sumBuffer(pBuffer1, startPos, endPos, reversed);
            mean[i] /= summedWeights;
        }
        /* compute initial covariance matrix R */
        for (k=0; k<modelInstance->feature_dimension; k++){
            for (i=0; i<modelInstance->feature_dimension; i++){
                if (modelInstance->pFeature[i]->isNormalized) {
                    pBuffer1 = &modelInstance->pFeature[i]->fBufferNormalized[0];
                    pBuffer2 = &modelInstance->pFeature[k]->fBufferNormalized[0];
                }
                else {
                    pBuffer1 = &modelInstance->pFeature[i]->fBuffer[0];
                    pBuffer2 = &modelInstance->pFeature[k]->fBuffer[0];
                }
                R[i][k] = sumSquaredBuffer(pBuffer1, pBuffer2, startPos, endPos, reversed);
                R[i][k] /= summedWeights;
                R[i][k] -= mean[i]*mean[k];
                if (R[i][k]*R[i][k] < TINY) return false;
            }
        }
        Rmin = 0;
        for (i=0; i<modelInstance->feature_dimension; i++){
            Rmin += R[i][i];
        }
        Rmin = Rmin / (modelInstance->feature_dimension);
        Rmin = Rmin / DYNAMIC_RANGE;          // regularization term to avoid singularity of covariance matrix.
        for (i=0; i<modelInstance->feature_dimension; i++){
            R[i][i] += Rmin;
        }
        return true;
    }
}
/** @brief Initializes GMM parameters.
*
*       This function initializes GMM parameters. At every training time, this function is called to refresh the GMM parameters. If opt_num_components is less than init_maxGaussComponents,
*       GMM increases its own components and then refresh the parameters. This way helps to smoothly adapt GMM over time.
*
*       @param  modelInstance The pointer of ModelInstance structure.
*       @return true / false    True if success.
*/
bool initGMM(struct ModelInstance *modelInstance)
{
    struct GmmModel *gmmModel = &modelInstance->pModel->gmm_model;
    struct GaussComponent *gComponent;
    int16_t i,j,k;
    float R[MAX_FEATURE_DIMENSION][MAX_FEATURE_DIMENSION];
    bool initialized = modelInstance->pModel->gmm_model.initialized;
    uint16_t count;

    if (initialized && (gmmModel->opt_num_components < gmmModel->init_maxGaussComponents - 1) ) {
        gmmModel->nComponents = gmmModel->opt_num_components + 2;
    }
    else   gmmModel->nComponents = gmmModel->init_maxGaussComponents;       // # of Gaussian components

    if (!computeR(modelInstance, R))
        return false;

    for (i=0; i<gmmModel->nComponents; i++){
        gComponent = &gmmModel->gComponents[i];
        for (j=0; j<modelInstance->feature_dimension; j++){
            count = modelInstance->pFeature[j]->iBufferCount + 1;
            if (!modelInstance->isTrained) {
                if (modelInstance->pFeature[j]->isNormalized)
                    gComponent->muMeans[j] = modelInstance->pFeature[j]->fBufferNormalized[(int)(rand()%count)];
                else
                    gComponent->muMeans[j] = modelInstance->pFeature[j]->fBuffer[(int)(rand()%count)];
            }
            else
                gComponent->muMeans[j] = gComponent->muMeans[j];
	}
	for (j=0; j<modelInstance->feature_dimension; j++){
            for (k=0; k<modelInstance->feature_dimension; k++){
                gComponent->rCovariance[j][k] = R[j][k];
            }
	}
        gComponent->pi = 1.0/(gmmModel->nComponents);
    }
    gmmModel->opt_num_components = 1;
    // normalization
    normalizeMixProb(gmmModel);
    // inverse matrix and compute cnst
    ComputeCnst(&gmmModel->gComponents[0], modelInstance->feature_dimension, gmmModel->nComponents);
     //////////////////// end of computing initial values of mean, R ///////////////
    modelInstance->pModel->gmm_model.initialized = true;
    return true;
}

/** @brief Executes expectation-maximization (EM) algorithm to estimate GMM parameters.
*
*       This function executes EM algorithm to estimate GMM parameters. With an initial set of parameters,
*       it first computes minimum description length (MDL) criterion, which is used to find the optimal number of Gaussian components, by computeMDL_GMM().
*       Reducing the number of mixture components upto one, it evaluates MDL for different number of mixture components.
*       When it reduces the number of components, reduceModelOrder() sums two adjacent componets and discard one of the two.
*       The adjacency is measured by mean vectors of every given pairs of components.
*       For every case of mixture components, EM algoritm is processed by calling reestimate() and computeLL_regroup() in computeMDL_GMM().
*
*       @param  modelInstance The pointer of ModelInstance structure.
*       @return void.
*/
void runGMMEM(struct ModelInstance *modelInstance)
{
    float min_mdl;
    float mdl;		// variable to measure how the estimated GMM is close to the actual data distribution
    uint8_t j;
    struct GaussComponent Param[MAX_GAUSSIAN_COMPONENTS];         // initial maximum number of gaussian components
    struct GmmModel *gmmModel = &modelInstance->pModel->gmm_model;
    bool refresh = false;

    /* GMM-EM algorithm to estimate Gaussian component parameters */
    min_mdl = computeMDL_GMM(modelInstance);
    // reduce model order
    while((gmmModel->nComponents > 1)) {
        /* reduce the number of Gaussian components by combining two similar components (closely located) */
        reduceModelOrder(modelInstance);
        /* GMM-EM algorithm to update Gaussian component parameters */
        mdl = computeMDL_GMM(modelInstance);
        if (mdl < min_mdl) {
             min_mdl = mdl;
             gmmModel->opt_num_components = gmmModel->nComponents ;
             /* update the best parameters */
             for (j=0; j < gmmModel->nComponents; j++ ){
                 copyGaussComp(&gmmModel->gComponents[j], &Param[j], modelInstance->feature_dimension);
             }
         }
    } // end of while loop
    for (j=0; j<gmmModel->opt_num_components; j++){
        copyGaussComp(&Param[j], &gmmModel->gComponents[j], modelInstance->feature_dimension);
    }
    // troubleshooting....
    if (gmmModel->opt_num_components < 1)        refresh = true;
    for (j=0; j<gmmModel->init_maxGaussComponents; j++) {
        if (isnan(gmmModel->gComponents[j].N) || isinf(gmmModel->gComponents[j].cnst)) refresh = true;
    }
    if (!refresh)       modelInstance->enableTransmit = true;
    else {
#if DEBUG_ENABLE_TX_VARIABLE_IS_ON
        modelInstance->enableTransmit = false;
#endif
        refreshGMM(modelInstance);
        refresh = false;
    }
}

/** @brief Refreshes GMM parameters.
*
*       This function refreshes GMM parameters. Somtimes, GMM parameters are ill conditioned. This function is called to refresh the GMM parameters. If opt_num_components is less than init_maxGaussComponents,
*       GMM increases its own components and then refresh the parameters. This way helps to smoothly adapt GMM over time.
*
*       @param  modelInstance The pointer of ModelInstance structure.
*       @return true / false    True if success.
*/
void refreshGMM(struct ModelInstance *modelInstance)
{
    struct GmmModel *gmmModel = &modelInstance->pModel->gmm_model;
    struct GaussComponent *gComponent;
    int16_t i,j,k;
    float R[MAX_FEATURE_DIMENSION][MAX_FEATURE_DIMENSION];
    uint16_t count;

//    if (!computeR(modelInstance, R))
//        return false;
    for (i=0; i<modelInstance->feature_dimension; i++) {
        for (j=0; j<modelInstance->feature_dimension; j++) {
            R[i][j] = 0;
            if (i==j) R[i][j] = 0.5;
        }
    }
    gmmModel->nComponents = gmmModel->init_maxGaussComponents;


    for (i=0; i<gmmModel->nComponents; i++){
        gComponent = &gmmModel->gComponents[i];
        gComponent->N = modelInstance->featureBufferSize / gmmModel->nComponents;
        count = rand()%MAX_FEATURE_SAMPLES ;
        for (j=0; j<modelInstance->feature_dimension; j++){
            if (modelInstance->pFeature[j]->isNormalized)
                    gComponent->muMeans[j] = modelInstance->pFeature[j]->fBufferNormalized[count];
                else
                    gComponent->muMeans[j] = modelInstance->pFeature[j]->fBuffer[count];
	}
	for (j=0; j<modelInstance->feature_dimension; j++){
            for (k=0; k<modelInstance->feature_dimension; k++){
                gComponent->rCovariance[j][k] = R[j][k];
            }
	}
        gComponent->pi = 1.0/(gmmModel->nComponents);
    }

    gmmModel->opt_num_components = gmmModel->nComponents;
    // normalization
    normalizeMixProb(gmmModel);
    // inverse matrix and compute cnst
    ComputeCnst(&gmmModel->gComponents[0], modelInstance->feature_dimension, gmmModel->nComponents);

}

void trainOCSVM_instance(struct ModelInstance *modelInstance)
{
    int sPos, ePos;

    sPos = modelInstance->startPositionBuffer;
    ePos = modelInstance->endPositionBuffer;
    modelInstance->reversedPosition = checkReversed(sPos, ePos);
    trainOCSVM_malloc(modelInstance);
    modelInstance->isTrained = true;
}

/** @brief Executes TRAINING of SVM instance.
*
*       This function is for training a SVM instance. If the model instance is enabled and the features are normalized,
*       then one class support vector machine (OC-SVM) algorithm runs.
*       Before calling this function, it is assumed to check whether the moving window of feature buffer is reversed or not.
*       Once TRAINING is done, isTrained is set to true.
*
*       @param  modelCollection The pointer of ModelCollection structure.
*       @param  modelID Model ID.
*       @return true / false    True if success.
*/
void trainOCSVM(struct ModelCollection *modelCollection, uint8_t modelID)
{
    uint8_t  model_idx;
    bool isEnabled;
    struct ModelInstance *modelInstance;

    if (!get_model_idx_instance(&model_idx, &isEnabled, modelCollection, modelID)) {
        gbls.setStatus(&gbls, HARD_FAULT); // never returns
    }
    else {
        modelInstance = modelCollection->pModelInstance[model_idx];
        if (isEnabled&&(modelInstance->pFeature[0]->isNormalized)) {
            trainOCSVM_instance(modelInstance);
        }
    }
}

/** @brief Executes OC-SVM algorithm to estimate SVs
*
*       This function is for execution of OC-SVM. As a result of the algorithm, a set of support vectors (SVs) are provided, which build SVM model.
*       The parameter nu controls the bounds on the number of SVs and fraction of anomaly in training data set.
*       LIBSVM library is utilized to build the SVM model. Since LIBSVM uses dynamic memory allocation, this function follows the methodology
*       by using the capability provided by FreeRTOS. The dynamic memory allocation here is based on heap_4.c.
*       For the embedded system we are working on, we limited the maximum size of feature buffers, which means the maximum number of SVs is also limited.
*       This may reduce the risk of use of dynamic memory allocation in the current implementation.
*
*       @param  modelInstance The pointer of ModelInstance structure.
*       @return void.
*/
void trainOCSVM_malloc(struct ModelInstance *modelInstance)
{
    uint16_t i, j, r, k;
    struct svm_model            *model_trained;
    struct svm_problem          prob;
    struct svm_parameter        param;
    int startPos, endPos;
    bool reversed = modelInstance->reversedPosition;

    inputPositions(modelInstance, &startPos, &endPos);
    prob.l = endPos - startPos ;
    prob.y = Malloc(double, prob.l);
    prob.x = Malloc(struct svm_node *, prob.l);
    struct svm_node features[prob.l][modelInstance->feature_dimension+1];
    k = 0;
    for (i=startPos; i<endPos; i++)
    {
        if (!reversed) r = i;
        else r = i % MAX_FEATURE_SAMPLES;
        for (j=0; j<modelInstance->feature_dimension; j++) {
            features[k][j].index = j+1;
            if (modelInstance->pFeature[j]->isNormalized)
                features[k][j].value = (double) modelInstance->pFeature[j]->fBufferNormalized[r] ;
            else
                features[k][j].value = (double) modelInstance->pFeature[j]->fBuffer[r];
        }
        features[k][j].index = -1;
        prob.y[k] = 1;
        prob.x[k] = features[k];
        k++;
    }
    parse_svm_param(&modelInstance->pModel->ocsvm_model, &param);
    model_trained = svm_train(&prob, &param);
    save_model_instance(model_trained, &modelInstance->pModel->ocsvm_model);
    svm_free_and_destroy_model(&model_trained);
    Free(prob.y);
    Free(prob.x);
}

/** @brief To save the trained SVM model into a ModelInstance structure.
*
*       This function is to save the trained SVM model by LIBSVM into a ModelInstance structure. Type conversion is done.
*
*       @param  model   The pointer of struct svm_model.
*       @param  ocsvm   The pointer of struct OcsvmModel which is a union of Model in ModelInstance structure.
*       @return void.
*/
void save_model_instance(struct svm_model *model, struct OcsvmModel *ocsvm)
{
    int nr_class = model->nr_class;
    int l = model->l;
    ocsvm->opt_num_SVs = l;
    ocsvm->rho = (float) model->rho[0];
    struct svm_node *p;
    struct svm_node_embedded *q;

    for(int i=0;i<l;i++){
	for(int j=0;j<nr_class-1;j++)
            ocsvm->sv_coef[i] = (float) model->sv_coef[j][i];          // nr_class must be 2 for OC-SVM;
        p = model->SV[i];
        q = ocsvm->SV[i];
	while(p->index != -1) {
            q->index = (uint8_t) p->index;
            q->value = (float) p->value;
            p++;
            q++;
	}
    }
}

/** @brief To parse the input parameters for SVM before training.
*
*       This function is to parse the input parameters for SVM before training.
*
*       @param  ocsvm   The pointer of struct OcsvmModel which is a union of Model in ModelInstance structure.
*       @param  param   The pointer of struct svm_parameter which is defined in LIBSVM.
*       @return void.
*/
void parse_svm_param(struct OcsvmModel *ocsvm, struct svm_parameter *param)
{
    param->svm_type = ONE_CLASS;
    param->kernel_type = ocsvm->KernelType;
    param->gamma = 0.5/(ocsvm->KernelSize*ocsvm->KernelSize);
    param->coef0 = 0;
    param->nu = ocsvm->nu;
    param->cache_size = 0.5 ;          // MB // JL: if this value is too large, it causes hard fault. need to check. Original code uses 100;
    param->C = 1;
    param->eps = 1e-2;
    param->p = 0.1;
    param->shrinking = 1;
    param->probability = 0;
    param->weight = NULL;
    param->weight_label = NULL;
}



void trainEnsemble(struct ModelCollection *modelCollection)
{
//    int i;
//    struct ModelInstance *modelInstance;
//
//    for (i=0; i<modelCollection->num_LL_models; i++)
//    {
//        modelInstance = modelCollection->pModelInstance[i];
//        if (modelInstance->isEnabled) {
//            switch (modelInstance->model_type){
//            case GMM:
//                trainGMM_instance(modelInstance);
//                break;
//            case OCSVM:
//                trainOCSVM_instance(modelInstance);
//                break;
//            default:
//                break;
//            }
//        }
//    }
}


void clearFeatureBuffers_Instance(struct ModelInstance *modelInstance)
{
    modelInstance->startPositionBuffer = 0;
    modelInstance->endPositionBuffer = 0;
    modelInstance->pFeature[0]->iBufferCount = 0;
    if (modelInstance->feature_dimension == 2)
        modelInstance->pFeature[1]->iBufferCount = 0;
}





/*
* Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*! \file anomaly_detection.c
    \brief The anomaly_detection.c file implements the top level programming interface
*/

/* Including needed modules to compile this module/procedure */
#include <stdio.h>
#include <math.h>
#include "anomaly_detection.h"
#include "magnetic.h"
#include "drivers.h"
#include "sensor_drv.h"
#include "status.h"
#include "control.h"
#include "fusion.h"
//#include "debug.h"
#include "fsl_debug_console.h"  // KSDK header file for the debug interface
#include "timers.h"

float   training_time_used;
uint32_t xTimeNow, xTimeElapsed;

uint16_t feature_interval_number = 0;

/// Poor man's inheritance for status subsystem setStatus command
/// This function is normally involved via the "gbls." global pointer.
void setStatus(Globals *gbls, ad_status_t status)
{
    gbls->pStatusSubsystem->set(gbls->pStatusSubsystem, status);
}

/// Poor man's inheritance for status subsystem queueStatus command.
/// This function is normally involved via the "gbls." global pointer.
void queueStatus(Globals *gbls, ad_status_t status)
{
    gbls->pStatusSubsystem->queue(gbls->pStatusSubsystem, status);
}

/// Poor man's inheritance for status subsystem updateStatus command.
/// This function is normally involved via the "gbls." global pointer.
void updateStatus(Globals *gbls)
{
    gbls->pStatusSubsystem->update(gbls->pStatusSubsystem);
}

void testStatus(Globals *gbls)
{
    gbls->pStatusSubsystem->test(gbls->pStatusSubsystem);
}

/// utility function to insert default values in the top level structure
void initGlobals(Globals *gbls,
                             StatusSubsystem *pStatusSubsystem,
                             ControlSubsystem *pControlSubsystem,
                             FeatureCollection *pFeatureCollection,
                             ModelCollection *pModelCollection)
{
    gbls->iFlags = // all of the following defines are either 0x0000 or a 1-bit value (2, 4, 8 ...) and are defined in build.h
                F_USING_ACCEL           |
                F_USING_MAG             |
                F_USING_GYRO            |
                F_USING_PRESSURE        |
                F_USING_TEMPERATURE     |
                F_ALL_SENSORS                  // refers to all applicable sensor types for the given physical unit
;	// 9DOF accel, mag and gyro (Kalman): (accel + mag + gyro)

    gbls->pControlSubsystem = pControlSubsystem;
    gbls->pStatusSubsystem = pStatusSubsystem;
    gbls->pFeatureCollection = pFeatureCollection;
    gbls->pModelCollection = pModelCollection;
    gbls->loopcounter = 0;                     // counter incrementing each iteration of sensor fusion (typically 25Hz)
    gbls->systick_I2C = 0;                     // systick counter to benchmark I2C reads
    gbls->systick_Spare = 0;                   // systick counter for counts spare waiting for timing interrupt
    gbls->installSensor = installSensor;       // function for installing a new sensor into the structures
    gbls->initializeAD = initializeAD;   // function for installing a new sensor into the structures
    gbls->readSensors = readSensors;           // function for installing a new sensor into the structures
    gbls->runAD = runAD;               // function for installing a new sensor into the structures
    gbls->clearFIFOs = clearFIFOs;             // function to clear FIFO flags    gbls->applyPerturbation = ApplyPerturbation; // function used for step function testing
    gbls->setStatus = setStatus;               // function to immediately set status change
    gbls->queueStatus = queueStatus;           // function to queue status change
    gbls->updateStatus = updateStatus;         // function to promote queued status change
    gbls->testStatus = testStatus;             // function for unit testing the status subsystem
    gbls->pSensors = NULL;                     // pointer to linked list of physical sensors
//  put error value into whoAmI as initial value
#if F_USING_ACCEL
    gbls->Accel.iWhoAmI = 0;
#endif
#if F_USING_MAG
    gbls->Mag.iWhoAmI = 0;
#endif
#if F_USING_GYRO
    gbls->Gyro.iWhoAmI = 0;
#endif
#if F_USING_PRESSURE
    gbls->Pressure.iWhoAmI = 0;
#endif
}
/// installSensor is used to instantiate a physical sensor driver into the
/// sensor system.
/// This function is normally involved via the "gbls." global pointer.
int8_t installSensor(
                     Globals *gbls,  ///< top level fusion structure
                     struct PhysicalSensor *pSensor,    ///< pointer to structure describing physical sensor
                     uint16_t addr,             ///< I2C address for sensor (if applicable)
                     uint16_t schedule,         ///< Parameter to control sensor sampling rate
                     void *bus_driver,          ///< ISSDK sensor bus driver (usually KSDK I2C bus)
                     registerDeviceInfo_t *busInfo, ///< information required for bus power management
                     initializeSensor_t *initialize,    ///< pointer to sensor initialization function
                     readSensor_t *read)        ///< pointer to sensor read function
{
    if (gbls && pSensor && bus_driver && initialize && read)
    {
        pSensor->bus_driver = bus_driver;
        pSensor->deviceInfo.deviceInstance = busInfo->deviceInstance;
        pSensor->deviceInfo.functionParam = busInfo->functionParam;
        pSensor->deviceInfo.idleFunction = busInfo->idleFunction;

        pSensor->initialize = initialize;       // The initialization function is responsible for putting the sensor
                                                // into the proper mode for sensor fusion.  It is normally KSDK-based.
        pSensor->read = read;                   // The read function is responsible for taking sensor readings and
                                                // loading them into the sensor fusion input structures.  Also KDSK-based.
        pSensor->addr = addr;                   // I2C address if applicable
        pSensor->schedule = schedule;
        pSensor->slaveParams.pReadPreprocessFN = NULL;  // SPI-specific parameters get overwritten later if used
        pSensor->slaveParams.pWritePreprocessFN = NULL;
        pSensor->slaveParams.pTargetSlavePinID = NULL;
        pSensor->slaveParams.spiCmdLen = 0;
        pSensor->slaveParams.ssActiveValue = 0;
        // Now add the new sensor at the head of the linked list
        pSensor->next = gbls->pSensors;
        gbls->pSensors = pSensor;
        return (0);
    }
    else
    {
        return (1);
    }
}
// The initializeSensors function traverses the linked list of physical sensor
// types and calls the initialization function for each one.
int8_t initializeSensors(Globals *gbls)
{
    struct PhysicalSensor  *pSensor;
    int8_t          s;
    int8_t          status = 0;
    for (pSensor = gbls->pSensors; pSensor != NULL; pSensor = pSensor->next)
    {
        s = pSensor->initialize(pSensor, gbls);
        if (status == 0) status = s;            // will return 1st error flag, but try all sensors
    }
    return (status);
}

// process<Sensor>Data routines do post processing for HAL and averaging.  They
// are called from the readSensors() function below.
#if F_USING_ACCEL
void computeAccelFeatures(Globals *gbls)
{
    if (gbls->Accel.iFIFOExceeded > 0) {
      gbls->setStatus(gbls, SOFT_FAULT);
    }
    computeBasicFeatures((union FifoSensor *) &(gbls->Accel));
    return;
}
#endif
#if F_USING_MAG
void computeMagFeatures(Globals *gbls)
{
    // printf("ProcessingMagData()\n");
    if (gbls->Mag.iFIFOExceeded > 0) {
      gbls->setStatus(gbls, SOFT_FAULT);
    }

    return;
}
#endif
#if F_USING_GYRO
void computeGyroFeatures(Globals *gbls)
{
    if (gbls->Gyro.iFIFOExceeded > 0) {
      gbls->setStatus(gbls, SOFT_FAULT);
    }
    computeBasicFeatures((union FifoSensor *) &(gbls->Gyro));

    return;
}
#endif
/// readSensors traverses the linked list of physical sensors, calling the
/// individual read functions one by one.
/// This function is normally involved via the "gbls." global pointer.
int8_t readSensors(
    Globals *gbls,   ///< pointer to global sensor fusion data structure
    uint16_t read_loop_counter  ///< current loop counter (used for multirate processing)
)
{
    struct PhysicalSensor  *pSensor;
    int8_t          s;
    int8_t          status = 0;
    float           remainder;

    pSensor = gbls->pSensors;

    for (pSensor = gbls->pSensors; pSensor != NULL; pSensor = pSensor->next)
    {   if (pSensor->isInitialized) {
            remainder = fmod(read_loop_counter, pSensor->schedule);
            if (remainder==0) {
                s = pSensor->read(pSensor, gbls);
                if (status == 0) status = s;            // will return 1st error flag, but try all sensors
            }
        }
    }
    if (status==SENSOR_ERROR_INIT) gbls->setStatus(gbls, HARD_FAULT);  // Never returns
    return (status);
}
/// conditionSensorReadings() transforms raw software FIFO readings into forms that
/// can be consumed by the ADT.  This include sample averaging
/// and (in the case of the gyro) integrations, applying hardware abstraction layers,
/// and calibration functions.
/// This function is normally involved via the "gbls." global pointer.
void computeFeatures(Globals *gbls) {
#if F_USING_ACCEL
    if (gbls->Accel.isEnabled) computeAccelFeatures(gbls);
#endif

#if F_USING_MAG
    if (gbls->Mag.isEnabled) computeMagFeatures(gbls);
#endif

#if F_USING_GYRO
    if (gbls->Gyro.isEnabled) computeGyroFeatures(gbls);
#endif
    return;
}

void process_ML_command(Globals *gbls) {
        uint8_t modelID;
        bool sts=true;
        ModelCollection *modelCollection = gbls->pModelCollection;
        switch(gbls->controlCommandforML){
        case RUN_COMMAND:
          if (inEnsembleRange(gbls->current_model_id)) {
              for (int i=0; i<modelCollection->num_LL_models; i++) {
                  modelID = modelCollection->model_IDs_ensemble[i];
                  sts = sts && enableModel(modelCollection, modelID);
                  sts = sts && enableRunModel(modelCollection, modelID);
              }
              modelCollection->isEnsembleEnabled = true;
              modelCollection->iStage = RUN_COMMAND;            // to maintain the same iStage with LLs.
          }
          else {
              modelID = gbls->current_model_id;
              sts = sts && enableModel(modelCollection, modelID);
              sts = sts && enableRunModel(modelCollection, modelID);
          }
          if (sts)
              // streaming will be done in fusion_task() in main.c
              sts = sts;
          else
              gbls->setStatus(gbls, HARD_FAULT); // never returns
          break;
        case TRAIN_COMMAND:
          xTimeNow = xTaskGetTickCount();  // check execution time ticks for training.
          modelID = gbls->current_model_id;
          sts = sts && enableModel(modelCollection, modelID);
          sts = sts && trainModel(modelCollection, modelID);
          if (!sts || !streamModels(modelCollection, modelID))
                  gbls->setStatus(gbls, HARD_FAULT); // never returns
          xTimeElapsed = xTaskGetTickCount();
          training_time_used = xTimeElapsed - xTimeNow;
          break;
        case TED_COMMAND:      // TRAIN_ENSEMBLE_DEPENDENCIES
          for (int i=0; i<modelCollection->num_LL_models; i++) {
              modelID = modelCollection->model_IDs_ensemble[i];
              sts = sts && enableModel(modelCollection, modelID);
              sts = sts && trainModel(modelCollection, modelID);
              if (!sts || !streamModels(modelCollection, modelID))
                  gbls->setStatus(gbls, HARD_FAULT); // never returns
          }
          break;
        default:
          break;
        }
}


void zeroArray(StatusSubsystem *pStatus, void* data, uint16_t size, uint16_t numElements, uint8_t check) {
  uint16_t i;
  uint8_t *d8;
  uint16_t *d16;
  uint32_t *d32;
  switch(size) {
  case 8:
    d8 = data;
    for (i=0; i<numElements; i++) d8[i]=0;
    break;
  case 16:
    d16 = data;
    for (i=0; i<numElements; i++) d16[i]=0;
    break;
  case 32:
    d32 = data;
    for (i=0; i<numElements; i++) d32[i]=0;
    break;
  default:
    pStatus->set(pStatus, HARD_FAULT);
  }
  if (check) {
    switch(size) {
    case 8:
      d8 = data;
      for (i=0; i<numElements; i++)
        if (d8[i]!=0) pStatus->set(pStatus, HARD_FAULT);
      break;
    case 16:
      d16 = data;
      for (i=0; i<numElements; i++)
        if (d16[i]!=0) pStatus->set(pStatus, HARD_FAULT);
      break;
    case 32:
      d32 = data;
      for (i=0; i<numElements; i++)
        if (d32[i]!=0) pStatus->set(pStatus, HARD_FAULT);
      break;
    }
    return;
  }
}
/// Function to clear FIFO at the end of each fusion computation
void clearFIFOs(Globals *gbls) {
  // We only clear FIFOs if the sensors are enabled.  This allows us
  // to continue to use these values when we've shut higher power consumption
  // sensors down during periods of no activity.
  int i, j;
#if F_USING_ACCEL
    gbls->Accel.iFIFOCount=0;
    gbls->Accel.iFIFOExceeded = false;
    for (i=CHX; i<=VM; i++) {
        gbls->Accel.fSum[i] = 0;
        gbls->Accel.fSum2[i] = 0;
        for (j=MEAN; j<NORMALIZED_STD; j++) {
            gbls->Accel.features[j][i]=0;
        }
    }
#endif
#if F_USING_GYRO
    gbls->Gyro.iFIFOCount=0;
    gbls->Gyro.iFIFOExceeded = false;
    for (i=CHX; i<=VM; i++) {
        gbls->Gyro.fSum[i] = 0;
        gbls->Gyro.fSum2[i] = 0;
        for (j=MEAN; j<=NORMALIZED_STD; j++) {
            gbls->Gyro.features[j][i]=0;
        }
    }
#endif
}

/// runAD the top level call that actually runs the sensor fusion.
/// This is a utility function which manages the various defines in build.h.
/// You should feel free to drop down a level and implement only those portions
/// of fFuseSensors() that your application needs.
/// This function is normally involved via the "gbls." global pointer.
void runAD(Globals *gbls)
{
    feature_interval_number++;
    computeFeatures(gbls);
}

/// This function is responsible for initializing the system prior to starting
/// the main fusion loop.
/// This function is normally involved via the "gbls." global pointer.
void initializeAD(Globals *gbls)
{
    int16_t status = 0;

    // configure the 24 bit downwards ARM systick timer and wait 50ms=CORE_SYSTICK_HZ / 20 clock ticks
    // to avoid a race condition between Kinetis and the sensors after power on.
    ARM_systick_enable();
    // wait 50ms to avoid a race condition with sensors at power on
    ARM_systick_delay_ms(CORE_SYSTICK_HZ, 50);

    gbls->setStatus(gbls, INITIALIZING);
    status = initializeSensors(gbls);
    if (status!=SENSOR_ERROR_NONE) {  // fault condition found
        gbls->setStatus(gbls, HARD_FAULT);  // Never returns
    }

    // reset the loop counter to zero for first iteration
    gbls->loopcounter = 0;

    gbls->setStatus(gbls, NORMAL);

    clearFIFOs(gbls);
}
uint16 sign(float x) {
	if (x>=0) return 1;
	else return -1;
}

float max(float a, float b) {
  if (a>b) return (a);
  else return(b);
}

void computeBasicFeatures(union FifoSensor *sensor) {
  // Note that FifoSensor is a union of GyroSensor, MagSensor and AccelSensor.
  // All contain FIFO structures in the same location.  We use the Accel
  // structure to index here.
  float X, XXX, std, var, lastX;
  int i, CH;
  int N = sensor->Accel.iFIFOCount;
  for (CH=CHX; CH<=VM; CH++) {
      sensor->Accel.features[MEAN][CH] = sensor->Accel.fSum[CH] / N;
      for (i=0; i<N; i++) {
          // Detrend the data
          X  = sensor->Accel.fFIFO[i][CH] - sensor->Accel.features[MEAN][CH];
          sensor->Accel.fFIFO[i][CH] = X;
          // Compute sum V^2 for use in std/variance calculations
          sensor->Accel.fSum2[CH] += X*X;
      }
  }

  // We now have zero mean data
  for (CH=VM; CH>=0; CH--) {
      sensor->Accel.features[VARIANCE][CH] = sensor->Accel.fSum2[CH] / N;
      sensor->Accel.features[STD][CH] = sqrt(sensor->Accel.features[VARIANCE][CH]);
      sensor->Accel.features[NORMALIZED_STD][CH] = sensor->Accel.features[STD][CH]  / sensor->Accel.features[STD][VM];
  }
  for (CH=CHX; CH<=VM; CH++) {
      // Handle first element separate so we don't need conditional for CR computation
      X = sensor->Accel.fFIFO[0][CH];
      XXX = X*X*X;
      sensor->Accel.features[SKEW_FACTOR][CH] += XXX;
      sensor->Accel.features[KURTOSIS][CH] += X*XXX;
      sensor->Accel.features[CREST_FACTOR][CH] = max(sensor->Accel.features[CREST_FACTOR][CH], fabs(X));
      for (i=1; i<N; i++) {
          lastX = sensor->Accel.fFIFO[i-1][CH];
          X = sensor->Accel.fFIFO[i][CH];
          if ( sign(lastX) != sign(X) ) sensor->Accel.features[CROSSING_RATE][CH] += 1;
          XXX = X*X*X;
          sensor->Accel.features[SKEW_FACTOR][CH] += XXX;
          sensor->Accel.features[KURTOSIS][CH] += X*XXX;
          sensor->Accel.features[CREST_FACTOR][CH] = max(sensor->Accel.features[CREST_FACTOR][CH], fabs(X));
      }
      sensor->Accel.features[CREST_FACTOR][CH] /= sensor->Accel.features[STD][CH];
      sensor->Accel.features[CROSSING_RATE][CH] /= (N-1);
      var = sensor->Accel.features[VARIANCE][CH];
      std = sensor->Accel.features[STD][CH];
      sensor->Accel.features[SKEW_FACTOR][CH] /= ( N * var * std );
      sensor->Accel.features[KURTOSIS][CH] /= (N * var * var);
  }
}

void addToFifo(union FifoSensor *sensor, uint16_t maxFifoSize, int16_t sample[3])
{
  // Note that FifoSensor is a union of GyroSensor, MagSensor and AccelSensor.
  // All contain FIFO structures in the same location.  We use the Accel
  // structure to index here.

  // example usage: if (status==SENSOR_ERROR_NONE) addToFifo((FifoSensor*) &(gbls->Mag), MAG_FIFO_SIZE, sample);
    uint8_t fifoCount = sensor->Accel.iFIFOCount;
    float X, Y, Z, vm;
    if (fifoCount < maxFifoSize) {
        // we have room for the new sample
        X = sample[CHX];
        Y = sample[CHY];
        Z = sample[CHZ];
        X = sensor->Accel.fFloatPerCount * X;
        Y = sensor->Accel.fFloatPerCount * Y;
        Z = sensor->Accel.fFloatPerCount * Z;
        vm = sqrt( X*X + Y*Y + Z*Z );

        sensor->Accel.fFIFO[fifoCount][CHX] = X;
        sensor->Accel.fFIFO[fifoCount][CHY] = Y;
        sensor->Accel.fFIFO[fifoCount][CHZ] = Z;
        sensor->Accel.fFIFO[fifoCount][VM]  = vm;

        sensor->Accel.fSum[CHX]  += X;
        sensor->Accel.fSum[CHY]  += Y;
        sensor->Accel.fSum[CHZ]  += Z;
        sensor->Accel.fSum[VM]   += vm;

        sensor->Accel.iFIFOCount += 1;
        sensor->Accel.iFIFOExceeded = 0;
    } else {
        //there is no room for a new sample
        sensor->Accel.iFIFOExceeded += 1;
    }

}

void clearFeatureBuffers(struct ModelCollection *modelCollection)
{
    int i;

    for (i=0; i<modelCollection->num_LL_models; i++)
    {
        clearFeatureBuffers_Instance(modelCollection->pModelInstance[i]);
    }

}


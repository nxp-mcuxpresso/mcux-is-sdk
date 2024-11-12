/*
* Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

// Sensor fusion requires a fairly extensive set of data structures, which are
// defined in this file.  The top level structure is shown near the bottom.  The
// size of this structure (Globals) varies dramatically as a function
// of which fusion variations have been selected in build.h.

/*! \file anomaly_detection.h
    \brief The anomaly_detection.h file implements the top level programming interface
*/

#ifndef ANOMALY_DETECTION_TYPES_H
#define ANOMALY_DETECTION_TYPES_H

// Standard includes that everyone needs
#include "math.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdint.h"

#include "issdk_hal.h"                  // Hardware Abstraction Layer board dependencies beyond those generated in board.h by PEX
#include "build.h"                      // This is where the build parameters are defined
#include "orientation.h"                // Functions for manipulating orientations
#include "register_io_spi.h"

/// @name Integer Typedefs
/// Typedefs to map common integer types to standard form
///@{
typedef unsigned char byte;
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
///@}

#define MAX_LLMODEL_NUMBER                      0x10
#define MAX_ENSEMBLE_MODEL_NUMBER               0x20

// booleans
#define true 1  ///< Boolean TRUE
#define false 0 ///< Boolean FALSE

/// @name Generic bit-field values
/// Generic bit-field values
///@{
#define B0 (1 << 0)
#define B1 (1 << 1)
#define B2 (1 << 2)
#define B3 (1 << 3)
///@}

/// @name Math Constants
/// useful multiplicative conversion constants
///@{
#define PI 3.141592654F				///< pi
#define PIOVER2 1.570796327F			///< pi / 2
#define FPIOVER180 0.01745329251994F	        ///< degrees to radians conversion = pi / 180
#define F180OVERPI 57.2957795130823F	        ///< radians to degrees conversion = 180 / pi
#define F180OVERPISQ 3282.8063500117F	        ///< square of F180OVERPI
#define ONETHIRD 0.33333333F			///< one third
#define ONESIXTH 0.166666667F			///< one sixth
#define ONESIXTEENTH 0.0625F			///< one sixteenth
#define ONEOVER12 0.083333333F			///< 1 / 12
#define ONEOVER48 0.02083333333F		///< 1 / 48
#define ONEOVER120 0.0083333333F		///< 1 / 120
#define ONEOVER3840 0.0002604166667F	        ///< 1 / 3840
#define ONEOVERSQRT2 0.707106781F		///< 1/sqrt(2)
#define SQRT15OVER4  0.968245837F		///< sqrt(15)/4
#define GTOMSEC2 9.80665			///< standard gravity in m/s2
///@}

// Placeholder structures (redefined later, but needed now for pointer definitions)
struct Globals;                                 ///< Top level structure has pointers to everything else
struct StatusSubsystem;                         ///< Application-specific status subsystem
struct PhysicalSensor;                          ///< We'll have one of these for each physical sensor (FXOS8700 = 1 physical sensor)
struct ControlSubsystem;                        ///< Application-specific serial communications system


#define  CI_ACCELEROMETER_X    0x01
#define  CI_ACCELEROMETER_Y    0x02
#define  CI_ACCELEROMETER_Z    0x03
#define  CI_ACCELEROMETER_VM   0x04
#define  CI_GYRO_X             0x05
#define  CI_GYRO_Y             0x06
#define  CI_GYRO_Z             0x07
#define  CI_GYRO_VM            0x08
#define  CI_TEMPERATURE        0x09
#define  CI_PRESSURE           0x0A
#define  CI_MICROPHONE         0x0B

#define  CI_MEAN            0x0040
#define  CI_VARIANCE        0x0020
#define  CI_SKEW_FACTOR     0x0010
#define  CI_KURTOSIS        0x0008
#define  CI_CROSSING_RATE   0x0004
#define  CI_STD             0x0002
#define  CI_NORMALIZED_STD  0x0001

/// \brief Machine Learning Algorithms types
typedef enum{
    NO_MODEL=0,
    GMM,                        ///< gaussian mixture model. EM algorithm will be used.
    OCSVM,                      ///< one-class SVM is assumed.
    ENSEMBLE
} model_t;

/// @name Vector Components
/// Index values for accessing vector terms
typedef enum {CHX = 0, CHY, CHZ, VM, SCALAR, END_OF_AXIS_TYPES } axis_t;

typedef enum {
        ACCEL = 0,
        GYRO,
        MAG,
        TEMPERATURE,
        PRESSURE,
        MICROPHONE,
        END_OF_SENSORS} sensor_t;

typedef enum {
        MEAN = 0,
        VARIANCE,
        SKEW_FACTOR,
        KURTOSIS,
        CROSSING_RATE,
        STD,
        NORMALIZED_STD,
	CREST_FACTOR,
        END_OF_FEATURES} feature_t;

#define NUM_FEATURES (END_OF_FEATURES-1)

typedef enum {                                  ///  These are the state definitions for the status subsystem
	OFF,                                    ///< Application hasn't started
	INITIALIZING,                           ///< Initializing sensors and algorithms
	LOWPOWER,                               ///< Running in reduced power mode
	NORMAL,                                 ///< Operation is Nominal
        RECEIVING,                              ///< Receiving commands over wired interface (momentary)
        SENDING,                                ///< Sending data over wireless interface (momentary)
	HARD_FAULT,                             ///< Non-recoverable FAULT = something went very wrong
        SOFT_FAULT                              ///< Recoverable FAULT = something went wrong, but we can keep going
} ad_status_t;

typedef enum { // See the Set Model Mode Command byte definition
        NO_COMMAND = 0x00,
        CLR_COMMAND = 0x01,
        STOP_COMMAND = 0x02,
        RUN_COMMAND = 0x04,
        TRAIN_COMMAND = 0x08,
        DELETE_COMMAND = 0x10,
        DELETE_ALL_COMMAND = 0x20,
        TED_COMMAND = 0x40      // TRAIN_ENSEMBLE_DEPENDENCIES
} command_t;

// declare typedefs for function prototypes that need to be installed
typedef int8_t (initializeSensor_t) (
    struct PhysicalSensor *sensor,
    struct Globals *gbls
) ;
typedef int8_t (readSensor_t) (
    struct PhysicalSensor *sensor,
    struct Globals *gbls
) ;
typedef int8_t (readSensors_t) (
    struct Globals *gbls,
    uint16_t read_loop_counter
) ;
typedef int8_t (installSensor_t) (
    struct Globals *gbls,    ///< Global data structure pointer
    struct PhysicalSensor *sensor,      ///< SF Structure to store sensor configuration
    uint16_t addr,                      ///< I2C address or SPI_ADDR
    uint16_t schedule,                  ///< Specifies sampling interval
    void *bus_driver,                   ///< I2C or SPI handle
    registerDeviceInfo_t *busInfo,      ///< information required for bus power management
    initializeSensor_t *initialize,     ///< SF Sensor Initialization Function pointer
    readSensor_t *read                  ///< SF Sensor Read Function pointer
);
#define SPI_ADDR 0x00   // Use SPI_ADDR as the address parameter to the installSensor function for SPI-based sensors.
                        // 0x00 is reserved for I2C General Call, and will therefore never occur for any sensor type

typedef void   (initializeFusionEngine_t) 	(struct Globals *gbls);
typedef void   (runFusion_t) 			(struct Globals *gbls);
typedef void   (clearFIFOs_t) 			(struct Globals *gbls);
typedef void   (setStatus_t) 			(struct Globals *gbls, ad_status_t status);
typedef void   (updateStatus_t) 		(struct Globals *gbls);
typedef void   (ssSetStatus_t) 			(struct StatusSubsystem *pStatus, ad_status_t status);
typedef void   (ssUpdateStatus_t) 		(struct StatusSubsystem *pStatus);

/// \brief An instance of PhysicalSensor structure type should be allocated for each physical sensors (combo devices = 1)
///
/// These structures sit 'on-top-of' the pre-7.0 sensor fusion structures and give us the ability to do run
/// time driver installation.
struct PhysicalSensor {
        registerDeviceInfo_t deviceInfo;        ///< I2C device context
	void *bus_driver;  			///< should be of type (ARM_DRIVER_I2C* for I2C-based sensors, ARM_DRIVER_SPI* for SPI)
        registerDeviceInfo_t *busInfo;          ///< information required for bus power management
	uint16_t addr;  			///< I2C address if applicable
        uint16_t isInitialized;                 ///< Bitfields to indicate sensor is active (use SensorBitFields from build.h)
        spiSlaveSpecificParams_t slaveParams;   ///< SPI specific parameters.  Not used for I2C.
	struct PhysicalSensor *next;		///< pointer to next sensor in this linked list
        uint16_t schedule;                      ///< Parameter to control sensor sampling rate
	initializeSensor_t *initialize;  	///< pointer to function to initialize sensor using the supplied drivers
	readSensor_t *read;			///< pointer to function to read       sensor using the supplied drivers
};

// Now start "standard" sensor fusion structure definitions

/// \brief The PressureSensor structure stores raw and processed measurements for an altimeter.
///
/// The PressureSensor structure stores raw and processed measurements, as well as
/// metadata for a pressure sensor/altimeter.
struct PressureSensor
{
	uint8_t iWhoAmI;		        ///< sensor whoami
        bool  isEnabled;                        ///< true if the device is sampling
	int32_t iH;				///< most recent unaveraged height (counts)
	int32_t iP;				///< most recent unaveraged pressure (counts)
	float fH;				///< most recent unaveraged height (m)
	float fT;				///< most recent unaveraged temperature (C)
	float fmPerCount;		        ///< meters per count
	float fCPerCount;		        ///< degrees Celsius per count
	int16_t iT;				///< most recent unaveraged temperature (counts)
};

/// \brief The AccelSensor structure stores raw and processed measurements for a 3-axis accelerometer.
///
/// The AccelSensor structure stores raw and processed measurements, as well as metadata
/// for a single 3-axis accelerometer.  This structure is
/// normally "fed" by the sensor driver and "consumed" by the fusion routines.
struct AccelSensor
{
	uint8_t iWhoAmI;			///< sensor whoami
        bool  isEnabled;                        ///< true if the device is sampling
	uint8_t iFIFOCount;			///< number of measurements read from FIFO
        uint16_t iFIFOExceeded;                 ///< Number of samples received in excess of software FIFO size
	float fFloatPerCount;			///< g per count
						// fSum, fSum2 and features are all cleared by clearFIFOs
	float fSum[4];			        ///< sum of all measurements in this epoch
	float fSum2[4];				///< sum of squares over all measurements in this epoch
        float features[NUM_FEATURES][4];      ///< rows are mean, variance, SF, kurtosis and mean crossing rate
	float fFIFO[ACCEL_FIFO_SIZE][4];	///< FIFO measurements
        // End of common fields which can be referenced via FifoSensor union type
};

/// \brief The MagSensor structure stores raw and processed measurements for a 3-axis magnetic sensor.
///
/// The MagSensor structure stores raw and processed measurements, as well as metadata
/// for a single 3-axis magnetometer.  This structure is
/// normally "fed" by the sensor driver and "consumed" by the fusion routines.
struct MagSensor
{
	uint8_t iWhoAmI;			///< sensor whoami
        bool  isEnabled;                        ///< true if the device is sampling
	uint8_t iFIFOCount;			///< number of measurements read from FIFO
        uint16_t iFIFOExceeded;                 ///< Number of samples received in excess of software FIFO size
	float fFloatPerCount;			///< uT per count
	float fSum[4];			        ///< sum of all measurements in this epoch
	float fSum2[4];				///< sum of squares over all measurements in this epoch
        float features[NUM_FEATURES][4];        ///< rows are mean, variance, SF, kurtosis and mean crossing rate
	float fFIFO[MAG_FIFO_SIZE][4];	        ///< FIFO measurements
        // End of common fields which can be referenced via FifoSensor union type
};

/// \brief The GyroSensor structure stores raw and processed measurements for a 3-axis gyroscope.
///
/// The GyroSensor structure stores raw and processed measurements, as well as metadata
/// for a single 3-axis gyroscope.  This structure is
/// normally "fed" by the sensor driver and "consumed" by the fusion routines.
struct GyroSensor
{
	uint8_t iWhoAmI;			///< sensor whoami
        bool  isEnabled;                        ///< true if the device is sampling
	uint8_t iFIFOCount;			///< number of measurements read from FIFO
        uint16_t iFIFOExceeded;                 ///< Number of samples received in excess of software FIFO size
	float fFloatPerCount;		        ///< deg/s per count
						//   fSum, fSum2 and features are all cleared by clearFIFOs
	float fSum[4];			        ///< sum of all measurements in this epoch
	float fSum2[4];				///< sum of squares over all measurements in this epoch
        float features[NUM_FEATURES][4];        ///< rows are mean, variance, SF, kurtosis and mean crossing rate
	float fFIFO[GYRO_FIFO_SIZE][4];	        ///< FIFO measurements (counts)
        // End of common fields which can be referenced via FifoSensor union type
};

/// \brief The FifoSensor union allows us to use common pointers for Accel, Mag & Gyro logical sensor structures.
///
/// Common elements include: iWhoAmI, isEnabled, iFIFOCount, iFIFOExceeded and the FIFO itself.
union FifoSensor  {
    struct GyroSensor Gyro;
    struct MagSensor  Mag;
    struct AccelSensor Accel;
};

/// \brief The top level fusion structure
///
/// The top level fusion structure grows/shrinks based upon flag definitions
/// contained in build.h.  These same flags will populate the .iFlags field for
/// run-time access.
typedef struct Globals
{
	// Subsystem Pointers
        ///@{
        /// @name SubsystemPointers
        /// The Status and Control subsystems can be used as-is, or completely
        /// replaced with alternate implementations, as long as those implementations
        /// provide the same interfaces defined in control.h and status.h.
	struct ControlSubsystem *pControlSubsystem;
	struct StatusSubsystem *pStatusSubsystem;

        // JL: Machine Learning subsystems pointers
        struct FeatureCollection *pFeatureCollection;     ///< pointer for feature operation object
        struct ModelCollection *pModelCollection;        ///< pointer model operation object
        uint8_t controlCommandforML;                     ///< machine learning control commands received from GUI: e.g., start training, download, etc.
        uint8_t current_model_id;


        ///@{
        /// @name MiscFields
        uint32_t iFlags;                        ///< a bit-field of sensors and algorithms used
	struct PhysicalSensor *pSensors;    	        ///< a linked list of physical sensors
	volatile uint8_t iPerturbation;	        ///< test perturbation to be applied
	// Book-keeping variables
	int32_t loopcounter;			///< counter incrementing each iteration of sensor fusion (typically 25Hz)
	int32_t systick_I2C;			///< systick counter to benchmark I2C reads
	int32_t systick_Spare;			///< systick counter for counts spare waiting for timing interrupt
        ///@}
        ///@{
        /// @name SensorRelatedStructures
        /// These structures provide homes for sensor readings, as well as
        /// various calibration functions.  Only those needed for a specific
        /// build are included.
#if     F_1DOF_P_BASIC
	struct PressureSensor	Pressure;       ///< pressure sensor storage
#endif
#if     F_USING_ACCEL
	struct AccelSensor 	Accel;                  ///< accelerometer storage
#endif
#if     F_USING_MAG
	struct MagSensor 	Mag;                    ///< magnetometer storage
#endif
#if     F_USING_GYRO
	struct GyroSensor 	Gyro;                   ///< gyro storage
#endif
        ///@}
        ///@{
        /// @name FunctionPointers
        /// Function pointers (the SF library external interface)
	installSensor_t 	*installSensor;         ///< function for installing a new sensor into t
	initializeFusionEngine_t *initializeAD ;  ///< set sensor fusion structures to initial values
	readSensors_t		*readSensors;		///< read all physical sensors
	runFusion_t		*runAD;		///< run the fusion routines
        clearFIFOs_t            *clearFIFOs;            ///< clear sensor FIFOs
	setStatus_t		*setStatus;		///< change status indicator immediately
	setStatus_t		*queueStatus;  	        ///< queue status change for next regular interval
	updateStatus_t		*updateStatus; 		///< status=next status
	updateStatus_t		*testStatus; 		///< increment to next enumerated status value (test only)
        ///@}
} Globals;

// The following functions are defined in anomaly_detection.c
void initGlobals(
    Globals *gbls,                           ///< Global data structure pointer
    struct StatusSubsystem *pStatusSubsystem,           ///< Status subsystem pointer
    struct ControlSubsystem *pControlSubsystem ,         ///< Control subsystem pointer
    struct FeatureCollection *pFeatureCollection,       ///< Feature collection
    struct ModelCollection *pModelCollection            ///< Model collection
);
installSensor_t installSensor;
initializeFusionEngine_t initializeAD ;
/// computeFeatures() transforms raw software FIFO readings into features to
/// be used as input to the ML routines
void computeFeatures(
    Globals *gbls                            ///< Global data structure pointer
);

void computeBasicFeatures(
    union FifoSensor *sensor                ///< logical sensor data structure pointer
);
void clearFIFOs(
    Globals *gbls                            ///< Global data structure pointer
);
runFusion_t runAD;
readSensors_t readSensors;
void zeroArray(
    struct StatusSubsystem *pStatus,                    ///< Status subsystem pointer
    void* data,                                         ///< pointer to array to be zeroed
    uint16_t size,                                      ///< data type size = 8, 16 or 32
    uint16_t numElements,                               ///< number of elements to zero out
    uint8_t check                                       ///< true if you would like to verify writes, false otherwise
);

// The following functions are defined in <hal_board_name>.c.
// Please note that these are board-dependent

/// \brief addToFifo is called from within sensor driver read functions
///
/// addToFifo is called from within sensor driver read functions to transfer new readings into
/// the sensor structure corresponding to accel, gyro or mag.  This function ensures that the software
/// FIFOs are not overrun.
///
/// example usage: if (status==SENSOR_ERROR_NONE) addToFifo((FifoSensor*) &(gbls->Mag), MAG_FIFO_SIZE, sample);
void addToFifo(
    union FifoSensor *sensor,                           ///< pointer to structure of type AccelSensor, MagSensor or GyroSensor
    uint16_t maxFifoSize,                               ///< the size of the software (not hardware) FIFO
    int16_t sample[3]                                   ///< the sample to add
);
/// \brief Apply the accelerometer Hardware Abstraction Layer
void ApplyAccelHAL(
    struct AccelSensor *Accel                                  ///< pointer to accelerometer logical sensor
);
/// \brief Apply the magnetometer Hardware Abstraction Layer
void ApplyMagHAL(
    struct MagSensor *Mag                                     ///< pointer to magnetometer logical sensor
);
/// \brief Apply the gyroscope Hardware Abstraction Layer
void ApplyGyroHAL(
    struct GyroSensor *Gyro                                    ///< pointer to gyroscope logical sensor
);
/// \brief ApplyPerturbation is a reverse unit-step test function
///
/// The ApplyPerturbation function applies a user-specified step function to
/// prior fusion results which is then "released" in the next fusion cycle.
/// When used in conjuction with the NXP Sensor Fusion Toolbox, this provides
/// a visual indication of the dynamic behavior of the library. ApplyPerturbation()
/// is defined in debug.c.

#include "matrix.h"  // this is only down here so we can take advantage of _t style typedefs above
#include "machineLearning_subsystem.h"
#include "output_stream.h"

extern uint16_t feature_interval_number; // increments by 1 for each feature interval
void process_ctr_command(Globals *gbls);
void process_ML_command(Globals *gbls);
void clearFeatureBuffers(struct ModelCollection *modelCollection);
#endif // ANOMALY_DETECTION_TYPES_H

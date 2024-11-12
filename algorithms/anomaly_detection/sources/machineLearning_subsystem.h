/*
* Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/** @file machineLearning_subsystem.h
*   @brief Data structures and function prototypes for the machine learning library.
*
*   This contains the data structure and function proto types for the machine learning algorithm library.
*   General ones are described here. Additional structures and prototypes for GMM and SVM are in "gmm_utility.h" and "svm.h" respectively.
*/

#ifndef MACHINELEARNING_SUBSYSTEM_H_
#define MACHINELEARNING_SUBSYSTEM_H_

#include "svm.h"
#include "FreeRTOS.h"

#define MAX_GMM                                 4       ///< Maximum number of Gaussian Mixture Model allowed to train/run in the embedded system.
#define MAX_GAUSSIAN_COMPONENTS                 7       ///< Maximum number of Gaussian components allowed for a single GMM.
#define MAX_FEATURE_DIMENSION                   2       ///< Maximum dimension of features. Each feature represents an axis.
#define MAX_FEATURE_SAMPLES                     200     ///< Maximum number of feature samples that is allowed to train a model.
#define MAX_FEATURE_INSTANCES                   10      ///< Maximum number of feature instances. Each feature instance is linked to model instances.
#define MAX_MODEL_INSTANCES                     MAX_GMM + MAX_SVM       ///< Maximum number of model instances. The total number is the sum of the number GMMs and SVMs.
#define MAX_SUBMODELS                           MAX_MODEL_INSTANCES

#define DEBUG_ENABLE_TX_VARIABLE_IS_ON         0       ///< 0: not debug (not using the modelInstance->enableTransmit variable), 1: debugging (it uses it modelInstance->enableTransmit).

/// \brief Feature Instance structure contains the buffers for feature samples and other information.
///
/// Feature type, sensor type, and sensor axis are used to identify feature instance.
/// fBufferNormalized is the buffer for normalized features (zscore) from fBuffer, fBufferMean, and fBufferStd.
/// The other variables in FeatureInstance structure are control variables.
typedef struct FeatureInstance {
    feature_t   feature_type;                   ///< Feature type defined in feature_t.
    sensor_t    sensor_type;                    ///< Sensor type defined in sensor_t.
    axis_t      sensor_axis;                    ///< Sensor axis defined in axis_t.
    int16_t     iBufferCount;                   ///< Count feature samples when stored in fBuffer
    bool        iBufferExceeded;                ///< True if fBuffer is filled.
    bool        isEnabled;                      ///< True if this FeatureInstance structure was enabled.
    bool        isNormalized;                   ///< True if this FeatureInstance was normalized (assumed zscore, i.e. standardization)
    bool        doNormalization;                ///< True if feature normalization is reqested.
    float       fBufferMean;                    ///< Sample mean to compute zscore, within a moving window whose size is defined by featureBufferSize in ModelInstance structure.
    float       fBufferStd;                     ///< Sample std to compute zscore, within a moving window whose size is defined by featureBufferSize in ModelInstance structure.
    float       fBufferNormalized[MAX_FEATURE_SAMPLES];   ///< A buffer array, normalized from fBuffer.
    float       fBuffer[MAX_FEATURE_SAMPLES];        ///< A buffer that contains un-normalized features from raw sensor data.
} FeatureInstance;

/// \brief Feature Collection structure contains the poinsters of FeatureInstance structures.
///
/// Potentially, control parameters for a collection of Feature Instances may be added here.
typedef struct FeatureCollection {
    struct FeatureInstance      *pFeatureInstance[MAX_FEATURE_INSTANCES];       ///< pointers of feature instances
} FeatureCollection;

/// \brief A Gaussian component structure in the mixture model.
///
/// This structure contains the computational information as well as the parameter set of a single Gaussian component in the mixture model.
/// GaussCompoentn is included in GmmModel structure.
typedef struct GaussComponent {
    float       N;                              ///< Soft number of samples assigned to this Gaussian component.
    float       pi;                             ///< Mixing probability.
    float       cnst;                           ///< 1/sqrt((2PI)^dim * determinant) in log scale.
    float       muMeans[MAX_FEATURE_DIMENSION]; ///< Mean vector.
    float       rCovariance[MAX_FEATURE_DIMENSION][MAX_FEATURE_DIMENSION];      ///< Covariance matrix.
    float       rInvCovariance[MAX_FEATURE_DIMENSION][MAX_FEATURE_DIMENSION];   ///< Inverse covariance matrix.
} GaussComponent;

/// \brief A Gaussian mixture model (GMM) structure
///
/// GMM parameters as well as trained GMM by expectation-maximization (EM) algorithm.
/// These are linked to ModelInstance.
typedef struct GmmModel {
    bool                initialized;
    uint8_t             opt_num_components;             ///< The optimal number of GaussComponent in the mixture model. It is set after trained.
    uint8_t             init_maxGaussComponents;           ///< The allowed max number of GaussComponent for training.
    int8_t              nComponents;                    ///< The number of GaussComponent used during training time.
    float               Rmin;                           ///< For regularization of covariance matrix. Singularity may be avoided.
    float               pProb[MAX_FEATURE_SAMPLES][MAX_GAUSSIAN_COMPONENTS];   ///< The probability that a feature sample belongs to a mixture component. For example, pProb[i][j] represents the probability for feature sample i to j component.
    GaussComponent      gComponents[MAX_GAUSSIAN_COMPONENTS];   ///< Collection of Gaussian components used for the mixture model
} GmmModel;

/// \brief A structure for each support vector.
///
/// This structure was defined for embedded systems with reduced memory usage, rather than using 8 bytes double type.
/// It follows the same structure of LIBSVM.
typedef struct svm_node_embedded {
	uint8_t index;          ///< Index indicates the dimensionality.
	float   value;          ///< A single dimensional value of a SV.
} svm_node_embedded;

/// \brief A One Class Support Vector Machine (OC-SVM) structure
///
/// OC-SVM parameters as well as trained model by use of LIBSVM.
/// This structure is linked to ModelInstance.
typedef struct OcsvmModel {
    uint8_t                     modelID;                ///< The model number, assigned from GUI.
    uint8_t                     opt_num_SVs;            ///< The optimal number of SVs decided after training.
    uint8_t                     KernelType;             ///< A Kernel type is chosen from this list: LINEAR, RBF, POLY, SIGMOID, PRECOMPUTED. But current version only considers RBF.
    float                       KernelSize;             ///< The kernel size sigma for RBF. In LIBSVM, RBF is defined as exp( - gamma * x^2). Thus, gamma = 1 / 2 * sigma^2.
    float                       nu;                     ///< Nu parameter for OC-SVM affects the bounds on the number SVs and fraction of anomaly. More details are in [Scholkopf et al., "Estimating the Support of a High-Dimensional Distriution].
    struct svm_node_embedded    SV[MAX_SVM_SVs][MAX_FEATURE_DIMENSION];         ///< Collection of SVs.
    float                       sv_coef[MAX_SVM_SVs];           ///< Coefficients that correspond to the trained SVs.
    float                       rho;                    ///< Threshold computed after training.
} OcsvmModel;

/// \brief A Union type model structure
///
/// Each modelInstance has one of these two types of models.
union Model {
    struct GmmModel     gmm_model;
    struct OcsvmModel   ocsvm_model;
};

/// \brief Model Instance Structure contains the buffers for feature samples and other information.
///
/// Model type and ID are used to identify model instance.
/// The selected feature instances are linked to a model instance.
/// GMM and OC-SVM are linked to a model instance as a union. So, only one model is used in a model instance.
typedef struct ModelInstance {
    model_t                             model_type;             ///< Machine learning model type defined in model_t.
    uint8_t                             modelID;                ///< Model name (index) determined by a user in GUI
    uint8_t                             iStage;                 ///< Stages for each algorithm. They are set by gbls->controlCommandforML.
    uint8_t                             feature_dimension;      ///< Dimensionality of feature inputs for this model instance
    uint8_t                             featureBufferSize;      ///< This determins a window size, moving in fBufferNormalized. We may have different buffer use for different models.
    int                                 startPositionBuffer;    ///< Start position of a moving window.
    int                                 endPositionBuffer;      ///< End position of a moving window.
    bool                                reversedPosition;       ///< An indicator whether the start-end positions are reversed. Note the buffer size limited, and the window moves in a circular type buffer.
    bool                                isEnabled;               ///< True if the model instance is enabled.
    float                               threshold;              ///< A threshold value to detect anomality.
    bool                                isTrained ;              ///< True once the model instance is trained.
    bool                                decision_hard ;          ///< Binary decision of anomaly
    float                               decision_soft;           ///< Soft decision such as output of a probability density function.
    bool                                enableTransmit;
    struct FeatureInstance              *pFeature[MAX_FEATURE_DIMENSION];       ///< pointers that indicates feature instances
    union Model                         *pModel;                ///< A pointor that indicates a union of GMM and OC-SVM.
} ModelInstance;

/// \brief Model Collection structure contains the poinsters of ModelInstance structures.
///
/// Potentially, control parameters for a collection of Model Instances may be added more.
typedef struct ModelCollection {
    struct ModelInstance                *pModelInstance[MAX_MODEL_INSTANCES];
    uint16_t                            training_rate;                          ///< variable to define how often the model instance is trained. every "training_rate" feature samples.
    uint8_t                             model_IDs_ensemble[MAX_MODEL_INSTANCES];///< array that contains lower level model IDs.
    uint8_t                             ensemble_modelID;                       ///< ensemble model ID (range: 0x11 - 0x1F)
    uint8_t                             num_LL_models;                          ///< # of lower level models
    uint8_t                             votes_required;                         ///< condition to fuse lower level model decisions.
    bool                                isEnsembleEnabled;                              ///< True if the ensemble is enabled.
    uint8_t                             iStage; 
} ModelCollection;


//////////////////////////////////////////////////////////////////
//////////////// performance evaluation related //////////////////
/////////////////////////////////////////////////////////////////

// ROC
// confusion matrix, if possible
// ..

///////////////////////////////////////////////////////////////
//////////////// end of performance evaluation ////////////////
///////////////////////////////////////////////////////////////




void initFeatureCollection(struct FeatureCollection *featureCollection, struct FeatureInstance *featureInstance);
void initModelCollection(struct ModelCollection *models, struct ModelInstance *modelInstance, union Model *ml_model);
void addToFeatureBuffers(struct Globals *gbls);
bool initGMM(struct ModelInstance *modelInstance);
void runGMMEM(struct ModelInstance *modelInstance);
void trainGMM(struct ModelCollection *models, uint8_t modelID);
float getLastFeatureValueAdded(FeatureInstance *featureInstance);
int8_t idxViaFindOrAssignModelID(ModelCollection *models, uint8_t modelID);
void disableAllModels(struct ModelCollection *models);
bool enableRunModel(struct ModelCollection *modelCollection, uint8_t modelID);
bool enableModel(struct ModelCollection *modelCollection, uint8_t modelID);
void incrementFeatureBufferIndices(struct Globals *gbls);
void detectAnomalyInGMM(struct ModelInstance *modelInstance);
void computeZscore(float *xn, float x, float mu, float sigma);
void incrementFeatureBufferIndicesModels(struct Globals *gbls);
void incrementFeatureBufferIndicesStartEnd(struct ModelInstance *model);
bool checkReversed(int startPosition, int endPosition);
void inputPositions(struct ModelInstance *modelInstance, int *startPos, int *endPos);
void zscoreFeatBuffer(struct FeatureInstance *featureInstance);
void refreshGMM(struct ModelInstance *modelInstance);
bool checkMixProb(struct GmmModel *gmm_model);
bool computeR(struct ModelInstance *modelInstance, float R[][2]);

void normalizeFeatures (struct ModelInstance *modelInstance);
void normalizeFeatWindow (struct FeatureInstance *feat_inst, int sPos, int ePos, bool reversed);

float sumBuffer(float *pBuffer, int startPos, int endPos, bool reversed);
float sumSquaredBuffer(float *pBuffer1, float *pBuffer2, int startPos, int endPos, bool reversed);

void trainGMM_instance(struct ModelInstance *modelInstance);
void clearFeatureBuffers_Instance(struct ModelInstance *modelInstance);


// OC-SVM related
void trainOCSVM(struct ModelCollection *modelCollection, uint8_t modelID);
void trainOCSVM_malloc(struct ModelInstance *modelInstance);
void parse_svm_param(struct OcsvmModel *ocsvm, struct svm_parameter *param);
void save_model_instance(struct svm_model *model, struct OcsvmModel *ocsvm);
void detectAnomalyInSVM(struct ModelInstance *modelInstance);
void trainOCSVM_instance(struct ModelInstance *modelInstance);

// ensemble model
void detectAnomalyInEnsemble(struct ModelInstance *modelInstance) ;
void trainEnsemble(struct ModelCollection *modelCollection);

/// \brief Feature Operation Functions
/// control functions for machine learning feature operation for creating and saving features from raw sensor data.
void addToFeatBuffer(struct Globals *gbls, struct FeatureInstance *feature_instance);
bool get_feature_attributes(struct FeatureCollection *features, uint8_t idx, sensor_t *sensor, axis_t *axis, feature_t *feature);
bool delete_feature_by_attributes(struct FeatureCollection *features, sensor_t sensor, axis_t axis, feature_t feature);
bool delete_feature_by_index(struct FeatureCollection *features, uint8_t idx);
bool add_feature(uint8_t *idx, struct FeatureCollection *features, sensor_t sensor, axis_t axis, feature_t feature);
bool get_feature_number(uint8_t *idx, struct FeatureCollection *features, sensor_t sensor, axis_t axis, feature_t feature);


/// \brief Model Operation Functions
/// control functions for machine learning models operation such as training, testing, etc.
bool get_model_idx_instance(uint8_t *idx, bool *is_enabled, struct ModelCollection *models, uint8_t modelID);
bool trainModel(struct ModelCollection *modelCollection, uint8_t modelID);
bool runModel(struct ModelInstance *modelInstance, bool *passFail, float *likelihood);

bool inEnsembleRange(int ModelNumber);
bool inLLModelRange(int ModelNumber);
void nullEnsemble(struct ModelCollection *modelCollection);
void nullModelInstance(struct ModelInstance *modelInstance);
void nullModelCollection(struct ModelCollection *modelCollection);
bool deleteModel(uint8_t ModelNumber);

#endif /* MACHINELEARNING_SUBSYSTEM_H_ */
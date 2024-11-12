


#include <math.h>
#include <stdlib.h>
#include "anomaly_detection.h"


#define         TINY            1.0e-20     
#define         DYNAMIC_RANGE   10


void normalizeMixProb(struct GmmModel *gmmModel);
void ComputeCnst(struct GaussComponent *gaussComponent, uint8_t nDimensions, uint8_t nComponent);
bool computeInvMatDet(struct GaussComponent *gaussComponent, uint8_t nDim, float *det);
float computeMDL_GMM(struct ModelInstance *modelInstance);
float computeLL_regroup(struct ModelInstance *modelInstance);
float loglike(float *x, struct GaussComponent *gaussComponent, uint8_t nDimensions);
void reestimate(struct ModelInstance *modelInstance);
void reduceModelOrder(struct ModelInstance *modelInstance);
float distance(struct GaussComponent *gaussComponent1, struct GaussComponent *gaussComponent2, uint8_t nDimensions);
void addGaussComp(struct GaussComponent *gaussComponent1, struct GaussComponent *gaussComponent2, struct GaussComponent *gaussComponent3, uint8_t nDimensions);
void copyGaussComp(struct GaussComponent *gaussComponent1, struct GaussComponent *gaussComponent2, uint8_t nDimensions);



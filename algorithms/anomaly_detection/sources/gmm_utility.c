/** @file gmm_util.c
*   @brief Utitlity functions for Gaussian Mixture Model (GMM).
*
*       This contatins utitlity functions for GMM. The implmentation is paritally based on the paper 
*       by C. A. Bouman "Cluster: An unsupervised algorithm for modeling Gaussian mixtures" 1997 (available from http://engineering.purdue.edu/\string~bouman).
*       The source code has been changed a lot, targetting to make completely new one for the embedded sensor system.
*/

#include <stdio.h>
#include <math.h>
#include "anomaly_detection.h"
#include "magnetic.h"
#include "drivers.h"
#include "sensor_drv.h"
#include "status.h"
#include "control.h"
#include "fusion.h"
#include "fsl_debug_console.h"
#include "gmm_utility.h"


/** @brief Computes minimum discrition length (MDL) criterion of GMM
*
*       This function computes MDL criterion of GMM with the given number of Gaussian components. 
*       It repeatedly estimates the parameters of GMM, until it satisfies a stopping criterion (epsilon). 
*       Once it reaches the criteria, it computes MDL and returns it. 
*
*       @param  modelInstance   The pointer of ModelInstance structure.
*       @return mdl / 0         float mdl if success, otherwise 0.
*/
float computeMDL_GMM(struct ModelInstance *modelInstance)
{
    float Nparam;
    float L;
    float NM;
    bool repeat;
    float mdl;
    float change,ll_new,ll_old;
    float epsilon;
    uint8_t dim = modelInstance->feature_dimension;
    int8_t nComp = modelInstance->pModel->gmm_model.nComponents;

    /* compute epsilon which is a stopping criterion */
    Nparam = 1 + dim + 0.5 * (dim+1) * dim;
    NM = modelInstance->featureBufferSize * dim;
    epsilon = Nparam * log(NM);
    epsilon = epsilon*0.005;            // stopping cretria that affects EM resolution and training time.
    /* compute log-likelihood of the current GMM parameters */
    ll_new = computeLL_regroup(modelInstance);
    /* perform EM algorithm */
    change = epsilon;
    do {
       ll_old = ll_new;
       reestimate(modelInstance);
       ll_new = computeLL_regroup(modelInstance);
       change = ll_new - ll_old;
       repeat = change > epsilon;
     } while(repeat);
     /* compute MDL */
     if(nComp>0) {
        L = nComp * Nparam - 1;
        mdl = -ll_new + 0.5*L*log(NM);
        return(mdl);
     }
     else {
        return((float)0);
     }
}

/** @brief      Computes log-likelihood of the estimated parameters. 
*
*       This function computes the log-likelihood of currently estimated parameters of mixture model.
*
*       @param modelInstance    The pointer of ModelInstance structure.
*       @return likelihood      The likelihood computed based on GMM parameter set and feature samples.
*/
float computeLL_regroup(struct ModelInstance *modelInstance)
{
    uint16_t s,r;
    uint8_t i,j;
    float tmp;
    float maxlike;
    float likelihood;
    float subsum;
    struct FeatureInstance *featureInstance[MAX_FEATURE_DIMENSION];
    float x[MAX_FEATURE_DIMENSION];
    struct GmmModel *gmmModel = &modelInstance->pModel->gmm_model;
    int startPos, endPos;
    bool reversed = modelInstance->reversedPosition;
    uint8_t dimension = modelInstance->feature_dimension;
    int8_t nComp = gmmModel->nComponents;
    
    inputPositions(modelInstance, &startPos, &endPos);
    /* compute likelihood */
    for(s=0; s<dimension; s++){
        featureInstance[s] = modelInstance->pFeature[s];
    }
    likelihood = 0;
    for(s=startPos; s<=endPos; s++) {
        if (!reversed) r = s;
        else r = s % MAX_FEATURE_SAMPLES;
        for(i=0; i<nComp; i++) {
            for (j=0; j<dimension; j++){
                if (featureInstance[j]->isNormalized)   x[j] = featureInstance[j]->fBufferNormalized[r];
                else   x[j] = featureInstance[j]->fBuffer[r];
            }
            tmp = loglike(&x[0], &gmmModel->gComponents[i], dimension);
            gmmModel->pProb[r][i] = tmp;
            if(i==0)   maxlike = tmp;
            if(tmp>maxlike)   maxlike = tmp;
        }
        subsum = 0;
        for(i=0; i<nComp; i++) {
            tmp = exp(gmmModel->pProb[r][i]-maxlike) * gmmModel->gComponents[i].pi;
            subsum += tmp;
            gmmModel->pProb[r][i] = tmp;
        }
        likelihood += log(subsum) + maxlike;
        for(i=0; i<nComp; i++)
            gmmModel->pProb[r][i] /= subsum;
    }
   return(likelihood);
}

/** @brief Estimates the GMM parameters
*
*       This function estimates the GMM parameters with the given number of Gaussian components. 
*
*       @param  modelInstance   The pointer of ModelInstance structure.
*       @return void.
*/
void reestimate(struct ModelInstance *modelInstance)
{
     uint8_t i;
     uint16_t s,r;
     int b1,b2;
     float diff1,diff2;
     uint8_t dimension = modelInstance->feature_dimension;
     struct GmmModel *gmmModel = &modelInstance->pModel->gmm_model;
     struct GaussComponent *gaussComponent; // = &gmmModel->gComponents[0];
     int8_t nComp = gmmModel->nComponents;
     int startPos, endPos;
     bool reversed = modelInstance->reversedPosition;
     
     inputPositions(modelInstance, &startPos, &endPos);
     /* Compute N */
     for(i=0; i<nComp; i++) {
         gaussComponent = &gmmModel->gComponents[i];
    	 gaussComponent->N = 0;
         for(s=startPos; s<=endPos; s++) {
              if (!reversed) r = s;
              else r = s % MAX_FEATURE_SAMPLES;
              gaussComponent->N += (float) (gmmModel->pProb[r][i] );
         }
         gaussComponent->pi = gaussComponent->N;
     }
     /* Compute means and variances for each subcluster */
     for(i=0; i<nComp; i++) {
        gaussComponent = &gmmModel->gComponents[i];
        /* Compute mean */
        for(b1=0; b1<dimension; b1++) {
            gaussComponent->muMeans[b1] = 0;
            for(s=startPos; s<=endPos; s++) {
                if (!reversed) r = s;
                else r = s % MAX_FEATURE_SAMPLES;
                if (modelInstance->pFeature[b1]->isNormalized)
                    gaussComponent->muMeans[b1] += gmmModel->pProb[r][i] * modelInstance->pFeature[b1]->fBufferNormalized[r]  ;
                else
                    gaussComponent->muMeans[b1] += gmmModel->pProb[r][i] * modelInstance->pFeature[b1]->fBuffer[r] ;
            }
            gaussComponent->muMeans[b1] /= gaussComponent->N;
        }
        /* Compute R */
        for(b1=0; b1<dimension; b1++) {
            for(b2=b1; b2<dimension; b2++) {
                gaussComponent->rCovariance[b1][b2] = 0;
                for(s=startPos; s<=endPos; s++) {
                    if (!reversed) r = s;
                    else r = s % MAX_FEATURE_SAMPLES;
                    if (modelInstance->pFeature[b1]->isNormalized)
                        diff1 = modelInstance->pFeature[b1]->fBufferNormalized[r] - gaussComponent->muMeans[b1];
                    else
                        diff1 = modelInstance->pFeature[b1]->fBuffer[r] - gaussComponent->muMeans[b1];
                    if (modelInstance->pFeature[b2]->isNormalized)
                        diff2 = modelInstance->pFeature[b2]->fBufferNormalized[r] - gaussComponent->muMeans[b2];
                    else
                        diff2 = modelInstance->pFeature[b2]->fBuffer[r] - gaussComponent->muMeans[b2];
                    gaussComponent->rCovariance[b1][b2] += gmmModel->pProb[r][i]*diff1*diff2;
                }
                gaussComponent->rCovariance[b1][b2] /= gaussComponent->N;
                gaussComponent->rCovariance[b2][b1] = gaussComponent->rCovariance[b1][b2];
            }
        }
        /* Regularize the matrix */
        for(b1=0; b1<dimension; b1++) {
            gaussComponent->rCovariance[b1][b1] += gmmModel->Rmin;
        }
     }
     /* Normalize the mixing probabilities */
     normalizeMixProb(gmmModel);
     /* inverse matrix and compute cnst*/
     ComputeCnst(&gmmModel->gComponents[0], dimension, nComp);
}

/** @brief Reduces model order.
*
*       This function reduces the order of GMM (i.e., the number of Gaussian components). This is needed when we want to find the best number of Gaussian components for GMM. 
*
*       @param  modelInstance   The pointer of ModelInstance structure.
*       @return void.
*/
void reduceModelOrder(struct ModelInstance *modelInstance)
{
    uint8_t i,j;
    uint8_t min_i,min_j;
    float dist;
    float min_dist;
    uint8_t dimension = modelInstance->feature_dimension;
    struct GmmModel *gmmModel = &modelInstance->pModel->gmm_model;
    int8_t nComp = gmmModel->nComponents;
    struct GaussComponent *gaussComponent = &gmmModel->gComponents[0];
    struct GaussComponent *gaussComponent1, *gaussComponent2;
    struct GaussComponent gaussComponent_local;
    struct GaussComponent *gaussComponent3 = &gaussComponent_local;

    if(nComp>1) {
        /* find the closest subclasses */
        for(i=0; i<nComp-1; i++) {
            for(j=i+1; j<nComp; j++) {
                dist = distance(&gaussComponent[i], &gaussComponent[j], dimension);
                if((i==0)&&(j==1)) {
                    min_dist = dist;
                    min_i = i;
                    min_j = j;
                }
                if(dist<min_dist) {
                    min_dist = dist;
                    min_i = i;
                    min_j = j;
                }
            }
        }
        /* Combine Subclasses */
        gaussComponent1 = &gaussComponent[min_i];
        gaussComponent2 = &gaussComponent[min_j];
        addGaussComp(gaussComponent1, gaussComponent2, gaussComponent3, dimension);
        copyGaussComp(gaussComponent3, gaussComponent1, dimension);
        /* remove extra subclass, actually shift (JL) */
        for(i=min_j; i<nComp-1; i++)
            copyGaussComp(&gaussComponent[i+1], &gaussComponent[i], dimension);

        /* Remove last Subclass */
        gaussComponent[nComp-1].pi = 0;
        /* Rerun compute_constants */
        ComputeCnst(&gaussComponent[0], dimension, nComp);
        normalizeMixProb(gmmModel);
        gmmModel->nComponents = nComp - 1;
    }
}

/** @brief Computes log-likelihood
*
*       This function computes log-likelihood of the estimated GMM parameters with the given number of Gaussian components. 
*
*       @param  x       The pointer of a two(or one)-dimensional feature sample. 
*       @param  gaussComponent  The pointer of GaussComponent structure to be used.
*       @param  dimension       Feature's dimensionality.
*       @return sum     Log-likelihood for a Gaussian component.
*/
float loglike(float *x, struct GaussComponent *gaussComponent, uint8_t dimension)
{
    uint8_t b1,b2;
    float diff1,diff2;
    float sum = 0;
    
    for(b1=0; b1<dimension; b1++) {
        for(b2=0; b2<dimension; b2++) {
            diff1 = x[b1] - gaussComponent->muMeans[b1];
            diff2 = x[b2] - gaussComponent->muMeans[b2];
            sum += diff1*diff2*gaussComponent->rInvCovariance[b1][b2];
        }
    }
    sum = -0.5*sum + gaussComponent->cnst;
    return(sum);
}

/** @brief Computes distance between two Gaussian components
*
*       This function computes distance between two Gaussian components with the given GMM parameter set. 
*       This function is used to evaluate which pair of components is the closest so that the order of GMM can be properly reduced.
*
*       @param  gaussComponent1 The pointer of a Gaussian component.
*       @param  gaussComponent2 The pointer of the other Gaussian component.
*       @param  dimension       Dimensionality of feature sample.
*       @return dist    The estimated distance between gaussComponent1 and gaussComponent2.
*/
float distance(struct GaussComponent *gaussComponent1, struct GaussComponent *gaussComponent2, uint8_t dimension)
{
    float dist;
    struct GaussComponent gaussComponent;
    struct GaussComponent *gaussComponent3 = &gaussComponent;

    /* form gaussComponent3 by adding gaussComponent1 and gaussComponent2 */
    addGaussComp(gaussComponent1, gaussComponent2, gaussComponent3, dimension);
    /* compute the constant for gaussComponent3 */
    ComputeCnst(gaussComponent3, dimension, 1);
    /* compute distance */
    dist = gaussComponent1->N*gaussComponent1->cnst + gaussComponent2->N*gaussComponent2->cnst - gaussComponent3->N*gaussComponent3->cnst;
    return(dist);
}

/** @brief Adds two Gaussian components to single one.
*
*       This function is to add two Gaussian components into a single one. This is used to evaluate which pair of components should be combined.
*       
*       @param  gaussComponent1 The pointer of GaussComponent structure.
*       @param  gaussComponent2 The pointer of GaussComponent structure.
*       @param  gaussComponent3 The pointer of GaussComponent structure. Weighted sum of gaussComponent1 and gaussComponent2 is saved in gaussComponent3.
*       @param  dimension       Dimensionality of feature samples.
*       @return void.
*/
void addGaussComp(struct GaussComponent *gaussComponent1, struct GaussComponent *gaussComponent2, struct GaussComponent *gaussComponent3, uint8_t dimension)
{
    uint8_t b1,b2;
    float wt1,wt2;
    float tmp;

    wt1 = gaussComponent1->N/(gaussComponent1->N + gaussComponent2->N);
    wt2 = 1 - wt1;
    /* compute means */
    for(b1=0; b1<dimension; b1++)
        gaussComponent3->muMeans[b1] = wt1*gaussComponent1->muMeans[b1] + wt2*gaussComponent2->muMeans[b1];

    /* compute covariance */
    for(b1=0; b1<dimension; b1++) {
        for(b2=b1; b2<dimension; b2++) {
            tmp = (gaussComponent3->muMeans[b1]-gaussComponent1->muMeans[b1])*(gaussComponent3->muMeans[b2]-gaussComponent1->muMeans[b2]);
            gaussComponent3->rCovariance[b1][b2] = wt1*(gaussComponent1->rCovariance[b1][b2] + tmp);
            tmp = (gaussComponent3->muMeans[b1]-gaussComponent2->muMeans[b1])*(gaussComponent3->muMeans[b2]-gaussComponent2->muMeans[b2]);
            gaussComponent3->rCovariance[b1][b2] += wt2*(gaussComponent2->rCovariance[b1][b2] + tmp);
            gaussComponent3->rCovariance[b2][b1] = gaussComponent3->rCovariance[b1][b2];
        }
    }
    /* compute pi and N */
    gaussComponent3->pi = gaussComponent1->pi + gaussComponent2->pi;
    gaussComponent3->N = gaussComponent1->N + gaussComponent2->N;
}

/** @brief Copies one Gaussian component to another.
*
*       This function is to copy one Gaussian component to another one. It copies N, pi, cnst, muMean[], rCovarianc[][], and rInvCovariance[][].
*
*       @param  gaussComponent1 The pointer of GaussComponent structure.
*       @param  gaussComponent2 The pointer of GaussComponent structure. gaussComponent1 is copied to gaussComponent2.
*       @param  dimension       Dimensionality of feature samples.
*       @return void.
*/
void copyGaussComp(struct GaussComponent *gaussComponent1, struct GaussComponent *gaussComponent2, uint8_t dimension)
{
    uint8_t b1,b2;

    gaussComponent2->N = gaussComponent1->N;
    gaussComponent2->pi = gaussComponent1->pi;
    gaussComponent2->cnst = gaussComponent1->cnst;
    for(b1=0; b1<dimension; b1++)
        gaussComponent2->muMeans[b1] = gaussComponent1->muMeans[b1];

    for(b1=0; b1<dimension; b1++) {
        for(b2=0; b2<dimension; b2++) {
            gaussComponent2->rCovariance[b1][b2] = gaussComponent1->rCovariance[b1][b2];
            gaussComponent2->rInvCovariance[b1][b2] = gaussComponent1->rInvCovariance[b1][b2];
        }
    }
}

/** @brief Normalizes the mixing probabilities.
*
*       This function normalizes the mixing probabilities of Gaussian components with the given order of GMM. 
*
*       @param  gmmModel        The pointer of GmmModel structure.
*       @return void.
*/
void normalizeMixProb(struct GmmModel *gmmModel)
{
    uint8_t i;
    struct GaussComponent *gaussComponent = &gmmModel->gComponents[0];
    int8_t nComp = gmmModel->nComponents;
    float sum = 0.0;

    for(i=0; i<nComp; i++)
    	sum += gaussComponent[i].pi;
    if(sum>0) {
    	for(i=0; i<nComp; i++)    gaussComponent[i].pi /= sum;
    }
    else {
    	for(i=0; i<nComp; i++)    gaussComponent[i].pi  = 0.0;
    }
}

/** @brief Computes the constant
*
*       This function computes the constant that is -log((2PI)^(dimension/2)*det^0.5). This term is just the coefficient of normalized Gaussian function.
*
*       @param  gaussComponent  The pointer of GaussComponent structure.
*       @param  dimension       Dimensionality of feature sample.
*       @param  nComp   The number of Gaussian components.
*       @return void.
*/
void ComputeCnst(struct GaussComponent *gaussComponent, uint8_t dimension, uint8_t nComp)
{
    uint8_t i,j,k;
    float det;

    for (i=0; i<nComp; i++) {
        for (j=0; j<dimension; j++) {
            for (k=0; k<dimension; k++){
                gaussComponent[i].rInvCovariance[j][k] = gaussComponent[i].rCovariance[j][k];
            }
	}
        if (computeInvMatDet(&gaussComponent[i], dimension, &det))
            gaussComponent[i].cnst = (-dimension/2.0)*log(2*PI) - 0.5*log(det);
    }
}

/** @brief Computes the inverse covariance matrix and determinant.
*
*       This function is to compute the inverse covariance matrix and determinant of covariance matrix. Since the implementation considers upto two dimension, the computation is quite simple.
*
*       @param  gaussComponent  The pointer of GaussComponent structure.
*       @param  nDim    Dimensionality of feature sample.
*       @param  det     The output for determinant computed.
*       @return true / false    True if success.
*/
bool computeInvMatDet(struct GaussComponent *gaussComponent, uint8_t nDim, float *det)
{   
    if (nDim == 2) {
        *det = f2x2matrixDetA(gaussComponent->rCovariance);
        if (*det > 0) {
            gaussComponent->rInvCovariance[0][0] = gaussComponent->rCovariance[1][1] / (*det);
            gaussComponent->rInvCovariance[0][1] = -gaussComponent->rCovariance[0][1] / (*det);
            gaussComponent->rInvCovariance[1][0] = -gaussComponent->rCovariance[1][0] / (*det);
            gaussComponent->rInvCovariance[1][1] = gaussComponent->rCovariance[0][0] / (*det);
        } 
        else    return false;
    } 
    else {
        if (gaussComponent->rCovariance[0][0] > 0) {
            *det = gaussComponent->rCovariance[0][0];
            gaussComponent->rInvCovariance[0][0] = 1 / (*det);
        }
        else    return false;
    }
    return true;
}


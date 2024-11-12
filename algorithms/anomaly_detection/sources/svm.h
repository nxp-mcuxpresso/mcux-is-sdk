#ifndef _LIBSVM_H
#define _LIBSVM_H

#define LIBSVM_VERSION 321

#ifdef __cplusplus
extern "C" {
#endif

/** @file       This svm.h file defines SVM related data structures. 
    @brief      SVM related data structures. Mostly taken from LIBSVM
  
  More details of LIBSVM is available here: https://www.csie.ntu.edu.tw/~cjlin/libsvm/
  
  */
extern int libsvm_version;

#define MAX_SVM                                 4               // The number of total SVMs allowed to run.
#define MAX_SVM_SVs                             MAX_FEATURE_SAMPLES     // The maximum size of SVs. 

/// @brief Each svm_node represents an element of multidimensional features. 
///
///     Index starts from 1 upto the dimensionality of feature sample. 
///     If index is set to -1, then the svm_node is the end of the SV.
///     double value contains the value of SV.
struct svm_node
{
	int index;
	double value;
};

/// @brief svm_problem describes a problem with a SVM.
///
///     l denotes the number of training data.
///     y denotes an array that contains the target information. For OC-SVM, only the single label 1 is considered.
///     x denotes an array of pointers. Each pointer indicates a training data vector.
struct svm_problem
{
	int l;
	double *y;
	struct svm_node **x;
};


enum { C_SVC, NU_SVC, ONE_CLASS, EPSILON_SVR, NU_SVR };	/* svm_type */


/// @brief svm_parameter contains the parameters for a SVM.
///
///     OC-SVM parameters are put by parse_svm_param().
struct svm_parameter
{
	int svm_type;
	int kernel_type;
	int degree;	/* for poly */
	double gamma;	/* for poly/rbf/sigmoid */
	double coef0;	/* for poly/sigmoid */

	/* these are for training only */
	double cache_size; /* in MB */
	double eps;	/* stopping criteria */
	double C;	/* for C_SVC, EPSILON_SVR and NU_SVR */
	int nr_weight;		/* for C_SVC */
	int *weight_label;	/* for C_SVC */
	double* weight;		/* for C_SVC */
	double nu;	/* for NU_SVC, ONE_CLASS, and NU_SVR */
	double p;	/* for EPSILON_SVR */
	int shrinking;	/* use the shrinking heuristics */
	int probability; /* do probability estimates */

};

/// @brief smv_model structure stores the model obtained from the training procedure.
struct svm_model
{
	struct svm_parameter param;	/* parameter */
	int nr_class;		/* number of classes, = 2 in regression/one class svm */
	int l;			/* total #SV */
	struct svm_node **SV;		/* SVs (SV[l]) */
	double **sv_coef;	/* coefficients for SVs in decision functions (sv_coef[k-1][l]) */
	double *rho;		/* constants in decision functions (rho[k*(k-1)/2]) */
	double *probA;		/* pariwise probability information */
	double *probB;
	int *sv_indices;        /* sv_indices[0,...,nSV-1] are values in [1,...,num_traning_data] to indicate SVs in the training set */

	/* for classification only */
	int *label;		/* label of each class (label[k]) */
	int *nSV;		/* number of SVs for each class (nSV[k]) */
				/* nSV[0] + nSV[1] + ... + nSV[k-1] = l */
	/* XXX */
	int free_sv;		/* 1 if svm_model is created by svm_load_model*/
				/* 0 if svm_model is created by svm_train */
};


struct svm_model *svm_train(const struct svm_problem *prob, const struct svm_parameter *param);
void svm_free_model_content(struct svm_model* model_ptr);
void svm_free_and_destroy_model(struct svm_model** model_ptr_ptr);


/* Following is the definitions for dynamic memory allocation in FreeRTOS. */
#define Malloc(type,n) (type *)pvPortMalloc((n)*sizeof(type));
#define Free(v) vPortFree(v);

enum { LINEAR, RBF, POLY, SIGMOID, PRECOMPUTED }; /* kernel_type */


#ifdef __cplusplus
}
#endif

#endif /* _LIBSVM_H */

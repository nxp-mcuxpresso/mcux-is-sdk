
#ifndef OUTPUT_STREAM_TYPES_H
#define OUTPUT_STREAM_TYPES_H


void send_model_result(bool passFail, bool trainRun, uint8_t modelNumber, uint16_t featureInterval, float feature1, float feature2);
void send_computed_gmm_model(ModelInstance *modelInstance, uint16_t featureInterval);
void send_computed_svm_model(ModelInstance *modelInstance, uint16_t featureInterval);
void streamFeatures();
bool streamModels(struct ModelCollection *modelCollection, uint8_t modelID);



#endif // OUTPUT_STREAM_TYPES_H

#define MAX_FEATURES 10
extern struct FEATURE_LIST feature_list[MAX_FEATURES];
void initialize_feature_list() ;
bool get_feature_number(uint8_t *idx, sensor_t sensor, axis_t axis, feature_t feature) ;
bool add_feature(uint8_t *idx, sensor_t sensor, axis_t axis, feature_t feature) ;
bool delete_feature_by_index(uint8_t idx) ;
bool delete_feature_by_attributes(sensor_t sensor, axis_t axis, feature_t feature) ;
bool get_feature_attributes(uint8_t idx, sensor_t *sensor, axis_t *axis, feature_t *feature) ;

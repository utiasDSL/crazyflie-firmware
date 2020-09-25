/*
 *
 *  Created on: Jan. 28, 2020
 *      Author: Wenda
 *       Email: wenda.zhao@mail.utoronto.ca
 */

#ifndef _DSL_DNN_H_
#define _DSL_DNN_H_
#include "math.h"
#include "cf_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stabilizer_types.h"   // for quaternion data structure
// normalization range
void getErrMax(float * err_max);
void getErrMin(float * err_min);
// // dnn7
// void getFeatureMax(float uwb_feature_max_tdoa[14]);
// void getFeatureMin(float uwb_feature_min_tdoa[14]);
// dnn 5 (without anchor information)
void getFeatureMax(float uwb_feature_max_tdoa[10]);
void getFeatureMin(float uwb_feature_min_tdoa[10]);

// get quaternion
// typedef struct {
//     quaternion_t anchorQuaternion[8];
// }anchorPose; 
// void getQan(anchorPose * q);
// normalization functions
float scaler_normalize(float x, float x_min, float x_max);
float scaler_denormalize(float x, float x_min, float x_max);

// nn functions
float nn_inference(float* input, int size);
void float_matmul(float* input, int input_size, float* matrix, int matrix_size, float* output, int output_size);
void float_bias_add(float* input, int input_size, float* matrix);
void float_relu(float* input, int input_size);
void zero_tensor(float* tensor, int tensor_size);





#endif /* _DSL_DNN_H_ */
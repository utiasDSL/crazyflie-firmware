/*
 *
 *  Created on: Jan. 28, 2020
 *      Author: wenda
 *       Email: wenda.zhao@mail.utoronto.ca
 */

#include "dsl_dnn.h"
#include "weights_tdoa.h"       // The weights for UWB TDoA
#include "debug.h"

#define LAYER_1_SIZE 30
#define LAYER_2_SIZE 30
#define LAYER_3_SIZE 30
#define LAYER_4_SIZE 1

float layer_1[LAYER_1_SIZE] = {0};
float layer_2[LAYER_2_SIZE] = {0};
float layer_3[LAYER_3_SIZE] = {0};
float layer_4[LAYER_4_SIZE] = {0};

// get the input/output normalization data --> modify w.r.t the DNN
void getErrMax(float *err_max){
    *err_max = 0.9999115;
}
void getErrMin(float *err_min){
    *err_min = -0.9999114;
}

void getFeatureMax(float uwb_feature_max_tdoa[14]){
    float feature_max[14] =  {5.43476342 ,  6.18784673,    1.93577989,   5.43332027 ,   6.18791616,   1.93577989,
                                            252.11815442,  58.58216968,  254.00971492,  61.47142955,
                                            252.79877115,  89.96885131,  252.79877115,  89.96885131  };
    for(int i=0; i<14; i++){
        uwb_feature_max_tdoa[i] = feature_max[i];
    }
}
void getFeatureMin(float uwb_feature_min_tdoa[14]){
    float feature_min[14] = {-6.03388409,   -5.64088289,    -2.80429533,    -6.03388409,    -5.64084422,   -2.80505419,
                                            -260.50806393,  -58.83316539,   -252.52822871,  -58.90730597,
                                            -221.3852966,   23.94899832,   -221.3852966,    23.94988893 };
    for(int i=0; i<14; i++){
        uwb_feature_min_tdoa[i] = feature_min[i];
    }
}
//------------------------------------------------------------------------------- //
// [CHANGE] anchor quaternion 0817

// void getQan(anchorPose * q){
// // anchor orientation with Total Station Survey (or Vicon) 
// anchorPose q_an={
//     .anchorQuaternion = {
//               {timestamp: 1, x: 0.57368192, y:  0.37878907, z: 0.60540023,  w: -0.40110664 },   //0
//               {timestamp: 1, x: 0.70965068, y: -0.23988127, z: 0.62018589,  w:  0.23276886 },   //1
//               {timestamp: 1, x: 0.2946621,  y: -0.61468356, z: 0.2884659,   w:  0.67240289 },   //2
//               {timestamp: 1, x:-0.33070679, y:-0.59588659,  z: -0.37463275, w:  0.6285969  },   //3
//               {timestamp: 1, x: 0.2946621,  y: -0.61468356, z: 0.2884659,   w:  0.67240289 },   //4
//               {timestamp: 1, x:-0.33070679, y: -0.59588659, z: -0.37463275, w:  0.6285969  },   //5
//               {timestamp: 1, x: 0.35706695, y: -0.60343212, z: 0.33397901,  w:  0.62994319 },   //6
//               {timestamp: 1, x: 0.60102553, y: -0.34948114, z: 0.62287006,  w:  0.35869535 },   //7
//             }
// };
//     *q = q_an;
// }
//------------------------------------------------------------------------------- //
// normalization
float scaler_normalize(float x, float x_min, float x_max){
	float min_range = 0.0;  float max_range = 1.0;
	float scale = (max_range-min_range) / (x_max - x_min);
	float x_scaled = scale * x + min_range - x_min *scale;

	return x_scaled;
}

//denormalization
float scaler_denormalize(float x, float x_min, float x_max){
	float min_range = 0.0;  float max_range = 1.0;
	float scale = (max_range-min_range) / (x_max - x_min);
	float x_unscaled = (x + x_min * scale - min_range) / scale;
	if (x_unscaled > x_max){
		x_unscaled = x_max;
	}
	if (x_unscaled < x_min){
		x_unscaled = x_min;
	}
	return x_unscaled;
}


float nn_inference(float* input, int size){

	int weight_size1 = size * LAYER_1_SIZE;
	int weight_size2 = LAYER_1_SIZE * LAYER_2_SIZE;
    int weight_size3 = LAYER_2_SIZE * LAYER_3_SIZE;
    int weight_size4 = LAYER_3_SIZE * LAYER_4_SIZE;

    //layer 1
    float_matmul(input, size, weights_1, weight_size1, layer_1, LAYER_1_SIZE);
    float_bias_add(layer_1, LAYER_1_SIZE, bias_1);
    // activate function
    float_relu(layer_1,LAYER_1_SIZE);
    //layer 2
    float_matmul(layer_1, LAYER_1_SIZE, weights_2, weight_size2, layer_2, LAYER_2_SIZE);
    float_bias_add(layer_2, LAYER_2_SIZE, bias_2);
    // activate function
    float_relu(layer_2,LAYER_2_SIZE);

    //layer 3
    float_matmul(layer_2, LAYER_2_SIZE, weights_3, weight_size3, layer_3, LAYER_3_SIZE);
    float_bias_add(layer_3, LAYER_3_SIZE, bias_3);
    // activate function
    float_relu(layer_3,LAYER_3_SIZE);

    //layer 4 (last layer)
    float_matmul(layer_3, LAYER_3_SIZE, weights_4, weight_size4, layer_4, LAYER_4_SIZE);
    float_bias_add(layer_4, LAYER_4_SIZE, bias_4);

    float output = layer_4[0];

    zero_tensor(layer_1,LAYER_1_SIZE);
    zero_tensor(layer_2,LAYER_2_SIZE);
    zero_tensor(layer_3,LAYER_3_SIZE);
    zero_tensor(layer_4,LAYER_4_SIZE);
    // zero_tensor(layer_5,LAYER_5_SIZE);

    return output;
}

void float_matmul(float* input, int input_size, float* matrix, int matrix_size, float* output, int output_size){
    int i = 0;
    while (i< matrix_size){     // extra safety
        for (int k=0; k< input_size; k++ ){
            for (int j = 0; j < output_size; j++) {
                output[j] +=  matrix[i]*input[k];
                i++;
            }
        }
    }
    return;
}

void float_bias_add(float* input, int input_size, float* matrix){
    for(int i = 0; i<input_size; i++){
        input[i] += matrix[i];
    }
    return;
}

void float_relu(float* input, int input_size){
    for (int i = 0 ; i< input_size; i++){
        if(input[i]<0){
            input[i] = 0;
        }
    }
    return;
}

void zero_tensor(float* tensor, int tensor_size){
    for(int i = 0; i< tensor_size; i++){
        tensor[i] = 0.0;
    }
}

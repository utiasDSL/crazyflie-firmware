/*
 *
 *  Created on: Jan. 28, 2020
 *      Author: wenda
 *       Email: wenda.zhao@mail.utoronto.ca
 */

#include "dsl_dnn.h"
#include "weights_tdoa.h"       // The weights for UWB TDoA
#include "debug.h"

#define LAYER_1_SIZE 50
#define LAYER_2_SIZE 50
#define LAYER_3_SIZE 50
#define LAYER_4_SIZE 1

float layer_1[LAYER_1_SIZE] = {0};
float layer_2[LAYER_2_SIZE] = {0};
float layer_3[LAYER_3_SIZE] = {0};
float layer_4[LAYER_4_SIZE] = {0};

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
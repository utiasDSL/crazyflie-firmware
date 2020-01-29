/*
 *
 *  Created on: Jan. 28, 2020
 *      Author: wenda
 *       Email: wenda.zhao@mail.utoronto.ca
 */

#include "dsl_nn.h"
#include "weights.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"       // for vTaskDelay

#define LAYER_1_SIZE 20
#define LAYER_2_SIZE 20
#define LAYER_3_SIZE 3

float layer_1[LAYER_1_SIZE] = {0};
float layer_2[LAYER_2_SIZE] = {0};
float layer_3[LAYER_3_SIZE] = {0};

float nn_inference(float* input, int size){

	int weight_size1 = size * LAYER_1_SIZE;
	int weight_size2 = LAYER_1_SIZE * LAYER_2_SIZE;
	int output_size = LAYER_2_SIZE * LAYER_3_SIZE;
    //layer 1
    float_matmul(input, size, weights_1, weight_size1, layer_1, LAYER_1_SIZE);
    float_bias_add(layer_1, LAYER_1_SIZE, bias_1);
    // activate function
    float_relu(layer_1,LAYER_1_SIZE);

    float_matmul(layer_1, LAYER_1_SIZE,weights_2, weight_size2, layer_2, LAYER_2_SIZE);

    vTaskDelay(M2T(20));   // why need this?
    float_bias_add(layer_2, LAYER_2_SIZE, bias_2);
    float_relu(layer_2,LAYER_2_SIZE);

    float_matmul(layer_2, LAYER_2_SIZE,weights_3, output_size, layer_3, LAYER_3_SIZE);
    float_bias_add(layer_3, LAYER_3_SIZE, bias_3);

    float output = layer_3[0];

    zero_tensor(layer_1,LAYER_1_SIZE);
    zero_tensor(layer_2,LAYER_2_SIZE);
    zero_tensor(layer_3,LAYER_3_SIZE);

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





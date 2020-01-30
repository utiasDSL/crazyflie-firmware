/*
 *
 *  Created on: Jan. 28, 2020
 *      Author: Wenda
 *       Email: wenda.zhao@mail.utoronto.ca
 */

#ifndef _DSL_NN_H_
#define _DSL_NN_H_

// normalization range
static float uwb_feature_max[6] = {6.21573126, 6.8558558, 1.96787431, 3.6977465, 0.56576387, 0.58581404};
static float uwb_feature_min[6] = {-6.47791386, -6.28997147, -3.37768704, -3.4279357, -0.50756258, -0.51739977};
static float uwb_err_max =  0.19264864;
static float uwb_err_min = -0.69999307;

// normalization functions
float scaler_normalize(float x, float x_min, float x_max);
float scaler_denormalize(float x, float x_min, float x_max);

// nn functions
float nn_inference(float* input, int size);
void float_matmul(float* input, int input_size, float* matrix, int matrix_size, float* output, int output_size);
void float_bias_add(float* input, int input_size, float* matrix);
void float_relu(float* input, int input_size);
void zero_tensor(float* tensor, int tensor_size);





#endif /* _DSL_NN_H_ */

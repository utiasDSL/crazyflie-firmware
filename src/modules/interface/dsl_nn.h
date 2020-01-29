/*
 *
 *  Created on: Jan. 28, 2020
 *      Author: Wenda
 *       Email: wenda.zhao@mail.utoronto.ca
 */

#ifndef _DSL_NN_H_
#define _DSL_NN_H_

float float_inference(float* input, int size);
void float_matmul(float* input, int input_size, float* matrix, int matrix_size, float* output, int output_size);
void float_bias_add(float* input, int input_size, float* matrix);
void float_relu(float* input, int input_size);
void zero_tensor(float* tensor, int tensor_size);





#endif /* _DSL_NN_H_ */

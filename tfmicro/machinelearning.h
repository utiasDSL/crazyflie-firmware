/*
 * machinelearning.h
 *
 *  Created on: Jan. 24, 2020
 *      Author: Wenda
 *      Email : wenda.zhao@mail.utoronto.ca
 */
#include <stdint.h>
#ifndef TFMICRO_MODEL
#define TFMICRO_MODEL nn_model_tflite
#define TFLtest fc_tflite
#endif

// type of model. uint8_t if quantized, usually float if not
#define model_type uint8_t

#ifdef __cplusplus
extern "C" {
#endif

int testDoubleFunction(int x);
int machine_learning_test(int n);

struct CTfLiteModel;    // An opaque type that we'll use as a handle
typedef struct CTfLiteModel CTfLiteModel;
const CTfLiteModel* CTfLiteModel_create();
void CTfLiteModel_destroy(CTfLiteModel*);
int CTfLiteModel_version(CTfLiteModel* v);

struct CTfInterpreter; // An opaque type that we'll use as a handle
typedef struct CTfInterpreter CTfInterpreter;
int CTfInterpreter_create_return_version(const CTfLiteModel*, int);
int CTfLiteModel_dimensions(const CTfLiteModel* c_model, uint8_t* arena, size_t size, int dim);

// Actual inference functions
void CTfInterpreter_simple_fc(const CTfLiteModel* c_model, uint8_t* tensor, int alloc_size, uint8_t* input, int* result);
void CTfInterpreter_simple_conv(const CTfLiteModel*, uint8_t*, size_t, model_type*, size_t, model_type*, size_t);

#ifdef __cplusplus
}
#endif


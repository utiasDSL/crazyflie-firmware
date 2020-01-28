/*
 * machinelearning.cc
 *
 *  Created on: Jan. 24, 2020
 *      Author: Wenda
 *      Email : wenda.zhao@mail.utoronto.ca
 */

#include "tensorflow/lite/experimental/micro/examples/micro_speech/recognize_commands.h"
#include "tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "tensorflow/lite/experimental/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "eprintf.h"
#include "console.h"
#include "machinelearning.h"
#include <stdio.h>
// The neural network we are putting in :)
#include "tfmicro_models.h"

extern "C"{
	/*
	 * TfLiteModel wrapper functions
	 */
	const CTfLiteModel * CTfLiteModel_create() {
		return reinterpret_cast<const CTfLiteModel*>(
			::tflite::GetModel(TFMICRO_MODEL));
	}
	void CTfLiteModel_destroy(CTfLiteModel* v) {
		delete reinterpret_cast<tflite::Model*>(v);
	}

	int CTfLiteModel_version(CTfLiteModel* v) {
		return static_cast<int>(reinterpret_cast<tflite::Model*>(v)->version());
	}

	/**
	 * TF Micro Interpreter wrapper functions
	 */
	int CTfInterpreter_create_return_version(const CTfLiteModel* c_model, int alloc_size) {
		tflite::MicroErrorReporter micro_error_reporter;
		tflite::ErrorReporter* error_reporter = &micro_error_reporter;
		::tflite::ops::micro::AllOpsResolver resolver;

		const tflite::Model* model = reinterpret_cast<const tflite::Model*>(c_model);
		uint8_t tensor_arena[alloc_size];
		tflite::SimpleTensorAllocator tensor_allocator(tensor_arena, alloc_size);
		tflite::MicroInterpreter interpreter(model, resolver, &tensor_allocator, error_reporter);
		TfLiteTensor* model_input = interpreter.input(0);
		return model_input->dims->size;
	}

	int CTfLiteModel_dimensions(const CTfLiteModel* c_model, uint8_t* arena, size_t size, int dim) {
		tflite::MicroErrorReporter micro_error_reporter;
		tflite::ErrorReporter* error_reporter = &micro_error_reporter;
		::tflite::ops::micro::AllOpsResolver resolver;

		const tflite::Model* model = reinterpret_cast<const tflite::Model*>(c_model);
		tflite::SimpleTensorAllocator tensor_allocator(arena, size);
		tflite::MicroInterpreter interpreter(model, resolver, &tensor_allocator, error_reporter);
		TfLiteTensor* model_input = interpreter.input(0);
		return model_input->dims->data[dim];
	}

	/* Actual methods used for inferencing */
	void CTfInterpreter_simple_fc(const CTfLiteModel* c_model,
			uint8_t* tensor, int alloc_size, uint8_t* input, int* result) {
		tflite::MicroErrorReporter micro_error_reporter;
		tflite::ErrorReporter* error_reporter = &micro_error_reporter;

		// Pull in all the operation implementations we need
		::tflite::ops::micro::AllOpsResolver resolver;
		// Load a model using the self-defined wrap function 'reinterpret_cast'
		const tflite::Model* model = reinterpret_cast<const tflite::Model*>(c_model);
		// Create an area of memory to use for input, output, and intermediate arrays
		tflite::SimpleTensorAllocator tensor_allocator(tensor, alloc_size);
		// Build an interpreter to run the model with
		tflite::MicroInterpreter interpreter(model, resolver, &tensor_allocator, error_reporter);
		// Get information about the memory area to use for the model's input
		TfLiteTensor* model_input = interpreter.input(0);
		// ?? send the input as the input features into the nn ??
		memcpy(model_input->data.uint8, input, 50 * sizeof(uint8_t));    // here changed 20 to 50
		// Run the model
		TfLiteStatus invoke_status = interpreter.Invoke();

		if (invoke_status != kTfLiteOk) {
			return;
		}
		// Get the inference results from the nn
		TfLiteTensor* output = interpreter.output(0);

		result = static_cast<int>(output->data.uint8[0]);

//		int NUM_CLASSES = 3;
//		for (int i = 0; i < NUM_CLASSES; i++) {
//
//			result[i] = static_cast<int>(output->data.uint8[i]);
//		}
	}

	void CTfInterpreter_simple_conv(const CTfLiteModel* c_model, uint8_t* tensor, size_t alloc_size,
		model_type* input, size_t input_size, model_type* result, size_t result_size) {

		tflite::MicroErrorReporter micro_error_reporter;
		tflite::ErrorReporter* error_reporter = &micro_error_reporter;
		::tflite::ops::micro::AllOpsResolver resolver;

		const tflite::Model* model = reinterpret_cast<const tflite::Model*>(c_model);
		tflite::SimpleTensorAllocator tensor_allocator(tensor, alloc_size);
		tflite::MicroInterpreter interpreter(model, resolver, &tensor_allocator, error_reporter);
		TfLiteTensor* model_input = interpreter.input(0);

		memcpy(model_input->data.uint8, input, input_size * sizeof(model_type));

		TfLiteStatus invoke_status = interpreter.Invoke();
		if (invoke_status != kTfLiteOk) {
			return;
		}

		TfLiteTensor* output = interpreter.output(0);
		memcpy(result, output->data.uint8, result_size * sizeof(model_type));
	}

}


extern "C" int machine_learning_test(int n) {

	tflite::MicroErrorReporter micro_error_reporter;
	tflite::ErrorReporter* error_reporter = &micro_error_reporter;

	// Map the model into a usable data structure. This doesn't involve any
	// copying or parsing, it's a very lightweight operation.
	// const tflite::Model* model = ::tflite::GetModel(tiny_mnist_model);
	const tflite::Model* model = ::tflite::GetModel(TFLtest);
	//consolePrintf("Loaded in model small_mnist_model.\n");
	//consolePrintf("Model Version: %d\n", model->version());
	//consolePrintf("Model Description: %s\n", model->description());

	if (model->version() != TFLITE_SCHEMA_VERSION) {
		error_reporter->Report(
				"Modeltiny_mnist_model provided is schema version %d not equal "
				"to supported version %d.\n",
				model->version(), TFLITE_SCHEMA_VERSION);
		return 1;
	}

	// This pulls in all the operation implementations we need.
	tflite::ops::micro::AllOpsResolver resolver;

	// Create an area of memory to use for input, output, and intermediate arrays.
	// The size of this will depend on the model you're using, and may need to be
	// determined by experimentation.
	const int tensor_arena_size = 1024 * 10;
	// consolePrintf("Trying to allocate tensor arena of size %d bytes\n", tensor_arena_size);
	return 1;
	for (int i = 0; i < 100000; i++) {}
	uint8_t tensor_arena[tensor_arena_size];
	tflite::SimpleTensorAllocator tensor_allocator(tensor_arena, tensor_arena_size);
	consolePrintf("Allocated the tensors.\n");

	// Build an interpreter to runtiny_mnist_model the model with.
	consolePrintf("Attempting to build the interpreter.\n");
	tflite::MicroInterpreter interpreter(model, resolver, &tensor_allocator, error_reporter);
	consolePrintf("The interpreter has been built.\n");

	TfLiteTensor* model_input = interpreter.input(0);
	// Get information about the memory area to use for the model's input.
	consolePrintf("Dims->size: %d\n", model_input->dims->size);
	consolePrintf("Dims->data[0]: %d\n", model_input->dims->data[0]);
	consolePrintf("Dims->data[1]: %d\n", model_input->dims->data[1]);
	printf("Dims->data[2]: %d\n", model_input->dims->data[2]);
	printf("kTfLiteUInt8 enum value: %d\n", kTfLiteUInt8);
	printf("model_input->type: %d\n", model_input->type);

	// Run the model on the spectrogram input and make sure it succeeds.
	TfLiteStatus invoke_status = interpreter.Invoke();
	if (invoke_status != kTfLiteOk) {
		error_reporter->Report("Invoke failed");
		return 1;
	}

	// The output from the model is a vector containing the scores for each
	// kind of prediction, so figure out what the highest scoring category was.
	TfLiteTensor* output = interpreter.output(0);
	printf("First byte of output: %d", output->data.uint8[0]);

	return 0;
}

extern "C" int testDoubleFunction(int x) {
	return x * 2;
};


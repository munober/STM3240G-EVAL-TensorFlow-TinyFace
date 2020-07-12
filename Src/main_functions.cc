/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/


using namespace std;

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/kernels/internal/tensor_ctypes.h"

#include "main_functions.h"
#include "tinyface.cpp.h"

// extern void tinyface_init();
// extern void tinyface_invoke();
// extern const size_t tinyface_output_size(int idx=0);
// extern const int *tinyface_output_dims(int idx=0);
// extern const int tinyface_output_dims_len(int idx=0);

extern TfLiteTensor *tinyface_input(int idx=0);
extern TfLiteTensor *tinyface_output(int idx=0);

// The name of this function is important for Arduino compatibility.
void setup()
{
    tinyface_init();
}

// The name of this function is important for Arduino compatibility.
float *loop(uint8_t *img, uint32_t size)
{
	for (uint32_t i = 0; i < size; i++)
        tflite::GetTensorData<float>(tinyface_input())[0] = img[i];

    tinyface_invoke();

    auto output_array = &tflite::GetTensorData<float>(tinyface_output())[0];
    return output_array;
}
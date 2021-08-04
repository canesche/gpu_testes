#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <chrono> 

using namespace std;
using namespace chrono;

//#define NUMBER_DRONES 128*128

#include "include/check.h"
#include "include/constants.h"
#include "include/kernel.h"

int main(int argc, char* argv[]) {

	int N_THREADS_PER_BLOCK = 64;
	int N_BLOCKS = 64;

	if (argc > 1) {
		N_THREADS_PER_BLOCK = atoi(argv[1]);
		N_BLOCKS = atoi(argv[2]);
	}
	
	int NUMBER_ROBOTS = N_THREADS_PER_BLOCK * N_BLOCKS;
    
    int idx = tmax / pParTs; 
    const int IDX_SIZE = idx + 2; 

    float data[IDX_SIZE][29];
    float *d_data;

    auto time_begin = high_resolution_clock::now();

    cudaMalloc((void**)&d_data, sizeof(float) * IDX_SIZE * 29);

    cudaMemcpy(d_data, data, sizeof(float) * IDX_SIZE * 29, cudaMemcpyHostToDevice);

    setup_coef_constant();

    //const int N_THREADS_PER_BLOCK = 128;
    //const int N_BLOCKS = 128; //IDX_SIZE % N_THREADS_PER_BLOCK == 0 ? IDX_SIZE / N_THREADS_PER_BLOCK : IDX_SIZE / N_THREADS_PER_BLOCK + 1;

    simulation<<<N_BLOCKS, N_THREADS_PER_BLOCK>>>(d_data, IDX_SIZE, NUMBER_ROBOTS);
    cudaDeviceSynchronize();

    cudaMemcpy(data, d_data, sizeof(float) * IDX_SIZE * 29, cudaMemcpyDeviceToHost);

    auto time_end = high_resolution_clock::now();
    duration<double, std::milli> ms_double = time_end - time_begin;

    printf("%lf, ", ms_double.count()/1000.0);
    
    return 0;
}

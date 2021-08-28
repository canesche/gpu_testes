#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>

using namespace std;

#define N_THREADS_PER_BLOCK 256
#define N_BLOCKS 256
#define NUMBER_DRONES N_BLOCKS*N_THREADS_PER_BLOCK      

#include "include/check.h"
#include "include/constants.h"
#include "include/kernel.h"

int main(int argc, char* argv[]) {
    
    int idx = tmax / pParTs; 
    const int IDX_SIZE = idx + 1; 
    
    float **data = new float *[IDX_SIZE]();

    for (int i = 0; i < IDX_SIZE; ++i){
        data[i] = new float[12*NUMBER_DRONES]();
    }

    float *d_data;

    for (int r = 0; r < IDX_SIZE; ++r){
        for (int c = 0; c < 12*NUMBER_DRONES; ++c){
            data[r][c] = 0.0;
        }
    }
    
    cout << "opa\n";
    // # Defina a quantidade de simulações a executar:
    float parametersData[NUMBER_DRONES][3]; // tirar!!!
    float *d_parametersData;

    // # Espaço de Busca dos Parâmetros (com atraso)
    float MAPA[NUMBER_DRONES][3];

    float *d_MAPA;
    int MAPAidx = 0;

    for (int idAtraso = 0; idAtraso < 64; idAtraso++){    // # Amostras atrasadas (de 0 a 1,92 segundos de atraso)
        for (int idxKp = 0; idxKp < 32; idxKp++){         // # Ganho Proporcional
            for (int idxKd = 0; idxKd < 32; idxKd++){     // # Ganho Derivativo

                MAPA[MAPAidx][0] = (float)idAtraso;
                MAPA[MAPAidx][1] = (float)(idxKp*0.75/31.0);
                MAPA[MAPAidx][2] = (float)(idxKd*1.0/31.0);
                MAPAidx += 1;
            }
        }
    }

    cout << "opa2" << endl;

    CHECK(cudaMalloc((void**)&d_data, sizeof(float) * IDX_SIZE * 12*NUMBER_DRONES));
    CHECK(cudaMalloc((void**)&d_MAPA, sizeof(float) * NUMBER_DRONES * 3));
    CHECK(cudaMalloc((void**)&d_parametersData, sizeof(float) * NUMBER_DRONES * 3));

    cout << "opa23" << endl;

    CHECK(cudaMemcpy(d_data, data, sizeof(float) * IDX_SIZE * 12*NUMBER_DRONES, cudaMemcpyHostToDevice));
    
    cout << "opa231" << endl;
    
    CHECK(cudaMemcpy(d_MAPA, MAPA, sizeof(float) * NUMBER_DRONES * 3, cudaMemcpyHostToDevice));
    
    cout << "opa232" << endl;
    
    CHECK(cudaMemcpy(d_parametersData, parametersData, sizeof(float) * NUMBER_DRONES * 3, cudaMemcpyHostToDevice));
    
    cout << "opa233" << endl;

    setup_coef_constant();

    cout << "opa3" << endl;

    // # Kernel Call *********************
    simulation<<<N_BLOCKS, N_THREADS_PER_BLOCK>>>(d_data, IDX_SIZE, d_MAPA, d_parametersData);
    // # *********************************

    CHECK(cudaDeviceSynchronize());

    CHECK(cudaMemcpy(data, d_data, sizeof(float) * IDX_SIZE * 12*NUMBER_DRONES, cudaMemcpyDeviceToHost));
    CHECK(cudaMemcpy(parametersData, d_parametersData, sizeof(float) * NUMBER_DRONES * 3, cudaMemcpyDeviceToHost));

    //cout << "data[1800][0] = " << data[1800][0] << endl;
    //cout << "data[1800][11] = " << data[1800][11] << endl;

    // --- --- Data Handling --- --- 

    /*
    ofstream arq;
    arq.open("include/historicSimGPU.csv");
    
    // Data Param 
    int rows  = IDX_SIZE;
    int cols  = 12*NUMBER_DRONES;

    for (int row = 0; row < rows; row ++) {
        for (int col = 0; col < cols; col ++){
            arq << (data[row][col]);
            arq << (col < cols-1 ? ",":"");
        }
        arq << "\n";
    }

    arq.close();

    cout << "criei o arquivo data.csv" << endl;
     */
    /*
    ofstream arqPar;
    arqPar.open("include/historicSimParamGPU.csv");
    
    int rowsP  = NUMBER_DRONES;
    int colsP  = 3;
    cout << rowsP << endl;
    cout << colsP << endl;
    for (int rowP = 0; rowP < rowsP; rowP ++) {
        for (int colP = 0; colP < colsP; colP ++){
            arqPar << (parametersData[rowP][colP]);
            arqPar << (colP < colsP-1 ? ",":"");
        }
        arqPar << "\n";
    }

    arqPar.close();
    */


    // free device global memory
    CHECK(cudaFree(d_data));

    // free host memory
    //free(data);
    
    cout << "cheguei no final" << endl;
    return 0;
}
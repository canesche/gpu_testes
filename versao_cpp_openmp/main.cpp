
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <chrono> 
#include <omp.h>

using namespace std;
using namespace chrono;

#define tmax 60.0

#include "include/ArDrone.h"
#include "include/iControlVariables.h"
#include "include/iParameters.h"
#include "include/rGetSensorData.h"
#include "include/rSendControlSignals.h"
#include "include/cNearHoverController.h"
#include "include/dataAnalysis.h"

int main(int argc, char* argv[]) {
    
    clock_t tSimu;
    tSimu = clock();       // Time execution

    int threads = omp_get_max_threads();

    cout << threads << endl;

    auto time_begin = high_resolution_clock::now();
    
    ArDrone A[threads]; 
    // # ArDrone d_A;   

    //#====================================== Inicalizando as Variáveis do Robô
    initControlVar(A, threads);
    initParameters(A, threads);

    //#======================================== Aquisição de Dados dos Sensores
    //rGetSensorData(A);

    //#============================================= Simulação
    int idx = tmax / A[0].pParTs; 
    // #    printf("\nelementos %d\n", idx);
    const int IDX_SIZE = idx + 2;      
    
    float data[IDX_SIZE][29];
    
    // #float **data = new float*[IDX_SIZE];
    // #for (int i = 0; i < IDX_SIZE; ++i){
    // #  data[i] = new float[29];
    // #}

    int numel = sizeof(data);
    //printf("\nelementos %d", numel);
    int rows  = sizeof(data)/sizeof(data[0]);
    //printf("\nlinhas %d", rows);
    int cols  = sizeof(data[0])/sizeof(data[0][0]);
    //printf("\ncolunas %d\n", cols);

    float *t = new float[IDX_SIZE*threads]; // t = 0:1/30:tmax; size(t,2);

    //printf("SIZE IDX: %d\n", IDX_SIZE);
	
	#pragma omp parallel for    
    for (int k = 0; k < threads; ++k) {
		for (int c = 0; c < IDX_SIZE; ++c) {    
			int cc = c + k*IDX_SIZE;
		    t[cc] = (float)(c*1.0/30);

		    if (t[cc] > 0.75*tmax){
		        A[k].pPosXd[0] = 0.0;
		        A[k].pPosXd[1] = 0.0;
		        A[k].pPosXd[2] = 1.5;
		        A[k].pPosXd[5] = 0.25*M_PI;
		    } else if (t[cc] > 0.5*tmax){
		        A[k].pPosXd[0] = -1.0;
		        A[k].pPosXd[1] =  0.0;
		        A[k].pPosXd[2] =  2.0;
		        A[k].pPosXd[5] =  0.0;            
		    } else if (t[cc] > 0.25*tmax){
		        A[k].pPosXd[0] = 1.0;
		        A[k].pPosXd[1] = 1.0;
		        A[k].pPosXd[2] = 1.0;
		        A[k].pPosXd[5] = 0.0; 
		    } else {
		        A[k].pPosXd[0] =  2.0;
		        A[k].pPosXd[1] = -1.0;
		        A[k].pPosXd[2] =  1.0;
		        A[k].pPosXd[5] =  0.25*M_PI;            
		    }
		    

		    //# getting robot data
		    //rGetSensorData(&A); //# !!!tenho uma duvida sobre essa função

		    //# Controlador:
		    cNearHoverController(&A[k]);
	 
		    //# Armazenando dados de simulação
		    // XX(:,c) = [A.pPosXd; A.pPosX; A.pSCU; t];
		    saveData(data, &A[k], t[cc], c);
		    
		    //# rSendControlSignals(&A);
		    rSendControlSignals(&A[k]);
		}
    }
    

    // End of the execution
    tSimu = clock() - tSimu;

    printf("%lf ", ((float)tSimu) / CLOCKS_PER_SEC);
    //printf("No. of clocks %ld clocks (%f seconds).\n", tSimu, ((float)tSimu) / CLOCKS_PER_SEC);

    //auto time_end = high_resolution_clock::now();
    //duration<double, std::milli> ms_double = time_end - time_begin;

    //printf("Time using Chrono: %lf ms\n", ms_double.count());
    
    
    /* ----> Salvando a Estrutura em um Arquivo .CSV <---- */    
    
    /*
    char* fileName = "historicSim.csv";

    printf("\nCreating %s file",fileName);
 
    FILE *hist;

    hist = fopen(fileName, "w+");       //create a file
    if (hist == NULL){
        printf("Error while opening the file.\n");
        return 0;
    }

    // Data Param 
    printf("\nelementos %d", numel);
    printf("\nlinhas %d", rows);
    printf("\ncolunas %d\n", cols);

    for (int row = 0; row < rows; row ++) {
        for (int col = 0; col < cols; col ++) {
            fprintf(hist, "%.10f%s", data[row][col], (col < cols-1 ? ",":""));
        }
        fprintf(hist,"\n");
    }

    fclose(hist);*/

    return 0;
}

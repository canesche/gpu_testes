
#include "utils.h"
#include "ginv.h"

__global__ void simulation(float *data, int IDX_SIZE, int NUMBER_DRONES) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx < NUMBER_DRONES) {
      #include "declaration_variable.h"
      #include "initControlVar.h"
      //#include "/content/rGetSensorData.h"
      
      float t;
      
      #pragma unroll
      for (int c = 0; c < IDX_SIZE; ++c) {    // # for (int c = 0; c < IDX_SIZE; ++c) {    
        t = c * 0.03333f;

        if (t > 0.75 * tmax){
            pPosXd[0] = 0.0;
            pPosXd[1] = 0.0;
            pPosXd[2] = 1.5;
            pPosXd[5] = 0.25 * M_PI;
        } else if (t > 0.5 * tmax){
            pPosXd[0] = -1.0;
            pPosXd[1] =  0.0;
            pPosXd[2] =  2.0;
            pPosXd[5] =  0.0;            
        } else if (t > 0.25 * tmax){
            pPosXd[0] = 1.0;
            pPosXd[1] = 1.0;
            pPosXd[2] = 1.0;
            pPosXd[5] = 0.0; 
        } else {
            pPosXd[0] =  2.0;
            pPosXd[1] = -1.0;
            pPosXd[2] =  1.0;
            pPosXd[5] =  0.25 * M_PI;            
        }

        //# getting robot data
        //#include "/content/rGetSensorData.h"

        //# Controlador:
        #include "cNearHoverController.h"

        
        //# rSendControlSignals(&A);
        #include "rSendControlSignals.h"

        //# Armazenando dados de simulação
        #include "savedata.h"
      }
    }
}

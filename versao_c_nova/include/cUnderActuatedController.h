// Controle Dinâmico do UAV

#include <stdlib.h>
#include <stdio.h>
#include <math.h>


void cUnderActuatedController(ArDrone *drone){
    
    // Declaração de variáveis auxiliares:
    float acc_des = 0.0;
    float K1 = 0.5;    // % Ganho derivativo (Kd)
    float K2 = 1.0;           
    float K3 = 0.5;    // % Ganho Proporcional (Kp)
    float K4 = 1.0;
    float ddX[3] = {0.0, 0.0, 0.0};


    // % % ---------------------------------------------------------
    for (int ii = 0; ii < 12; ++ii ) {
        drone->pPosXda[ii] = drone->pPosXd[ii];                        // \in \mathbb{R}^{12 \times 1}
        drone->pPosXtil[ii] = drone->pPosXd[ii] - drone->pPosX[ii];    // % Calculando erro de posição
    }

    for (int ii = 3; ii < 6; ++ii){
        if (fabs(drone->pPosXtil[ii]) > M_PI) {
           drone->pPosXtil[ii] += -2.0*M_PI;     // + pPosXtil[ii];
        }
        if (drone->pPosXtil[ii] < -M_PI) {
            drone->pPosXtil[ii] += 2.0*M_PI;     // + pPosXtil[ii];
        }
    }
    
    // % 1: Control Law Strategy
    for (int ii = 0; ii < 3; ++ii ) {
        ddX[ii] = acc_des + K3*tanh(K4*(drone->pPosXd[ii] - drone->pPosX[ii])) + K1*tanh(K2*(drone->pPosXd[ii+6] - drone->pPosX[ii+6]));
    }

    // % 2: ddXr -> U
    drone->pSCUd[0] = -ddX[1]/drone->pParuSat[0];
    drone->pSCUd[1] = -ddX[0]/drone->pParuSat[1];
    drone->pSCUd[2] =  ddX[2]/drone->pParuSat[2];
    drone->pSCUd[3] =  0.0000/drone->pParuSat[3];
    
    // Tratamento dos Sinais de Controle:
    drone->pSCUd[0] = tanh(drone->pSCUd[0]);
    drone->pSCUd[1] = tanh(drone->pSCUd[1]);
    drone->pSCUd[2] = tanh(drone->pSCUd[2]);
    drone->pSCUd[3] = tanh(drone->pSCUd[3]);
    

    // --- 
  
}

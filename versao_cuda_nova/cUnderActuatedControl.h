// Controle Dinâmico do UAV

    
// Declaração de variáveis auxiliares:
float ddX[3] = {0.0, 0.0, 0.0};
float acc_des = 0.00; // 0.85

// % % ---------------------------------------------------------
#pragma unroll
for (int ii = 0; ii < 12; ++ii ) {
    pPosXda[ii] = pPosXd[ii];                        // \in \mathbb{R}^{12 \times 1}
    pPosXtil[ii] = pPosXd[ii] - Xa[ii];    // % Calculando erro de posição
}

#pragma unroll
for (int ii = 3; ii < 6; ++ii){
    if (fabs(pPosXtil[ii]) > M_PI) {
        pPosXtil[ii] += -2.0*M_PI;     // + pPosXtil[ii];
    }
    if (pPosXtil[ii] < -M_PI) {
        pPosXtil[ii] += 2.0*M_PI;     // + pPosXtil[ii];
    }
}

// % 1: Control Law Strategy
#pragma unroll
for (int ii = 0; ii < 3; ++ii ) {
    ddX[ii] = acc_des + Kp*tanh(pPosXtil[ii]) + Kd*tanh(pPosXtil[ii+6]);
}


// % 2: ddXr -> U
pSCUd[0] = -ddX[1]/pParuSat[0];
pSCUd[1] = -ddX[0]/pParuSat[1];
pSCUd[2] =  ddX[2]/pParuSat[2];
pSCUd[3] =  0.0000/pParuSat[3];

// Tratamento dos Sinais de Controle:
pSCUd[0] = tanh(pSCUd[0]);
pSCUd[1] = tanh(pSCUd[1]);
pSCUd[2] = tanh(pSCUd[2]);
pSCUd[3] = tanh(pSCUd[3]);


// --- 
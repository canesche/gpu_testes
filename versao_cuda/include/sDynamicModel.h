
// # --- Limpar e organizar esta seção quando terminar de arrumar os erros!
    
//# Declaração de variáveis auxiliares:
//#float d0, d1, d2;
//#float a, b_a;
float uphi, utheta, udz, udpsi;
float pParV[4], pParF[4];
float Rx[9], Ry[9], Rz[9], Rd[9];
float Mrd[9], G[3];
//# Mt[9], R[9], , 
float ft[3];
float Cr[9];
float Att[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}; // \in \mathbb{R}^{3x4}
float Ard[12], T[3];
float auxR[9];        // # Aqui estava errado: float auxR[3]; mas é [9] 
float auxft[12];
float auxpParF[3];
float Mt_inv[9];
float auxT[3];
float auxCr[3];
float auxpPosX[3];
float auxT_Cr[3];
float auxMt[3];
float auxMult[3];
float auxMrTCr[3];

float auxW, auxW2, auxW3;
float auxDiv1;
float Mr_inv[9];

//# 1: Receive input signal
//#     pitch          | [-1,1] <==> [-15,15] degrees
//#     roll           | [-1,1] <==> [-15,15] degrees
//#     altitude rate  | [-1,1] <==> [-1,1] m/s
//#     yaw rate       | [-1,1] <==> [-100,100] degrees/s

#pragma unroll
for (int ii = 0; ii < 12; ++ii ) {
    pParXra[ii] = pParXr[ii];
}


pParXr[3]  =  pSCUd[0] * pParuSat[0];
pParXr[4]  = -1.0 * pSCUd[1] * pParuSat[1];
pParXr[8]  =  pSCUd[2] * pParuSat[2];
pParXr[11] = -1.0 * pSCUd[3] * pParuSat[3];


//# Receive the reference errors and compute the forces to be applied to the rigid body
//# 2: Error -> Voltage
auxDiv1 = 1.0/pParTs;

uphi   = pParkdp*(pParXr[3] -pPosX[3]  - pParXra[3] + pPosXa[3] )*auxDiv1  + pParkpp*(pParXr[3] -pPosX[3] );
utheta = pParkdt*(pParXr[4] -pPosX[4]  - pParXra[4] + pPosXa[4] )*auxDiv1  + pParkpt*(pParXr[4] -pPosX[4] ); 
udz    = pParkdz*(pParXr[8] -pPosX[8]  - pParXra[8] + pPosXa[8] )*auxDiv1  + pParkpz*(pParXr[8] -pPosX[8] );
udpsi  = pParkds*(pParXr[11]-pPosX[11] - pParXra[11]+ pPosXa[11])*auxDiv1  + pParkps*(pParXr[11]-pPosX[11]);


pParV[0] = pParVo + (11.1-pParVo)*(  0.15*tanh(uphi) - 0.15*tanh(utheta) + 0.4*tanh(udz) + 0.3*tanh(udpsi));
pParV[1] = pParVo + (11.1-pParVo)*(  0.15*tanh(uphi) + 0.15*tanh(utheta) + 0.4*tanh(udz) - 0.3*tanh(udpsi));
pParV[2] = pParVo + (11.1-pParVo)*(- 0.15*tanh(uphi) + 0.15*tanh(utheta) + 0.4*tanh(udz) + 0.3*tanh(udpsi));
pParV[3] = pParVo + (11.1-pParVo)*(- 0.15*tanh(uphi) - 0.15*tanh(utheta) + 0.4*tanh(udz) - 0.3*tanh(udpsi));


//# 2: V -> W
//# Motor dynamic model: 4 times faster than ArDrone dynamic model 

auxW = 1.0 / ( pParJm+ pParTsm * ( pParBm + pParKm * pParKb / pParR));
auxW2 =  pParKm / pParR;
auxW3 =  pParCt / pParr;

#pragma unroll
for (int iii = 0; iii < 4; ++iii) {         // # Este era o erro????
    for (int ii = 0; ii < 4; ++ii) {
        pParW[ii] = auxW * (pParJm*pParW[ii] + pParTsm*(auxW2*pParV[ii] - auxW3*pParW[ii]*pParW[ii]));
    }
}



//# 3: W -> F
//# Shifting Past Values
#pragma unroll
for (int ii = 0; ii < 4; ++ii) {           // # Este era o erro????(^2)
    pParF[ii]  = pParCf*pParW[ii]*pParW[ii];
}


//# Euler-Lagrange Model
#pragma unroll
for (int ii = 0; ii < 12; ++ii) {
    pPosXa[ii] = pPosX[ii];
}


//# Matriz de rotação
//# --- Rx = [1 0 0; 0 cos(drone->pPosX[3]) -sin(drone->pPosX[3]); 0 sin(drone->pPosX[3]) cos(drone->pPosX[3])];
//# Line 1:
Rx[0] = 1.0;
Rx[1] = 0.0;
Rx[2] = 0.0;

//# Line 2:
Rx[3] = 0.0;
Rx[4] = cos(pPosX[3]);
Rx[5] = -sin(pPosX[3]);
    
//# Line 3:
Rx[6] = 0.0;
Rx[7] = sin(pPosX[3]);
Rx[8] = cos(pPosX[3]);

//# --- Ry = [cos(drone->pPosX[4]) 0 sin(drone->pPosX[4]); 0 1 0; -sin(drone->pPosX[4]) 0 cos(drone->pPosX[4])];
//# Line 1:
Ry[0] = cos(pPosX[4]);
Ry[1] = 0.0;
Ry[2] = sin(pPosX[4]);

//# Line 2:
Ry[3] = 0.0;
Ry[4] = 1.0;
Ry[5] = 0.0;
    
//# Line 3:
Ry[6] = -sin(pPosX[4]);
Ry[7] = 0.0;
Ry[8] = cos(pPosX[4]);

//# --- Rz = [cos(drone->pPosX[5]) -sin(drone->pPosX[5]) 0; sin(drone->pPosX[5]) cos(drone->pPosX[5]) 0; 0 0 1];
//# Line 1:
Rz[0] = cos(pPosX[5]);
Rz[1] = -sin(pPosX[5]);
Rz[2] = 0.0;

//# Line 2:
Rz[3] = sin(pPosX[5]);
Rz[4] = cos(pPosX[5]);
Rz[5] = 0.0;
    
//# Line 3:
Rz[6] = 0.0;
Rz[7] = 0.0;
Rz[8] = 1.0;

//# R = Rz*Ry*Rx;
//# matrixMult(auxR, Rz, Ry, 3, 3, 3);
//# matrixMult(Rd, auxR, Rx, 3, 3, 3);

matrixMultn(auxR, Rz, Ry, 3);

matrixMultn(Rd, auxR, Rx, 3);

//print_matrix(Rd, 3, 3);

//# Translational Inertia Matrix
// #pragma unroll
// for (int ii = 0; ii < 9; ++ii) { 
//    Mt[ii] = pParm * (float) iv0[ii];
//}
    
//# Gravity matrix
G[0] = 0.0;
G[1] = 0.0;
G[2] = pParm * pParg;

//# Disturbance vector
matrixMult(auxft, Rd, Att, 3, 4, 3);
matrixMult(auxpParF, auxft, pParF, 3, 1, 4);

ft[0] = auxpParF[0] - pParD[0];
ft[1] = auxpParF[1] - pParD[1];
ft[2] = auxpParF[2] - pParD[2];

//# Numerical integration for Cartesian velocities
Mt_inv[0] = 2.3310;  Mt_inv[1] = 0.0;    Mt_inv[2] = 0.0;
Mt_inv[3] = 0.0;     Mt_inv[4] = 2.3310; Mt_inv[5] = 0.0;
Mt_inv[6] = 0.0;     Mt_inv[7] = 0.0;    Mt_inv[8] = 2.3310;
            
auxMt[0] = ft[0] - G[0];
auxMt[1] = ft[1] - G[1];
auxMt[2] = ft[2] - G[2];

matrixMult(auxMult, Mt_inv, auxMt, 3, 1, 3);

pPosX[6] = auxMult[0]*pParTs + pPosX[6];
pPosX[7] = auxMult[1]*pParTs + pPosX[7];
pPosX[8] = auxMult[2]*pParTs + pPosX[8];     


//# Rotational inertia matrix

//# Line 1:
Mrd[0] =  pParIxx;
Mrd[1] =  pParIxy*cos(pPosX[3]) - pParIxz*sin(pPosX[3]);
Mrd[2] = -pParIxx*sin(pPosX[4]) + pParIxy*sin(pPosX[3])*cos(pPosX[4]) + pParIxz*cos(pPosX[3])*cos(pPosX[4]);

//# Line 2:
Mrd[3] = pParIxy*cos(pPosX[3]) - pParIxz*sin(pPosX[3]);
Mrd[4] = pParIyy*cos(pPosX[3])*cos(pPosX[3]) + pParIzz*sin(pPosX[3])*sin(pPosX[3]) - 2.0*pParIyz*sin(pPosX[3])*cos(pPosX[3]);
Mrd[5] = pParIyy*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4]) - pParIzz*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4]) - pParIxy*cos(pPosX[3])*sin(pPosX[4]) + pParIxz*sin(pPosX[3])*sin(pPosX[4]) + pParIyz*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4]) - pParIyz*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4]);

//# Line 3:
Mrd[6] = -pParIxx*sin(pPosX[4]) + pParIxy*sin(pPosX[3])*cos(pPosX[4]) + pParIxz*cos(pPosX[3])*cos(pPosX[4]);
Mrd[7] =  pParIyy*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4]) - pParIzz*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4]) - pParIxy*cos(pPosX[3])*sin(pPosX[4]) + pParIxz*sin(pPosX[3])*sin(pPosX[4]) + pParIyz*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4]) - pParIyz*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4]);
Mrd[8] =  pParIxx*sin(pPosX[4])*sin(pPosX[4]) + pParIyy*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) + pParIzz*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) - 2*pParIxy*sin(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]) - 2*pParIxz*cos(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]) + 2*pParIyz*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]);

//# Rotational Coriolis matrix
//# Line 1:
Cr[0] = 0.0;
Cr[1] = pPosX[10]*(pParIyy*sin(pPosX[3])*cos(pPosX[4]) - pParIzz*sin(pPosX[3])*cos(pPosX[3]) + pParIyz*cos(pPosX[3])*cos(pPosX[3]) - pParIyz*sin(pPosX[3])*sin(pPosX[3])) + pPosX[11]*(-pParIxx*cos(pPosX[4])*0.5 - pParIyy*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*0.5 + pParIyy*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*0.5 + pParIzz*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*0.5 - pParIzz*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*0.5 - pParIxy*sin(pPosX[3])*sin(pPosX[4]) - pParIxz*cos(pPosX[3])*sin(pPosX[4]) + 2*pParIyz*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4]));
Cr[2] = pPosX[10]*(-pParIxx*cos(pPosX[4])*0.5 - pParIyy*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*0.5 + pParIyy*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*0.5 + pParIzz*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*0.5 - pParIzz*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*0.5 - pParIxy*sin(pPosX[3])*sin(pPosX[4]) - pParIxz*cos(pPosX[3])*sin(pPosX[4]) + 2*pParIyz*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])) + pPosX[11]*(-pParIyy*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) + pParIzz*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) + pParIxy*cos(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]) - pParIxz*sin(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]) - pParIyz*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) + pParIyz*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]));

//# Line 2:
Cr[3] = pPosX[9]*(-pParIxy*sin(pPosX[3]) - pParIxz*cos(pPosX[3])) + pPosX[10]*(-pParIyy*sin(pPosX[3])*cos(pPosX[3]) + pParIzz*sin(pPosX[3])*cos(pPosX[3]) - pParIyz*cos(pPosX[3])*cos(pPosX[3]) + pParIyz*sin(pPosX[3])*sin(pPosX[3])) + pPosX[11]*(pParIxx*cos(pPosX[4])*0.5 + pParIyy*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*0.5 - pParIyy*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*0.5 - pParIzz*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*0.5 + pParIzz*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*0.5 + pParIxy*sin(pPosX[3])*sin(pPosX[4]) + pParIxz*cos(pPosX[3])*sin(pPosX[4]) - 2*pParIyz*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4]));
Cr[4] = pPosX[9]*(-pParIyy*sin(pPosX[3])*cos(pPosX[3]) + pParIzz*sin(pPosX[3])*cos(pPosX[3]) - pParIyz*cos(pPosX[3])*cos(pPosX[3]) + pParIyz*sin(pPosX[3])*sin(pPosX[3]));
Cr[5] = pPosX[9]*(pParIxx*cos(pPosX[4])*0.5 + pParIyy*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*0.5 - pParIyy*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*0.5 - pParIzz*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*0.5 + pParIzz*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*0.5 + pParIxy*sin(pPosX[3])*sin(pPosX[4]) + pParIxz*cos(pPosX[3])*sin(pPosX[4]) - 2*pParIyz*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])) + pPosX[11]*(-pParIxx*sin(pPosX[4])*cos(pPosX[4]) + pParIyy*sin(pPosX[3])*sin(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]) + pParIzz*cos(pPosX[3])*cos(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]) + pParIxy*sin(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) - pParIxy*sin(pPosX[3])*sin(pPosX[4])*sin(pPosX[4]) + pParIxz*cos(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) - pParIxz*cos(pPosX[3])*sin(pPosX[4])*sin(pPosX[4]) + 2*pParIyz*sin(pPosX[3])*cos(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]));

//# Line 3:
Cr[6] = pPosX[9]*(pParIxy*cos(pPosX[3])*cos(pPosX[4]) - pParIxz*sin(pPosX[3])*cos(pPosX[4])) + pPosX[10]*(-pParIxx*cos(pPosX[4])*0.5 + pParIyy*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*0.5 - pParIyy*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*0.5 - pParIzz*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*0.5 + pParIzz*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*0.5 - 2*pParIyz*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])) + pPosX[11]*(pParIyy*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) - pParIzz*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) - pParIxy*cos(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]) + pParIxz*sin(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]) + pParIyz*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) - pParIyz*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]));
Cr[7] = pPosX[9]*(-pParIxx*cos(pPosX[4])*0.5 + pParIyy*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*0.5 - pParIyy*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*0.5 - pParIzz*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*0.5 + pParIzz*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*0.5 - 2*pParIyz*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])) + pPosX[10]*(-pParIyy*sin(pPosX[3])*cos(pPosX[3])*sin(pPosX[4]) + pParIzz*sin(pPosX[3])*cos(pPosX[3])*sin(pPosX[4]) - pParIxy*cos(pPosX[3])*cos(pPosX[4]) + pParIxz*sin(pPosX[3])*cos(pPosX[4]) + pParIyz*sin(pPosX[3])*sin(pPosX[3])*sin(pPosX[4]) - pParIyz*cos(pPosX[3])*cos(pPosX[3])*sin(pPosX[4])) + pPosX[11]*(pParIxx*sin(pPosX[4])*cos(pPosX[4]) - pParIyy*sin(pPosX[3])*sin(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]) - pParIzz*cos(pPosX[3])*cos(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]) - pParIxy*sin(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) + pParIxy*sin(pPosX[3])*sin(pPosX[4])*sin(pPosX[4]) - pParIxz*cos(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) + pParIxz*cos(pPosX[3])*sin(pPosX[4])*sin(pPosX[4]) - 2*pParIyz*sin(pPosX[3])*cos(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]));
Cr[8] = pPosX[9]*(pParIyy*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) - pParIzz*sin(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) - pParIxy*cos(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]) + pParIxz*sin(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]) + pParIyz*cos(pPosX[3])*cos(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) - pParIyz*sin(pPosX[3])*sin(pPosX[3])*cos(pPosX[4])*cos(pPosX[4])) + pPosX[10]*(pParIxx*sin(pPosX[4])*cos(pPosX[4]) - pParIyy*sin(pPosX[3])*sin(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]) - pParIzz*cos(pPosX[3])*cos(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]) - pParIxy*sin(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) + pParIxy*sin(pPosX[3])*sin(pPosX[4])*sin(pPosX[4]) - pParIxz*cos(pPosX[3])*cos(pPosX[4])*cos(pPosX[4]) + pParIxz*cos(pPosX[3])*sin(pPosX[4])*sin(pPosX[4]) - 2*pParIyz*sin(pPosX[3])*cos(pPosX[3])*sin(pPosX[4])*cos(pPosX[4]));

//# ArDrone
Ard[0] =  pPark1;
Ard[1] =  pPark1;
Ard[2] = -pPark1;
Ard[3] = -pPark1;

Ard[4] = -pPark1;
Ard[5] =  pPark1;
Ard[6] =  pPark1;
Ard[7] = -pPark1;

Ard[8]  =  pPark2;
Ard[9]  = -pPark2;
Ard[10] =  pPark2;
Ard[11] = -pPark2;

//# Aerodynamic thrust 
matrixMult(auxT, Ard, pParF, 3, 1, 4);

T[0] = auxT[0] - pParQ[0];
T[1] = auxT[1] - pParQ[1];
T[2] = auxT[2] - pParQ[2];

//# Numerical integration of rotational movement
invMatrix(Mrd, Mr_inv, 3);                                                                     

auxpPosX[0] = pPosX[9];
auxpPosX[1] = pPosX[10];
auxpPosX[2] = pPosX[11];

matrixMult(auxCr, Cr, auxpPosX, 3, 1, 3);

auxT_Cr[0] = (T[0] - auxCr[0])*pParTs;
auxT_Cr[1] = (T[1] - auxCr[1])*pParTs;
auxT_Cr[2] = (T[2] - auxCr[2])*pParTs;

matrixMult(auxMrTCr, Mr_inv, auxT_Cr, 3, 1, 3);

#pragma unroll
for (int i = 9; i < 12; i++){                                     
    pPosX[i] += auxMrTCr[i-9];
}                                                                 

//# ArDrone pose - Numerical integration
#pragma unroll
for (int ii = 0; ii < 6; ii++) {
    pPosX[ii] += pPosX[ii + 6] * pParTs;
    if (ii > 2) {
        if (pPosX[ii] > 3.14159) {     // M_PI
            pPosX[ii] += -6.283185;    // -2*M_PI
        }
        if (pPosX[ii] < -3.14159) {    // -M_PI
            pPosX[ii] += 6.283185;     // 2*M_PI
        }
    }
}
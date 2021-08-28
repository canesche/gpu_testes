
/*
% Dynamic model from
% Brandão, A. S., M. Sarcinelli-Filho, and R. Carelli. 
% "High-level underactuated nonlinear control for rotorcraft machines." 
% Mechatronics (ICM), 2013 IEEE International Conference on. IEEE, 2013.
%
% ArDrone 2.0 Parameters
% Li, Qianying. "Grey-box system identification of a quadrotor unmanned 
% aerial vehicle." Master of Science Thesis Delft University of
% Technology (2014).
%
% Simulate ArDrone dynamic model
%
%      +----------+  W   +--------+  F   +----------+  T   +-------+
% U -> | Actuator |  ->  | Rotary |  ->  | Forces & |  ->  | Rigid |  -> X
%      | Dynamics |      | Wing   |      | Torques  |      | Body  |
%      +----------+      +--------+      +----------+      +-------+
%            1                2                3               4
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

extern void print_matrix(float *v, int m, int n);
extern bool invMatrix(float *A, float *inverse, int n); 

extern void matrixMult(float *hres, float *a, float *b, int n);

extern void matrixMultMNdebug(float *hres, float *a, float *b, int n, int m, int p);  

void sDynamicModel(struct ArDrone *drone){
    
    // Declaração de variáveis auxiliares:
    float auxVar1;
    float d0, d1, d2;
    float a, b_a;
    float uphi, utheta, udz, udpsi;
    float pParV[4], pParF[4];
    float Rx[9], Ry[9], Rz[9], R[9];
    float Mt[9], G[3];
    float ft[3];
    float Mr[9], Cr[9];
    float Ar[12], T[3];
    const float b[16] = { 1.0, 1.0, -1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0,-1.0, 1.0, -1.0 };
    const signed char iv0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 }; // eye(3,3);

    float At[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}; // \in \mathbb{R}^{3x4}
    
    //-------------------------------- end


    // % 1: Receive input signal
    // %     pitch          | [-1,1] <==> [-15,15] degrees
    // %     roll           | [-1,1] <==> [-15,15] degrees
    // %     altitude rate  | [-1,1] <==> [-1,1] m/s
    // %     yaw rate       | [-1,1] <==> [-100,100] degrees/s

    for (int ii = 0; ii < 12; ++ii ) {
        drone->pParXra[ii] = drone->pParXr[ii];
    }

    drone->pParXr[3]  =  drone->pSCUd[0]*drone->pParuSat[0];
    drone->pParXr[4]  = -1.0*drone->pSCUd[1]*drone->pParuSat[1];
    drone->pParXr[8]  =  drone->pSCUd[2]*drone->pParuSat[2];
    drone->pParXr[11] = -1.0*drone->pSCUd[3]*drone->pParuSat[3];
  
    // % Receive the reference errors and compute the forces to be applied to the rigid body

    // % 2: Error -> Voltage
    float auxDiv1 = 1.0/drone->pParTs;

    uphi   = drone->pParkdp*(drone->pParXr[3] -drone->pPosX[3]  - drone->pParXra[3] + drone->pPosXa[3] )*auxDiv1  + drone->pParkpp*(drone->pParXr[3] -drone->pPosX[3] );
    utheta = drone->pParkdt*(drone->pParXr[4] -drone->pPosX[4]  - drone->pParXra[4] + drone->pPosXa[4] )*auxDiv1  + drone->pParkpt*(drone->pParXr[4] -drone->pPosX[4] ); 
    udz    = drone->pParkdz*(drone->pParXr[8] -drone->pPosX[8]  - drone->pParXra[8] + drone->pPosXa[8] )*auxDiv1  + drone->pParkpz*(drone->pParXr[8] -drone->pPosX[8] );
    udpsi  = drone->pParkds*(drone->pParXr[11]-drone->pPosX[11] - drone->pParXra[11]+ drone->pPosXa[11])*auxDiv1  + drone->pParkps*(drone->pParXr[11]-drone->pPosX[11]);

    // ---
    float auxVo     = (11.1-drone->pParVo);
    float th_uPhi   = 0.15*tanh(uphi);
    float th_uTheta = 0.15*tanh(utheta);
    float th_uDz    = 0.40*tanh(udz);
    float th_uPsi   = 0.30*tanh(udpsi);
    pParV[0] = drone->pParVo + (11.1-drone->pParVo)*(  0.15*tanh(uphi) - 0.15*tanh(utheta) + 0.4*tanh(udz) + 0.3*tanh(udpsi));
    pParV[1] = drone->pParVo + (11.1-drone->pParVo)*(  0.15*tanh(uphi) + 0.15*tanh(utheta) + 0.4*tanh(udz) - 0.3*tanh(udpsi));
    pParV[2] = drone->pParVo + (11.1-drone->pParVo)*(- 0.15*tanh(uphi) + 0.15*tanh(utheta) + 0.4*tanh(udz) + 0.3*tanh(udpsi));
    pParV[3] = drone->pParVo + (11.1-drone->pParVo)*(- 0.15*tanh(uphi) - 0.15*tanh(utheta) + 0.4*tanh(udz) - 0.3*tanh(udpsi));


    // % 2: V -> W
    // % Motor dynamic model: 4 times faster than ArDrone dynamic model 

    float auxW = 1.0/(drone->pParJm+drone->pParTsm*(drone->pParBm+drone->pParKm*drone->pParKb/drone->pParR));
    float auxW2 = drone->pParKm/drone->pParR;
    float auxW3 = drone->pParCt/drone->pParr;

    for (int ii = 0; ii < 4; ii++){
        drone->pParW[0] = auxW*(drone->pParJm*drone->pParW[0] + drone->pParTsm*(auxW2*pParV[0] - auxW3*drone->pParW[0]*drone->pParW[0]));
        drone->pParW[1] = auxW*(drone->pParJm*drone->pParW[1] + drone->pParTsm*(auxW2*pParV[1] - auxW3*drone->pParW[1]*drone->pParW[1]));
        drone->pParW[2] = auxW*(drone->pParJm*drone->pParW[2] + drone->pParTsm*(auxW2*pParV[2] - auxW3*drone->pParW[2]*drone->pParW[2]));
        drone->pParW[3] = auxW*(drone->pParJm*drone->pParW[3] + drone->pParTsm*(auxW2*pParV[3] - auxW3*drone->pParW[3]*drone->pParW[3]));
    }

    // % 3: W -> F
    // % Shifting Past Values
    pParF[0]  = drone->pParCf*drone->pParW[0]*drone->pParW[0];
    pParF[1]  = drone->pParCf*drone->pParW[1]*drone->pParW[1];
    pParF[2]  = drone->pParCf*drone->pParW[2]*drone->pParW[2];
    pParF[3]  = drone->pParCf*drone->pParW[3]*drone->pParW[3];

    // % Euler-Lagrange Model
    for (int ii = 0; ii < 12; ++ii ) {
        drone->pPosXa[ii] = drone->pPosX[ii];
    }

    // % Matriz de rotação
    // --- Rx = [1 0 0; 0 cos(drone->pPosX[3]) -sin(drone->pPosX[3]); 0 sin(drone->pPosX[3]) cos(drone->pPosX[3])];
    // Line 1:
    Rx[0] = 1.0;
    Rx[1] = 0.0;
    Rx[2] = 0.0;

    // Line 2:
    Rx[3] = 0.0;
    Rx[4] = cos(drone->pPosX[3]);
    Rx[5] = -sin(drone->pPosX[3]);
    
    // Line 3:
    Rx[6] = 0.0;
    Rx[7] = sin(drone->pPosX[3]);
    Rx[8] = cos(drone->pPosX[3]);


    // --- Ry = [cos(drone->pPosX[4]) 0 sin(drone->pPosX[4]); 0 1 0; -sin(drone->pPosX[4]) 0 cos(drone->pPosX[4])];
    // Line 1:
    Ry[0] = cos(drone->pPosX[4]);
    Ry[1] = 0.0;
    Ry[2] = sin(drone->pPosX[4]);

    // Line 2:
    Ry[3] = 0.0;
    Ry[4] = 1.0;
    Ry[5] = 0.0;
    
    // Line 3:
    Ry[6] = -sin(drone->pPosX[4]);
    Ry[7] = 0.0;
    Ry[8] = cos(drone->pPosX[4]);


    // --- Rz = [cos(drone->pPosX[5]) -sin(drone->pPosX[5]) 0; sin(drone->pPosX[5]) cos(drone->pPosX[5]) 0; 0 0 1];
    // Line 1:
    Rz[0] = cos(drone->pPosX[5]);
    Rz[1] = -sin(drone->pPosX[5]);
    Rz[2] = 0.0;

    // Line 2:
    Rz[3] = sin(drone->pPosX[5]);
    Rz[4] = cos(drone->pPosX[5]);
    Rz[5] = 0.0;
    
    // Line 3:
    Rz[6] = 0.0;
    Rz[7] = 0.0;
    Rz[8] = 1.0;

    // R = Rz*Ry*Rx;
    float  auxR[9]; // auxR[3];

    matrixMult(auxR, Rz, Ry, 3);
    matrixMult(R, auxR, Rx, 3);

    // % =======================================================================
    // % Translational Inertia Matrix

    for (int ii = 0; ii < 9; ii++) {                // % Inertia matrix
        Mt[ii] = drone->pParm * (float)iv0[ii];
    }
    
    // % Gravity matrix
    G[0] = 0.0;
    G[1] = 0.0;
    G[2] = drone->pParm * drone->pParg;

    // % --- ArDrone force matrix 

    // % Disturbance vector
    float auxft[12];
    float auxpParF[3];

    matrixMultMNdebug(auxft, R, At, 3, 4, 3);
    matrixMultMNdebug(auxpParF, auxft, pParF, 3, 1, 4);

    ft[0] = auxpParF[0] - drone->pParD[0];
    ft[1] = auxpParF[1] - drone->pParD[1];
    ft[2] = auxpParF[2] - drone->pParD[2];


    // % Numerical integration for Cartesian velocities
    float* Mt_inv = new float[3*3];

    Mt_inv[0] = 2.3310;  Mt_inv[1] = 0.0;    Mt_inv[2] = 0.0;
    Mt_inv[3] = 0.0;     Mt_inv[4] = 2.3310; Mt_inv[5] = 0.0;
    Mt_inv[6] = 0.0;     Mt_inv[7] = 0.0;    Mt_inv[8] = 2.3310;


    float auxMt[3];            
    auxMt[0] = ft[0] - G[0];
    auxMt[1] = ft[1] - G[1];
    auxMt[2] = ft[2] - G[2];
    
    float auxMult[3];
    matrixMultMNdebug(auxMult, Mt_inv, auxMt, 3, 1, 3);

    drone->pPosX[6] = auxMult[0]*drone->pParTs + drone->pPosX[6];
    drone->pPosX[7] = auxMult[1]*drone->pParTs + drone->pPosX[7];
    drone->pPosX[8] = auxMult[2]*drone->pParTs + drone->pPosX[8];


    // % ======================================================================
    // % Rotational inertia matrix

    // Line 1:
    Mr[0] =  drone->pParIxx;
    Mr[1] =  drone->pParIxy*cos(drone->pPosX[3]) - drone->pParIxz*sin(drone->pPosX[3]);
    Mr[2] = -drone->pParIxx*sin(drone->pPosX[4]) + drone->pParIxy*sin(drone->pPosX[3])*cos(drone->pPosX[4]) + drone->pParIxz*cos(drone->pPosX[3])*cos(drone->pPosX[4]);

    // Line 2:
    Mr[3] = drone->pParIxy*cos(drone->pPosX[3]) - drone->pParIxz*sin(drone->pPosX[3]);
    Mr[4] = drone->pParIyy*cos(drone->pPosX[3])*cos(drone->pPosX[3]) + drone->pParIzz*sin(drone->pPosX[3])*sin(drone->pPosX[3]) - 2.0*drone->pParIyz*sin(drone->pPosX[3])*cos(drone->pPosX[3]);
    Mr[5] = drone->pParIyy*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4]) - drone->pParIzz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4]) - drone->pParIxy*cos(drone->pPosX[3])*sin(drone->pPosX[4]) + drone->pParIxz*sin(drone->pPosX[3])*sin(drone->pPosX[4]) + drone->pParIyz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4]) - drone->pParIyz*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4]);
    
    // Line 3:
    Mr[6] = -drone->pParIxx*sin(drone->pPosX[4]) + drone->pParIxy*sin(drone->pPosX[3])*cos(drone->pPosX[4]) + drone->pParIxz*cos(drone->pPosX[3])*cos(drone->pPosX[4]);
    Mr[7] =  drone->pParIyy*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4]) - drone->pParIzz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4]) - drone->pParIxy*cos(drone->pPosX[3])*sin(drone->pPosX[4]) + drone->pParIxz*sin(drone->pPosX[3])*sin(drone->pPosX[4]) + drone->pParIyz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4]) - drone->pParIyz*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4]);
    Mr[8] =  drone->pParIxx*sin(drone->pPosX[4])*sin(drone->pPosX[4]) + drone->pParIyy*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) + drone->pParIzz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) - 2*drone->pParIxy*sin(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]) - 2*drone->pParIxz*cos(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]) + 2*drone->pParIyz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]);

// # --------------- Parei Aqui!!!
    // % Rotational Coriolis matrix
    // Line 1:
    Cr[0] = 0.0;
    Cr[1] = drone->pPosX[10]*(drone->pParIyy*sin(drone->pPosX[3])*cos(drone->pPosX[4]) - drone->pParIzz*sin(drone->pPosX[3])*cos(drone->pPosX[3]) + drone->pParIyz*cos(drone->pPosX[3])*cos(drone->pPosX[3]) - drone->pParIyz*sin(drone->pPosX[3])*sin(drone->pPosX[3])) + drone->pPosX[11]*(-drone->pParIxx*cos(drone->pPosX[4])*0.5 - drone->pParIyy*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 + drone->pParIyy*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 + drone->pParIzz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 - drone->pParIzz*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 - drone->pParIxy*sin(drone->pPosX[3])*sin(drone->pPosX[4]) - drone->pParIxz*cos(drone->pPosX[3])*sin(drone->pPosX[4]) + 2*drone->pParIyz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4]));
    Cr[2] = drone->pPosX[10]*(-drone->pParIxx*cos(drone->pPosX[4])*0.5 - drone->pParIyy*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 + drone->pParIyy*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 + drone->pParIzz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 - drone->pParIzz*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 - drone->pParIxy*sin(drone->pPosX[3])*sin(drone->pPosX[4]) - drone->pParIxz*cos(drone->pPosX[3])*sin(drone->pPosX[4]) + 2*drone->pParIyz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])) + drone->pPosX[11]*(-drone->pParIyy*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) + drone->pParIzz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) + drone->pParIxy*cos(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIxz*sin(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIyz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) + drone->pParIyz*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]));

    // Line 2:
    Cr[3] = drone->pPosX[9]*(-drone->pParIxy*sin(drone->pPosX[3]) - drone->pParIxz*cos(drone->pPosX[3])) + drone->pPosX[10]*(-drone->pParIyy*sin(drone->pPosX[3])*cos(drone->pPosX[3]) + drone->pParIzz*sin(drone->pPosX[3])*cos(drone->pPosX[3]) - drone->pParIyz*cos(drone->pPosX[3])*cos(drone->pPosX[3]) + drone->pParIyz*sin(drone->pPosX[3])*sin(drone->pPosX[3])) + drone->pPosX[11]*(drone->pParIxx*cos(drone->pPosX[4])*0.5 + drone->pParIyy*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 - drone->pParIyy*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 - drone->pParIzz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 + drone->pParIzz*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 + drone->pParIxy*sin(drone->pPosX[3])*sin(drone->pPosX[4]) + drone->pParIxz*cos(drone->pPosX[3])*sin(drone->pPosX[4]) - 2*drone->pParIyz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4]));
    Cr[4] = drone->pPosX[9]*(-drone->pParIyy*sin(drone->pPosX[3])*cos(drone->pPosX[3]) + drone->pParIzz*sin(drone->pPosX[3])*cos(drone->pPosX[3]) - drone->pParIyz*cos(drone->pPosX[3])*cos(drone->pPosX[3]) + drone->pParIyz*sin(drone->pPosX[3])*sin(drone->pPosX[3]));
    Cr[5] = drone->pPosX[9]*(drone->pParIxx*cos(drone->pPosX[4])*0.5 + drone->pParIyy*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 - drone->pParIyy*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 - drone->pParIzz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 + drone->pParIzz*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 + drone->pParIxy*sin(drone->pPosX[3])*sin(drone->pPosX[4]) + drone->pParIxz*cos(drone->pPosX[3])*sin(drone->pPosX[4]) - 2*drone->pParIyz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])) + drone->pPosX[11]*(-drone->pParIxx*sin(drone->pPosX[4])*cos(drone->pPosX[4]) + drone->pParIyy*sin(drone->pPosX[3])*sin(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]) + drone->pParIzz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]) + drone->pParIxy*sin(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIxy*sin(drone->pPosX[3])*sin(drone->pPosX[4])*sin(drone->pPosX[4]) + drone->pParIxz*cos(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIxz*cos(drone->pPosX[3])*sin(drone->pPosX[4])*sin(drone->pPosX[4]) + 2*drone->pParIyz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]));
    
    // Line 3:
    Cr[6] = drone->pPosX[9]*(drone->pParIxy*cos(drone->pPosX[3])*cos(drone->pPosX[4]) - drone->pParIxz*sin(drone->pPosX[3])*cos(drone->pPosX[4])) + drone->pPosX[10]*(-drone->pParIxx*cos(drone->pPosX[4])*0.5 + drone->pParIyy*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 - drone->pParIyy*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 - drone->pParIzz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 + drone->pParIzz*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 - 2*drone->pParIyz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])) + drone->pPosX[11]*(drone->pParIyy*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIzz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIxy*cos(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]) + drone->pParIxz*sin(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]) + drone->pParIyz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIyz*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]));
    Cr[7] = drone->pPosX[9]*(-drone->pParIxx*cos(drone->pPosX[4])*0.5 + drone->pParIyy*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 - drone->pParIyy*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 - drone->pParIzz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 + drone->pParIzz*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*0.5 - 2*drone->pParIyz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])) + drone->pPosX[10]*(-drone->pParIyy*sin(drone->pPosX[3])*cos(drone->pPosX[3])*sin(drone->pPosX[4]) + drone->pParIzz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*sin(drone->pPosX[4]) - drone->pParIxy*cos(drone->pPosX[3])*cos(drone->pPosX[4]) + drone->pParIxz*sin(drone->pPosX[3])*cos(drone->pPosX[4]) + drone->pParIyz*sin(drone->pPosX[3])*sin(drone->pPosX[3])*sin(drone->pPosX[4]) - drone->pParIyz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*sin(drone->pPosX[4])) + drone->pPosX[11]*(drone->pParIxx*sin(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIyy*sin(drone->pPosX[3])*sin(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIzz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIxy*sin(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) + drone->pParIxy*sin(drone->pPosX[3])*sin(drone->pPosX[4])*sin(drone->pPosX[4]) - drone->pParIxz*cos(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) + drone->pParIxz*cos(drone->pPosX[3])*sin(drone->pPosX[4])*sin(drone->pPosX[4]) - 2*drone->pParIyz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]));
    Cr[8] = drone->pPosX[9]*(drone->pParIyy*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIzz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIxy*cos(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]) + drone->pParIxz*sin(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]) + drone->pParIyz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIyz*sin(drone->pPosX[3])*sin(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4])) + drone->pPosX[10]*(drone->pParIxx*sin(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIyy*sin(drone->pPosX[3])*sin(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIzz*cos(drone->pPosX[3])*cos(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]) - drone->pParIxy*sin(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) + drone->pParIxy*sin(drone->pPosX[3])*sin(drone->pPosX[4])*sin(drone->pPosX[4]) - drone->pParIxz*cos(drone->pPosX[3])*cos(drone->pPosX[4])*cos(drone->pPosX[4]) + drone->pParIxz*cos(drone->pPosX[3])*sin(drone->pPosX[4])*sin(drone->pPosX[4]) - 2*drone->pParIyz*sin(drone->pPosX[3])*cos(drone->pPosX[3])*sin(drone->pPosX[4])*cos(drone->pPosX[4]));


    // % ArDrone
    Ar[0]  =  drone->pPark1;
    Ar[1]  =  drone->pPark1;
    Ar[2]  = -drone->pPark1;
    Ar[3]  = -drone->pPark1;

    Ar[4]  = -drone->pPark1;
    Ar[5]  =  drone->pPark1;
    Ar[6]  =  drone->pPark1;
    Ar[7]  = -drone->pPark1;
    
    Ar[8]  =  drone->pPark2;
    Ar[9]  = -drone->pPark2;
    Ar[10] =  drone->pPark2;
    Ar[11] = -drone->pPark2;


    // % Aerodynamic thrust 
    float auxT[3];

    matrixMultMNdebug(auxT, Ar, pParF, 3, 1, 4);

    T[0] = auxT[0] - drone->pParQ[0];
    T[1] = auxT[1] - drone->pParQ[1];
    T[2] = auxT[2] - drone->pParQ[2];


    // %--------------------------------------------
    // % Numerical integration of rotational movement

    //float Mr_inv[9];
    float* Mr_inv = new float[3*3];
    invMatrix(Mr,Mr_inv,3);                                                                     

    float auxCr[3];
    float auxpPosX[3];

    auxpPosX[0] = drone->pPosX[9];
    auxpPosX[1] = drone->pPosX[10];
    auxpPosX[2] = drone->pPosX[11];

    matrixMultMNdebug(auxCr, Cr, auxpPosX, 3, 1, 3);


    float auxT_Cr[3];
    auxT_Cr[0] = (T[0] - auxCr[0])*drone->pParTs;
    auxT_Cr[1] = (T[1] - auxCr[1])*drone->pParTs;
    auxT_Cr[2] = (T[2] - auxCr[2])*drone->pParTs;


    float auxMrTCr[3];
    matrixMultMNdebug(auxMrTCr, Mr_inv, auxT_Cr, 3, 1, 3);

    for (int i = 9; i < 12; i++){                                     
        drone->pPosX[i] += auxMrTCr[i-9];
    }                                                                 


    // % ArDrone pose - Numerical integration
    for (int ii = 0; ii < 6; ii++) {
        drone->pPosX[ii] += drone->pPosX[ii + 6] * drone->pParTs;
        if (ii > 2) {
            if (drone->pPosX[ii] > 3.14159) {     // M_PI
                drone->pPosX[ii] += - 6.283185;    // -2*M_PI
            }

            if (drone->pPosX[ii] < -3.14159) {    // -M_PI
                drone->pPosX[ii] += + 6.283185;     // 2*M_PI
            }
        }
    }

    delete Mt_inv, Mr_inv;

}
// Controle Dinâmico do UAV

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "importantFunctions.h"
#include "ginv.h"

/* Variable globais Definitions */
const signed char iv0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };  // eye(3,3);
static const signed char b[12] = { -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1 };
//-------------------------------- end

//-------------------------------------------------------- Arco-Tangente (atan2)
/* Function Definitions */
/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_atan2(float u0, float u1){
    float y;
    int b_u0;
    int b_u1;
  
    if (u1 == 0.0) { 
        if (u0 > 0.0) { y = 0.5 * M_PI; } 
        else if (u0 < 0.0) { y = - 0.5 * M_PI; } 
        else { y = 0.0; }
    } 
    else { y = atan2(u0, u1); }

    return y;
}

void cNearHoverController(struct ArDrone *drone){
    
    // Declaração de variáveis auxiliares:
    float Gkx1, Gkx2, Gkx3, Gkx4;
    float Gky1, Gky2, Gky3, Gky4;
    float Gkz1, Gkz2, Gkz3, Gkz4;
    float Gkp1, Gkp2, Gkp3, Gkp4;
    float Gkt1, Gkt2, Gkt3, Gkt4;
    float Gks1, Gks2, Gks3, Gks4;

    float etax, etay, etaz;
    float etap, etat, etas;

    float R[9], Rlinha[9];
    float Mt[9], Gt[3], Mr[9];
    float MM[36], GG[6];
    float At[12], Ar[12], A[24];
    float Map[8], Maa[16], Ea[4], MP[9], GP[3];
    float Wda[4], Mpp[4], Mpa[8], Ep[2];
    float D[16], H[4], eta[4], Fr[4];

    float Vr[4];

    float As[24], Ma[24], Y[8];
    float Fd[4];
    float* fTau = new float [6];


    float auxVar1[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};      // \in \mathbb{R}^{3 \times 4}
    float bb[12] = { -1.0, 1.0, 1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };     // \in \mathbb{R}^{3 \times 4}
    float cgains[12] = {0.5, 2.0, 0.5, 2.0, 5.0, 2.0, 1.0, 20.0, 1.0, 15.0, 1.0, 2.5};     // \in \mathbb{R}^{2 \times 6}

    // ---- Auxiliares Variables:
    float Ass[24], Aaux[24];

    // % Controllers Gains.
    // % The Gains must be given in the folowing order
    // % kx1 kx2 ky1 ky2 kz1 kz2 ; kPhi1 kPhi2 ktheta1 ktheta2 kPsi1 kPsi2
    // % cgains = [.5 2 .5 2 5 2; 1 20 1 15 1 2.5];

    Gkx1 = cgains[0];
    Gkx2 = cgains[1];
    Gkx3 = sqrt(4.0*Gkx1);
    Gkx4 = sqrt(4.0*Gkx1*Gkx2)/Gkx3;

    Gky1 = cgains[2];
    Gky2 = cgains[3];
    Gky3 = sqrt(4.0*Gky1);
    Gky4 = sqrt(4.0*Gky1*Gky2)/Gky3;

    Gkz1 = cgains[4];
    Gkz2 = cgains[5];
    Gkz3 = sqrt(4.0*Gkz1);
    Gkz4 = sqrt(4.0*Gkz1*Gkz2)/Gkz3;

    // % phi
    Gkp1 = cgains[6];
    Gkp2 = cgains[7];
    Gkp3 = sqrt(4.0*Gkp1);
    Gkp4 = sqrt(4.0*Gkp1*Gkp2)/Gkp3;
    
    // % theta
    Gkt1 = cgains[8];
    Gkt2 = cgains[9];
    Gkt3 = sqrt(4.0*Gkt1);
    Gkt4 = sqrt(4.0*Gkt1*Gkt2)/Gkt3;
    
    // % psi
    Gks1 = cgains[10];
    Gks2 = cgains[11];
    Gks3 = sqrt(4.0*Gks1);
    Gks4 = sqrt(4.0*Gks1*Gks2)/Gks3;

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
    
    //# Matriz de rotação (Simplificada)                              // \in \mathbb{R}^{3 \times 3}
     
    // Linha 1:
    R[0] = cos(drone->pPosX[5]);
    R[1] = sin(drone->pPosX[5]);
    R[2] = -drone->pPosX[4];

    // Linha 2:
    R[3] = drone->pPosX[4] * drone->pPosX[3] * cos(drone->pPosX[5]) - sin(drone->pPosX[5]);
    R[4] = drone->pPosX[4] * drone->pPosX[3] * cos(drone->pPosX[5]) + cos(drone->pPosX[5]);
    R[5] = drone->pPosX[3];
    
    // Linha 3:
    R[6] = drone->pPosX[4] * cos(drone->pPosX[5]) + drone->pPosX[3] * sin(drone->pPosX[5]);
    R[7] = drone->pPosX[4] * sin(drone->pPosX[5]) - drone->pPosX[3] * cos(drone->pPosX[5]);
    R[8] = 1.0;

    // %-------------------------------
    // % Controle Cinematico
    // %-------------------------------
    etax = drone->pPosdXd[6] + Gkx1*tanh(Gkx2*drone->pPosXtil[0]) + Gkx3* tanh(Gkx4*drone->pPosXtil[6]);
    etay = drone->pPosdXd[7] + Gky1*tanh(Gky2*drone->pPosXtil[1]) + Gky3* tanh(Gky4*drone->pPosXtil[7]);
    etaz = drone->pPosdXd[8] + Gkz1*tanh(Gkz2*drone->pPosXtil[2]) + Gkz3* tanh(Gkz4*drone->pPosXtil[8]);
  
    etap = drone->pPosdXd[9]  + Gkp1*tanh(Gkp2*drone->pPosXtil[3]) + Gkp3*tanh(Gkp4*drone->pPosXtil[9]);
    etat = drone->pPosdXd[10] + Gkt1*tanh(Gkt2*drone->pPosXtil[4]) + Gkt3*tanh(Gkt4*drone->pPosXtil[10]);
    etas = drone->pPosdXd[11] + Gks1*tanh(Gks2*drone->pPosXtil[5]) + Gks3*tanh(Gks4*drone->pPosXtil[11]);

    drone->pPosXd[3] = rt_atan2((etax*sin(drone->pPosX[5]) - etay*cos(drone->pPosX[5]))*cos(drone->pPosX[4]), (etaz + drone->pParg));
    drone->pPosXd[4] = rt_atan2((etax*cos(drone->pPosX[5]) + etay*sin(drone->pPosX[5])), (etaz + drone->pParg));

    for (int ii = 3; ii < 5; ii++){
        if (fabs(drone->pPosXd[ii]) > M_PI){    
            drone->pPosXd[ii] += -2*M_PI;
        }
        if (drone->pPosXd[ii] < -M_PI){
            drone->pPosXd[ii] += 2*M_PI;
        }
    }

    // % Filtro de orientação (Robô)
    float umPerTs = 1.0/drone->pParTs;

    drone->pPosXd[9] = (drone->pPosXd[3] - drone->pPosXda[3])*umPerTs;
    drone->pPosXd[10] = (drone->pPosXd[4] - drone->pPosXda[4])*umPerTs;
    

    // % =======================================================================
    // % Parte Translacional
    for (int ii = 0; ii < 9; ii++) {                         // % Inertia matrix
        Mt[ii] = drone->pParm * (float)iv0[ii];              // \in \mathbb{R}^{3 \times 3}
    }
    
    
    // % Gravity matrix                                      // \in \mathbb{R}^{3 \times 1}
    Gt[0] = 0.0;
    Gt[1] = 0.0;
    Gt[2] = drone->pParm * drone->pParg;


    //  % ======================================================================

    // % Rotational inertia matrix                           // \in \mathbb{R}^{3 \times 3}
    // Linha 1:
    Mr[0] =  drone->pParIxx;
    Mr[1] =  0.0;
    Mr[2] = -drone->pPosX[4]*drone->pParIxx;

    // Linha 2:
    Mr[3] = 0.0;
    Mr[4] = drone->pParIyy + drone->pParIzz*drone->pPosX[3]*drone->pPosX[3];
    Mr[5] = drone->pPosX[3]*(drone->pParIyy - drone->pParIzz);
    
    // Linha 3:
    Mr[6] = -drone->pPosX[4]*drone->pParIxx;
    Mr[7] =  drone->pPosX[3]*(drone->pParIyy-drone->pParIzz);
    Mr[8] =  drone->pParIyy*drone->pPosX[3]*drone->pPosX[3] + drone->pParIxx*drone->pPosX[4]*drone->pPosX[4] + drone->pParIzz;

    
    // % Modelo no formato: M \ddot{q} + C \dot{q} + G = F
    float Z[9] = {0,0,0,0,0,0,0,0,0};   // {Linha 1, Linha 2, Linha 3}    // \in \mathbb{R}^{3 \times 3}

    // % Coriolis comentada => assumiu-se produto de velocidades = 0;
    /* % Cr = [cr1;cr2;cr3]; */
    /* % CC = [Ct Z; Z Cr];   % Matriz de Coriolis */

    // % Gravity vector
    float Gr[3] = {0, 0, 0};                                              // \in \mathbb{R}^{3 \times 1}

    // % Matriz de Inércia: [Mt Z; Z Mr];                                 // \in \mathbb{R}^{6 \times 6}
    // Linha 1:
    MM[0] = Mt[0];
    MM[1] = Mt[1];
    MM[2] = Mt[2];
    MM[3] = Z[0];
    MM[4] = Z[1];
    MM[5] = Z[2];
       
    // Linha 2:
    MM[6] = Mt[3];
    MM[7] = Mt[4];
    MM[8] = Mt[5];
    MM[9] = Z[3];
    MM[10] = Z[4];
    MM[11] = Z[5];

    // Linha 3:
    MM[12] = Mt[6];
    MM[13] = Mt[7];
    MM[14] = Mt[8];
    MM[15] = Z[6];
    MM[16] = Z[7];
    MM[17] = Z[8];

    // Linha 4:
    MM[18] = Z[0];
    MM[19] = Z[1];
    MM[20] = Z[2];
    MM[21] = Mr[0];
    MM[22] = Mr[1];
    MM[23] = Mr[2];

    // Linha 5:
    MM[24] = Z[3];
    MM[25] = Z[4];
    MM[26] = Z[5];
    MM[27] = Mr[3];
    MM[28] = Mr[4];
    MM[29] = Mr[5];

    // Linha 6:
    MM[30] = Z[6];
    MM[31] = Z[7];
    MM[32] = Z[8];
    MM[33] = Mr[6];
    MM[34] = Mr[7];
    MM[35] = Mr[8];


    // % Vetor de Forças Gravitacionais
    /* GG = [Gt; Gr];        */                                   // \in \mathbb{R}^{6 \times 1}
    // Vetor coluna:
    GG[0] = Gt[0];
    GG[1] = Gt[1];
    GG[2] = Gt[2];
    GG[3] = Gr[0];
    GG[4] = Gr[1];
    GG[5] = Gr[2];


    // % Matriz de Acoplamento e Matriz dos Braços de Forcas
    /* % [F1 F2 F3]' = R*At*[fx fy fz fytr]' */                   // \in \mathbb{R}^{3 \times 4}

    matrixMultMNdebug(At, R, auxVar1, 3, 4, 3);


    /* % [L M N]' = Ar*[fx fy fz fytr]' */                        // \in \mathbb{R}^{3 \times 4}
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

    

    for (int ii = 0; ii < 3; ii++) {       // varrendo colunas     // \in \mathbb{R}^{6 \times 4}
        for (int jj = 0; jj < 4; jj++) {   // varrendo linhas
            A[4*ii + jj] = At[4*ii + jj];
            A[4*ii + 12 + jj] = Ar[4*ii + jj];
        }
    }

    //---- Auxiliar:
    for (int i3 = 0; i3 < 4; i3++) {       // varrendo colunas
        Aaux[6*i3] = At[i3];
        Aaux[6*i3 + 1] = At[i3 + 4];
        Aaux[6*i3 + 2] = At[i3 + 8];

        Aaux[6*i3 + 3] = Ar[i3];
        Aaux[6*i3 + 1 + 3] = Ar[i3 + 4];
        Aaux[6*i3 + 2 + 3] = Ar[i3 + 8];
    }

    // % Matriz Pseudo-Inversa de A: A-sharp
    /* As = pinv(A);   */                           // % (A'*A)\A';
    //pinv(A, As); 

    ginv(Aaux, Ass);                                // \in \mathbb{R}^{4 \times 6}

    // rearrajando os dados:
    for (int i3 = 0; i3 < 4; i3++) {       // varrendo colunas
        As[6*i3]     = Ass[i3];
        As[6*i3 + 1] = Ass[i3 + 4*1];
        As[6*i3 + 2] = Ass[i3 + 4*2];

        As[6*i3 + 3] = Ass[i3 + 4*3];
        As[6*i3 + 4] = Ass[i3 + 4*4];
        As[6*i3 + 5] = Ass[i3 + 4*5];
    }



    // % Montagem da matriz sub-atuada ativa
    // % Matriz de Inérica

    matrixMultMNdebug(Ma, As, MM, 4, 6, 6);         // \in \mathbb{R}^{4 \times 6}      

    // % Passive variables X and Y                  // \in \mathbb{R}^{4 \times 2}
    Map[0] = Ma[0];
    Map[1] = Ma[1];

    Map[2] = Ma[6];
    Map[3] = Ma[7];

    Map[4] = Ma[12];
    Map[5] = Ma[13];

    Map[6] = Ma[18];
    Map[7] = Ma[19];


    // % Active variables Z, PHI, THETA and PSI     // \in \mathbb{R}^{4 \times 4}
    Maa[0]  = Ma[2];
    Maa[1]  = Ma[3];
    Maa[2]  = Ma[4];
    Maa[3]  = Ma[5];

    Maa[4]  = Ma[8];
    Maa[5]  = Ma[9];
    Maa[6]  = Ma[10];
    Maa[7]  = Ma[11];

    Maa[8]  = Ma[14];
    Maa[9]  = Ma[15];
    Maa[10] = Ma[16];
    Maa[11] = Ma[17];

    Maa[12] = Ma[20];
    Maa[13] = Ma[21];
    Maa[14] = Ma[22];
    Maa[15] = Ma[23];


    // % Matriz de Coriolis (zero) e Vetor de Forças Gravitacionais Ativa
    /* Ea  = As*(GG + [drone.pParD(1:3)' 0 0 0]'); */
    GG[0] = GG[0] + drone->pParD[0];
    GG[1] = GG[1] + drone->pParD[1];
    GG[2] = GG[2] + drone->pParD[2];
    GG[3] = GG[3] + 0.0;
    GG[4] = GG[4] + 0.0;
    GG[5] = GG[5] + 0.0;

    matrixMultMNdebug(Ea, As, GG, 4, 1, 6);         // \in \mathbb{R}^{4 \times 1} 
    


    // % Seguindo a nova plataforma
    // % Escrita das matrizes passivas
    /* MP = R'*Mt; */
    /* GP = R'*Gt; */

    Transpose(R, Rlinha, 3, 3);                     // \in \mathbb{R}^{3 \times 3}

    matrixMultMNdebug(MP, Rlinha, Mt, 3, 3, 3);

    matrixMultMNdebug(GP, Rlinha, Gt, 3, 1, 3);          // \in \mathbb{R}^{3 \times 1}


    /* Mpp =  MP(1:2,1:2); */
    /* Mpa = [MP(1:2,3) zeros(2,3)]; */

    Mpp[0] = MP[0]; Mpp[1] = MP[1]; Mpp[2] = MP[3]; Mpp[3] = MP[4];   // \in \mathbb{R}^{2 \times 2}


    Mpa[0] = MP[2]; Mpa[1] = 0.0; Mpa[2] = 0.0; Mpa[3] = 0.0;         // \in \mathbb{R}^{2 \times 4}
    Mpa[4] = MP[5]; Mpa[5] = 0.0; Mpa[6] = 0.0; Mpa[7] = 0.0;


    Ep[0] = GP[0]; Ep[1] = GP[1];


    // % Representação na forma sub-atuada
    // % M = [Mpp Mpa; Map Maa];
    // % E = [Ep; Ea];

    /* D = Maa - Map*(Mpp\Mpa); */
    /* H = Ea - Map*(Mpp\Ep); */

    float Mpp_inv[4];
    //inverse(Mpp,Mpp_inv,2);                         // \in \mathbb{R}^{2 \times 2}           // Mpp[0] = a; Mpp[1] = b; 
                                                                                               // Mpp[2] = c; Mpp[3] = d;
    float detMpp = Mpp[0]*Mpp[3] - Mpp[1]*Mpp[2];     // Assume-se que det(Mpp) != 0 
    float divDetMpp = 1.0/detMpp;                     // https://pt.wikipedia.org/wiki/Matriz_inversa

    Mpp_inv[0] =  divDetMpp*Mpp[3];
    Mpp_inv[1] = -divDetMpp*Mpp[1];
    Mpp_inv[2] = -divDetMpp*Mpp[2];
    Mpp_inv[3] =  divDetMpp*Mpp[0];

    float MppMpa[8];
    matrixMultMNdebug(MppMpa, Mpp_inv, Mpa, 2, 4, 2);    // \in \mathbb{R}^{2 \times 4}

    float Daux[16];
    matrixMultMNdebug(Daux, Map, MppMpa, 4, 4, 2);       // \in \mathbb{R}^{4 \times 4}

    float MppEp[2];
    matrixMultMNdebug(MppEp, Mpp_inv, Ep, 2, 1, 2);      // \in \mathbb{R}^{2 \times 1}

    float Haux[4];
    matrixMultMNdebug(Haux, Map, MppEp, 4, 1, 2);        // \in \mathbb{R}^{4 \times 1}

    for (int i = 0; i < 16; i++){                   // \in \mathbb{R}^{4 \times 4}
        D[i] = Maa[i] - Daux[i];
    }

    for (int i = 0; i < 4; i++){                    // \in \mathbb{R}^{4 \times 1}
        H[i] = Ea[i] - Haux[i];
    }
    
    

    /* eta = [etaz; etap; etat; etas]; */           // \in \mathbb{R}^{4 \times 1}
    eta[0] = etaz;
    eta[1] = etap;
    eta[2] = etat; 
    eta[3] = etas;


    // % Vetor de Forças de referência aplicado no referencial do veículo
    /* Fr = D*eta + H; */

    float Fraux[4];
    matrixMultMNdebug(Fraux, D, eta, 4, 1, 4);           // \in \mathbb{R}^{4 \times 1}

    for (int i = 0; i < 4; i++){                    // \in \mathbb{R}^{4 \times 1}
        Fr[i] = Fraux[i] + H[i];
    }

    //print_matrix(Fr, 4, 1);

    // % Verificando se forças sobre o referência do veículo ocorrem somente na direção Z
    /* fTau = A*Fr; */
    matrixMultMNdebug(fTau, A, Fr, 6, 1, 4);             // \in \mathbb{R}^{6 \times 1}         

    // % ------------------------------------
    // % Forçando valores possíveis: 30% do valor da gravidade
    if (fTau[2] < 0) {                             
        fTau[2] = drone->pParm*drone->pParg*0.3;
    }
    // % ------------------------------------


    // % Considerando a situação mais simples de que a força de propulsão
    // % solicitada aos motores é imediatamente atendida
    // % NÂO considera modelo da bateria
    // % Modelo Inverso do Atuador: Forças desejada nos propulsores
    // Fd = As*fTau + (R*[[-1 1 1 -1];[-1 -1 1 1]; 1 1 1 1])'*drone.pParD(4:6); 
    

    float Fdaux[4];
    float Fdaux1[4];
    float Raux[12];
    float Rauxlinha[12];


    matrixMultMNdebug(Fdaux, As, fTau, 4, 1, 6);                                // \in \mathbb{R}^{4 \times 1}
    matrixMultMNdebug(Raux, R, bb, 3, 4, 3);                                    // \in \mathbb{R}^{3 \times 4}
    
    Transpose(Raux, Rauxlinha, 3, 4);                                      // \in \mathbb{R}^{4 \times 3}

    float parD[3] = {drone->pParD[3],drone->pParD[4],drone->pParD[5]};

    matrixMultMNdebug(Fdaux1, Raux, parD, 4, 1, 3);                             // \in \mathbb{R}^{4 \times 1}


    // % Caso a força do propulsor seja negativa, assume-se propulsão igual a zero
    for (int ii = 0; ii < 4; ii++){
        Fd[ii] = Fdaux[ii] + Fdaux1[ii];

        if (Fd[ii] < 0.0){
            Fd[ii] = 0.0;
        }
    }
        
    
    // % 1: Fr -> Wr                                                       // \in \mathbb{R}^{4 \times 1}
    Wda[0] = drone->pSCWd[0];
    Wda[1] = drone->pSCWd[1];
    Wda[2] = drone->pSCWd[2];
    Wda[3] = drone->pSCWd[3];

    float auxpSCWd = 1.0/drone->pParCf;

    drone->pSCWd[0] = sqrt(Fd[0]*auxpSCWd);                                // \in \mathbb{R}^{4 \times 1}
    drone->pSCWd[1] = sqrt(Fd[1]*auxpSCWd);
    drone->pSCWd[2] = sqrt(Fd[2]*auxpSCWd);
    drone->pSCWd[3] = sqrt(Fd[3]*auxpSCWd);



    // % 2: Wr -> V 
    float Vraux1 =  drone->pParJm*drone->pParR/drone->pParKm/drone->pParTs;
    float Vraux2 = (drone->pParBm*drone->pParR/drone->pParKm + drone->pParKb);
    float Vraux3 = drone->pParCt*drone->pParR/drone->pParKm/drone->pParr;

    for (int ii = 0; ii < 4; ii++){                                        // \in \mathbb{R}^{4 \times 1}
        Vr[ii] = -drone->pParVo + Vraux1*(drone->pSCWd[ii] - Wda[ii]) + Vraux2*drone->pSCWd[ii] + Vraux3*drone->pSCWd[ii]*drone->pSCWd[ii];
    }
    

    // % 3: V -> Xr
    drone->pSCXr[3] = drone->pPosX[3] + 1.0/(drone->pParkdp + drone->pParkpp*drone->pParTs)*(drone->pParkdp*(drone->pSCXr[3] - drone->pPosX[3]) + 0.25*drone->pParTs*(Vr[0] + Vr[1] - Vr[2] - Vr[3]));

    drone->pSCXr[4] = drone->pPosX[4] + 1.0/(drone->pParkdt + drone->pParkpt*drone->pParTs)*(drone->pParkdt*(drone->pSCXr[4] - drone->pPosX[4]) + 0.25*drone->pParTs*(- Vr[0] + Vr[1] + Vr[2] - Vr[3]));

    drone->pSCXr[8] = drone->pPosX[8] + 1.0/(drone->pParkdz + drone->pParkpz*drone->pParTs)*(drone->pParkdz*(drone->pSCXr[8] - drone->pPosX[8]) + 0.25*drone->pParTs*(Vr[0] + Vr[1] + Vr[2] + Vr[3]));

    drone->pSCXr[11] = drone->pPosX[11] + 1.0/(drone->pParkds + drone->pParkps*drone->pParTs)*(drone->pParkds*(drone->pSCXr[11] - drone->pPosX[11]) + 0.25*drone->pParTs*(Vr[0] - Vr[1] + Vr[2] - Vr[3]));
    
    
    
    // % 4: Xr -> U
    drone->pSCUd[0] =  drone->pSCXr[3] /drone->pParuSat[0];  // % Phi
    drone->pSCUd[1] = -drone->pSCXr[4] /drone->pParuSat[1];  // % Theta
    drone->pSCUd[2] =  drone->pSCXr[8] /drone->pParuSat[2];  // % dZ
    drone->pSCUd[3] = -drone->pSCXr[11]/drone->pParuSat[3];  // % dPsi

    // Tratamento dos Sinais de Controle:
    drone->pSCUd[0] = tanh(drone->pSCUd[0]);
    drone->pSCUd[1] = tanh(drone->pSCUd[1]);
    drone->pSCUd[2] = tanh(drone->pSCUd[2]);
    drone->pSCUd[3] = tanh(drone->pSCUd[3]);
    

    // --- 
    delete fTau;
  
}


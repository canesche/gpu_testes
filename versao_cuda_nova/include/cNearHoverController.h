//# Controle Dinâmico do UAV
    
//# Declaração de variáveis auxiliares:
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
float Ar[12], A[24];
float Map[8], Maa[16], Ea[4], MP[9], GP[3];
float Wda[4], Mpp[4], Mpa[8], Ep[2];
float D[16], H[4], eta[4], Fr[4];
float At[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}; // \in \mathbb{R}^{3x4}

float Vr[4];

float As[24], Ma[24];
float Fd[4];
float fTau[6];

float auxVar1[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};    
float bb[12] = { -1.0, 1.0, 1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };   
float cgains[12] = {0.5, 2.0, 0.5, 2.0, 5.0, 2.0, 1.0, 20.0, 1.0, 15.0, 1.0, 2.5}; 

//# Auxiliares Variables:
float Ass[24], Aaux[24];

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

//# phi
Gkp1 = cgains[6];
Gkp2 = cgains[7];
Gkp3 = sqrt(4.0*Gkp1);
Gkp4 = sqrt(4.0*Gkp1*Gkp2)/Gkp3;
    
//# theta
Gkt1 = cgains[8];
Gkt2 = cgains[9];
Gkt3 = sqrt(4.0*Gkt1);
Gkt4 = sqrt(4.0*Gkt1*Gkt2)/Gkt3;
    
//# psi
Gks1 = cgains[10];
Gks2 = cgains[11];
Gks3 = sqrt(4.0*Gks1);
Gks4 = sqrt(4.0*Gks1*Gks2)/Gks3;

//# ---------------------------------------------------------

#pragma unroll
for (int ii = 0; ii < 12; ++ii ) {
    pPosXda[ii] = pPosXd[ii];                        
    pPosXtil[ii] = pPosXd[ii] - pPosX[ii]; //# Calculando erro de posição
}

for (int ii = 3; ii < 6; ++ii){
    if (fabs(pPosXtil[ii]) > M_PI) {
        pPosXtil[ii] += -2.0*M_PI;   //# + pPosXtil[ii];
    }
    if (pPosXtil[ii] < -M_PI) {
        pPosXtil[ii] += 2.0*M_PI;    //# + pPosXtil[ii];
    }
}
    
//# Matriz de rotação (Simplificada) 
     
//# Linha 1:
R[0] = cos(pPosX[5]);
R[1] = sin(pPosX[5]);
R[2] = -pPosX[4];

//# Linha 2:
R[3] = pPosX[4] * pPosX[3] * cos(pPosX[5]) - sin(pPosX[5]);
R[4] = pPosX[4] * pPosX[3] * cos(pPosX[5]) + cos(pPosX[5]);
R[5] = pPosX[3];
    
//# Linha 3:
R[6] = pPosX[4] * cos(pPosX[5]) + pPosX[3] * sin(pPosX[5]);
R[7] = pPosX[4] * sin(pPosX[5]) - pPosX[3] * cos(pPosX[5]);
R[8] = 1.0;
    
//# Controle Cinematico
etax = pPosdXd[6] + Gkx1*tanh(Gkx2*pPosXtil[0]) + Gkx3* tanh(Gkx4*pPosXtil[6]);
etay = pPosdXd[7] + Gky1*tanh(Gky2*pPosXtil[1]) + Gky3* tanh(Gky4*pPosXtil[7]);
etaz = pPosdXd[8] + Gkz1*tanh(Gkz2*pPosXtil[2]) + Gkz3* tanh(Gkz4*pPosXtil[8]);
  
etap = pPosdXd[9]  + Gkp1*tanh(Gkp2*pPosXtil[3]) + Gkp3*tanh(Gkp4*pPosXtil[9]);
etat = pPosdXd[10] + Gkt1*tanh(Gkt2*pPosXtil[4]) + Gkt3*tanh(Gkt4*pPosXtil[10]);
etas = pPosdXd[11] + Gks1*tanh(Gks2*pPosXtil[5]) + Gks3*tanh(Gks4*pPosXtil[11]);

pPosXd[3] = rt_atan2((etax*sin(pPosX[5]) - etay*cos(pPosX[5]))*cos(pPosX[4]), etaz + pParg);
pPosXd[4] = rt_atan2(etax*cos(pPosX[5]) + etay*sin(pPosX[5]), etaz + pParg);

#pragma unroll
for (int ii = 3; ii < 5; ii++){
    if (fabs(pPosXd[ii]) > M_PI){    
        pPosXd[ii] += -2*M_PI;
    }
    if (pPosXd[ii] < -M_PI){
        pPosXd[ii] += 2*M_PI;
    }
}

//# Filtro de orientação (Robô)
float umPerTs = 1.0/pParTs;

pPosXd[9] = (pPosXd[3] - pPosXda[3])*umPerTs;
pPosXd[10] = (pPosXd[4] - pPosXda[4])*umPerTs;

//# Parte Translacional
#pragma unroll
for (int ii = 0; ii < 9; ii++) {       //# Inertia matrix
    Mt[ii] = pParm * (float) iv0[ii];    
}
    
//# Gravity matrix  
Gt[0] = 0.0;
Gt[1] = 0.0;
Gt[2] = pParm * pParg;

//# Rotational inertia matrix 
//# Linha 1:
Mr[0] = pParIxx;
Mr[1] = 0.0;
Mr[2] = -pPosX[4]*pParIxx;

//# Linha 2:
Mr[3] = 0.0;
Mr[4] = pParIyy + pParIzz * pPosX[3] * pPosX[3];
Mr[5] = pPosX[3] * (pParIyy - pParIzz);
    
//# Linha 3:
Mr[6] = -pPosX[4]*pParIxx;
Mr[7] =  pPosX[3]*(pParIyy-pParIzz);
Mr[8] =  pParIyy*pPosX[3]*pPosX[3] + pParIxx*pPosX[4]*pPosX[4] + pParIzz;

//# Modelo no formato: M \ddot{q} + C \dot{q} + G = F
float Z[9] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};   //# {Linha 1, Linha 2, Linha 3} 

//# Coriolis comentada => assumiu-se produto de velocidades = 0;
//# Cr = [cr1;cr2;cr3];
//# CC = [Ct Z; Z Cr];   % Matriz de Coriolis

//# Gravity vector
float Gr[3] = {0.0, 0.0, 0.0}; 

//# Matriz de Inércia: [Mt Z; Z Mr]; 
//# Linha 1:
MM[0] = Mt[0];
MM[1] = Mt[1];
MM[2] = Mt[2];
MM[3] = Z[0];
MM[4] = Z[1];
MM[5] = Z[2];
       
//# Linha 2:
MM[6] = Mt[3];
MM[7] = Mt[4];
MM[8] = Mt[5];
MM[9] = Z[3];
MM[10] = Z[4];
MM[11] = Z[5];

//# Linha 3:
MM[12] = Mt[6];
MM[13] = Mt[7];
MM[14] = Mt[8];
MM[15] = Z[6];
MM[16] = Z[7];
MM[17] = Z[8];

//# Linha 4:
MM[18] = Z[0];
MM[19] = Z[1];
MM[20] = Z[2];
MM[21] = Mr[0];
MM[22] = Mr[1];
MM[23] = Mr[2];

//# Linha 5:
MM[24] = Z[3];
MM[25] = Z[4];
MM[26] = Z[5];
MM[27] = Mr[3];
MM[28] = Mr[4];
MM[29] = Mr[5];

//# Linha 6:
MM[30] = Z[6];
MM[31] = Z[7];
MM[32] = Z[8];
MM[33] = Mr[6];
MM[34] = Mr[7];
MM[35] = Mr[8];

//# Vetor de Forças Gravitacionais
//# GG = [Gt; Gr];  
//# Vetor coluna:
GG[0] = Gt[0];
GG[1] = Gt[1];
GG[2] = Gt[2];
GG[3] = Gr[0];
GG[4] = Gr[1];
GG[5] = Gr[2];

//# Matriz de Acoplamento e Matriz dos Braços de Forcas
//# [F1 F2 F3]' = R*At*[fx fy fz fytr]' 
matrixMult(At, R, auxVar1, 3, 4, 3);

//# [L M N]' = Ar*[fx fy fz fytr]'
Ar[0] =  pPark1;
Ar[1] =  pPark1;
Ar[2] = -pPark1;
Ar[3] = -pPark1;

Ar[4] = -pPark1;
Ar[5] =  pPark1;
Ar[6] =  pPark1;
Ar[7] = -pPark1;

Ar[8] =  pPark2;
Ar[9] = -pPark2;
Ar[10] =  pPark2;
Ar[11] = -pPark2;  

for (int ii = 0; ii < 3; ii++) {       //# varrendo colunas
    for (int jj = 0; jj < 4; jj++) {   //# varrendo linhas
        A[4*ii + jj] = At[4*ii + jj];
        A[4*ii + 12 + jj] = Ar[4*ii + jj];
    }
}

//#Auxiliar:
for (int i3 = 0; i3 < 4; i3++) {       //# varrendo colunas
    Aaux[6*i3] = At[i3];
    Aaux[6*i3 + 1] = At[i3 + 4];
    Aaux[6*i3 + 2] = At[i3 + 8];

    Aaux[6*i3 + 3] = Ar[i3];
    Aaux[6*i3 + 1 + 3] = Ar[i3 + 4];
    Aaux[6*i3 + 2 + 3] = Ar[i3 + 8];
}

//# Matriz Pseudo-Inversa de A: A-sharp
//# As = pinv(A);  // % (A'*A)\A';
//# pinv(A, As); 

ginv(Aaux, Ass);                        // # O erro pode estar aqui???

//# rearrajando os dados:
for (int i3 = 0; i3 < 4; i3++) {       //# varrendo colunas
    As[6*i3]     = Ass[i3];
    As[6*i3 + 1] = Ass[i3 + 4*1];
    As[6*i3 + 2] = Ass[i3 + 4*2];

    As[6*i3 + 3] = Ass[i3 + 4*3];
    As[6*i3 + 4] = Ass[i3 + 4*4];
    As[6*i3 + 5] = Ass[i3 + 4*5];
}

//# Montagem da matriz sub-atuada ativa
//# Matriz de Inérica

matrixMult(Ma, As, MM, 4, 6, 6); 

//# Passive variables X and Y
Map[0] = Ma[0];
Map[1] = Ma[1];

Map[2] = Ma[6];
Map[3] = Ma[7];

Map[4] = Ma[12];
Map[5] = Ma[13];

Map[6] = Ma[18];
Map[7] = Ma[19];

//# Active variables Z, PHI, THETA and PSI
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

//# Matriz de Coriolis (zero) e Vetor de Forças Gravitacionais Ativa
//# Ea  = As*(GG + [drone.pParD(1:3)' 0 0 0]');
GG[0] = GG[0] + pParD[0];
GG[1] = GG[1] + pParD[1];
GG[2] = GG[2] + pParD[2];
GG[3] = GG[3] + 0.0;
GG[4] = GG[4] + 0.0;
GG[5] = GG[5] + 0.0;

matrixMult(Ea, As, GG, 4, 1, 6);

//# Seguindo a nova plataforma
//# Escrita das matrizes passivas
//# MP = R'*Mt;
//# GP = R'*Gt; 

Transpose(R, Rlinha, 3, 3); 

matrixMult(MP, Rlinha, Mt, 3, 3, 3);
matrixMult(GP, Rlinha, Gt, 3, 1, 3); 

//# Mpp =  MP(1:2,1:2); 
//# Mpa = [MP(1:2,3) zeros(2,3)]; 

Mpp[0] = MP[0]; Mpp[1] = MP[1]; Mpp[2] = MP[3]; Mpp[3] = MP[4];  

Mpa[0] = MP[2]; Mpa[1] = 0.0; Mpa[2] = 0.0; Mpa[3] = 0.0;    
Mpa[4] = MP[5]; Mpa[5] = 0.0; Mpa[6] = 0.0; Mpa[7] = 0.0;

Ep[0] = GP[0]; Ep[1] = GP[1];

//# Representação na forma sub-atuada
//# M = [Mpp Mpa; Map Maa];
//# E = [Ep; Ea];

//# D = Maa - Map*(Mpp\Mpa); 
//# H = Ea - Map*(Mpp\Ep); 

float Mpp_inv[4];

float detMpp = Mpp[0]*Mpp[3] - Mpp[1]*Mpp[2];     //# Assume-se que det(Mpp) != 0 
float divDetMpp = 1.0/detMpp;                     //# https://pt.wikipedia.org/wiki/Matriz_inversa

Mpp_inv[0] =  divDetMpp*Mpp[3];
Mpp_inv[1] = -divDetMpp*Mpp[1];
Mpp_inv[2] = -divDetMpp*Mpp[2];
Mpp_inv[3] =  divDetMpp*Mpp[0];

float MppMpa[8];
matrixMult(MppMpa, Mpp_inv, Mpa, 2, 4, 2);    

float Daux[16];
matrixMult(Daux, Map, MppMpa, 4, 4, 2);      

float MppEp[2];
matrixMult(MppEp, Mpp_inv, Ep, 2, 1, 2);     

float Haux[4];
matrixMult(Haux, Map, MppEp, 4, 1, 2);     

for (int i = 0; i < 16; i++){               
    D[i] = Maa[i] - Daux[i];
}

for (int i = 0; i < 4; i++){                  
    H[i] = Ea[i] - Haux[i];
} 

//# eta = [etaz; etap; etat; etas];
eta[0] = etaz;
eta[1] = etap;
eta[2] = etat; 
eta[3] = etas;

//# Vetor de Forças de referência aplicado no referencial do veículo
//# Fr = D*eta + H; 

float Fraux[4];
matrixMult(Fraux, D, eta, 4, 1, 4);      

for (int i = 0; i < 4; ++i){                   
    Fr[i] = Fraux[i] + H[i];
}

//# Verificando se forcas sobre o referência do veículo ocorrem somente na direção Z
//# fTau = A*Fr; 
matrixMult(fTau, A, Fr, 6, 1, 4);               

//# Forçando valores possíveis: 30% do valor da gravidade
if (fTau[2] < 0) {                             
    fTau[2] = pParm * pParg * 0.3;
}

//# Considerando a situação mais simples de que a força de propulsão
//# solicitada aos motores é imediatamente atendida
//# NÂO considera modelo da bateria
//# Modelo Inverso do Atuador: Forças desejada nos propulsores
//# Fd = As*fTau + (R*[[-1 1 1 -1];[-1 -1 1 1]; 1 1 1 1])'*drone.pParD(4:6);   

float Fdaux[4];
float Fdaux1[4];
float Raux[12];
float Rauxlinha[12];

matrixMult(Fdaux, As, fTau, 4, 1, 6);                               
matrixMult(Raux, R, bb, 3, 4, 3);                                   
    
Transpose(Raux, Rauxlinha, 3, 4);                                   

float parD[3] = {pParD[3], pParD[4], pParD[5]};

matrixMult(Fdaux1, Raux, parD, 4, 1, 3);                         

//# Caso a força do propulsor seja negativa, assume-se propulsão igual a zero
for (int ii = 0; ii < 4; ii++) {
    Fd[ii] = Fdaux[ii] + Fdaux1[ii];
    if (Fd[ii] < 0.0){ Fd[ii] = 0.0; };
}   
    
//# 1: Fr -> Wr          
Wda[0] = pSCWd[0];
Wda[1] = pSCWd[1];
Wda[2] = pSCWd[2];
Wda[3] = pSCWd[3];

float auxpSCWd = 1.0 / pParCf;

pSCWd[0] = sqrt(Fd[0]*auxpSCWd);            
pSCWd[1] = sqrt(Fd[1]*auxpSCWd);
pSCWd[2] = sqrt(Fd[2]*auxpSCWd);
pSCWd[3] = sqrt(Fd[3]*auxpSCWd);

//# 2: Wr -> V 
float Vraux1 = pParJm*pParR / pParKm / pParTs;
float Vraux2 = (pParBm*pParR / pParKm + pParKb);
float Vraux3 = pParCt*pParR / pParKm / pParr;

for (int ii = 0; ii < 4; ii++){                                       
  Vr[ii] = -pParVo + Vraux1*(pSCWd[ii] - Wda[ii]) + Vraux2*pSCWd[ii] + Vraux3*pSCWd[ii]*pSCWd[ii];
}  

//# 3: V -> Xr
pSCXr[3] = pPosX[3] + 1.0/(pParkdp + pParkpp*pParTs)*(pParkdp*(pSCXr[3] - pPosX[3]) + 0.25*pParTs*(Vr[0] + Vr[1] - Vr[2] - Vr[3]));
pSCXr[4] = pPosX[4] + 1.0/(pParkdt + pParkpt*pParTs)*(pParkdt*(pSCXr[4] - pPosX[4]) + 0.25*pParTs*(- Vr[0] + Vr[1] + Vr[2] - Vr[3]));
pSCXr[8] = pPosX[8] + 1.0/(pParkdz + pParkpz*pParTs)*(pParkdz*(pSCXr[8] - pPosX[8]) + 0.25*pParTs*(Vr[0] + Vr[1] + Vr[2] + Vr[3]));
pSCXr[11] = pPosX[11] + 1.0/(pParkds + pParkps*pParTs)*(pParkds*(pSCXr[11] - pPosX[11]) + 0.25*pParTs*(Vr[0] - Vr[1] + Vr[2] - Vr[3]));

//# 4: Xr -> U
pSCUd[0] =  pSCXr[3] /pParuSat[0];  //# Phi
pSCUd[1] = -pSCXr[4] /pParuSat[1];  //# Theta
pSCUd[2] =  pSCXr[8] /pParuSat[2];  //# dZ
pSCUd[3] = -pSCXr[11]/pParuSat[3];  //# dPsi

//# Tratamento dos Sinais de Controle:
pSCUd[0] = tanh(pSCUd[0]);
pSCUd[1] = tanh(pSCUd[1]);
pSCUd[2] = tanh(pSCUd[2]);
pSCUd[3] = tanh(pSCUd[3]);
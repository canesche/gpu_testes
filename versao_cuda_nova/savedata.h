
//# Armazenando dados de simulação (salvando em coluna/iteração)
//# XX(:,c) = [A.pPosXd; A.pPosX; A.pSCUd; t];

#pragma unroll
for (int ii = 0; ii < 12; ++ii){
    data[simN + c*12+ii] = pPosX[ii];    // # Postura atual    [0 -> 11]
    
    //data[c*12+ii] = pPosXd[ii];    // # Postura desejada    [0 -> 11]
    //data[c*12+ ii + 12] = pPosX[ii];  // # Postura atual       [12 -> 23]
}
//#pragma unroll
//for (int ii = 0; ii < 4; ++ii){
//    data[c*29 + ii + 24] = pSCU[ii];   // # Postura desejada    [24 -> 27]
//}
//data[c*29+28] = t;  // # Tempo de Simulação       [28 -> 28]    
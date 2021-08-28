
void saveData(float XX[1801][29], struct ArDrone *drone, float t,int c){
    
    // % Armazenando dados de simulação (salvando em coluna/iteração)
    // XX(:,c) = [A.pPosXd; A.pPosX; A.pSCUd; t];

    for (int ii = 0; ii < 12; ++ii){        
        XX[c][ii] = drone->pPosXd[ii];       // % Postura desejada    [0 -> 11]
        XX[c][ii + 12] = drone->pPosX[ii];   // % Postura atual       [12 -> 23]
    }
    for (int ii = 0; ii < 4; ++ii){
        XX[c][ii + 24] = drone->pSCU[ii];    // % Postura desejada    [24 -> 27]
    }    
    XX[c][28] = t;                      // % Tempo de Simulação       [28 -> 28]                  (float)       
}
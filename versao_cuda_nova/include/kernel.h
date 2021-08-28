
#include "utils.h"
#include "ginv.h"

__global__ void simulation(float *data, int IDX_SIZE, float *MAPA, float *d_parametersData) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    
    if (idx < NUMBER_DRONES) {
        #include "declaration_variable.h"
        #include "initControlVar.h"
        //#include "rGetSensorData.h"

        // # Parâmetros de Entrada:
        float delay = MAPA[3*idx + 0];
        float Kp = MAPA[3*idx + 1];
        float Kd = MAPA[3*idx + 2];


        int simN    = 12*idx*IDX_SIZE;       // pega a primeira posição da simulação N na matriz de dados

        float teto = 100.5;
            
        float t;


        // % Métricas
        float erro_instavel = 5;
        float IAE_naoAtrasado = 5.3218;
        float ITAE_naoAtrasado = 7.3256;
        float erro_inicial[3];

        
        // Control Análise
        bool limiteErro     = true;
        bool limiteIAE      = true;
        bool limiteITAE     = true;
        bool limiteNormErro = true;

        float posError;

        // # Way-point
        pPosXd[0] = 2.0;
        pPosXd[1] = 1.0;
        pPosXd[2] = 2.0;
        pPosXd[6] = 0.0;
        pPosXd[7] = 0.0;
        pPosXd[8] = 0.0;


        // # Inicialização dos Índices de Desempenho (drone atrasado):
        float IAE  = 0.0;
        float ITAE = 0.0;
        float IASC = 0.0;

        erro_inicial[0] = pPosXd[0] - pPosX[0];
        erro_inicial[1] = pPosXd[1] - pPosX[1];
        erro_inicial[2] = pPosXd[2] - pPosX[2];

        float Xa[12] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};



        #pragma unroll
        for (int c = 0; c < IDX_SIZE; ++c) {    // # for (int c = 0; c < IDX_SIZE; ++c) {    
            t = c * 0.03333f;


            //# getting robot data
            //#include "rGetSensorData.h"


            // % Inicio da realimentação:
            
            // % -- Implementação (drone envia informações atuais ao controlador
            // %                   até c atingir o idAtraso, então enviará
            // %                   dados atrasados ao controle):
            if (c > MAPA[3*idx + 0]){
                int idDelay = 12.0*(- MAPA[3*idx + 0]); // adiciona o atraso para ler na matriz em uma posição já salva, igual 
                                                     // ao número de iterações já feitas menos o atraso
                #pragma unroll
                for (int ii = 0; ii < 12; ++ii ) {
                    Xa[ii] = data[simN + idDelay + 12*c + ii]; // Salta uma linha (12*c, pois tem 12 colunas) e preence coluna por coluna com o 'ii'
                }
            } else {
                #pragma unroll
                for (int ii = 0; ii < 12; ++ii ) {
                    Xa[ii] = data[simN + 12*c + ii];
                }
            }
       
            
            // % --

            //# Controlador:
            #include "cUnderActuatedControl.h"

            //# Armazenando dados de simulação
            #include "savedata.h"

            // # Índices de desempenho (Atrasado & Não-atrasado)
            IAE = IAE + sqrt(pPosXtil[0]*pPosXtil[0]+pPosXtil[1]*pPosXtil[1]+pPosXtil[2]*pPosXtil[2])*pParTs;
            ITAE = ITAE + sqrt(pPosXtil[0]*pPosXtil[0]+pPosXtil[1]*pPosXtil[1]+pPosXtil[2]*pPosXtil[2])*t*pParTs;
            IASC = IASC + sqrt(pSCUd[0]*pSCUd[0]+pSCUd[1]*pSCUd[1]+pSCUd[2]*pSCUd[2]+pSCUd[3]*pSCUd[3])*pParTs;

            //# rSendControlSignals(&A);
            #include "rSendControlSignals.h"

            
            // # =========================================================================
            // # Métricas de Análise (Critérios de Parada)

            // # 1 - Limite para o Erro:
            posError = sqrt((pPosXd[0] - pPosX[0])*(pPosXd[0] - pPosX[0])+(pPosXd[1] - pPosX[1])*(pPosXd[1] - pPosX[1])+(pPosXd[2] - pPosX[2])*(pPosXd[2] - pPosX[2]));

            if (posError > erro_instavel){ // # Inválido para esta condição
                limiteErro = false;
                break;
            } // # erro_intavel = 5 -> Limite para o overshoot, que consideramos instável para essa tarefa!
            
            // # 2 - Limite para o IAE:
            if (IAE > 100.0*IAE_naoAtrasado){ // 30
                limiteIAE = false;
                break;
            } // # IAE_naoAtrasado = 5.3218 -> Valor obtido em simulação! Deseja-se que o erro de regime transitório não seja muito grande.
            
            // # 3 - Limite para o ITAE:
            if (ITAE > 100.0*ITAE_naoAtrasado){ // 50
                limiteITAE = false;
                break;
            } // # ITAE_naoAtrasado = 7.3256 -> Valor obtido em simulação! Deseja-se que o erro de regime permanente não seja muito grande. Que o drone não oscile muito e tenda a convergir.
            
            // # 4 - Limite para o Erro de Posição:
            if (posError > 2.0*sqrt(erro_inicial[0]*erro_inicial[0]+erro_inicial[1]*erro_inicial[1]+erro_inicial[2]*erro_inicial[2])){
                limiteNormErro = false;
                break;
            } // # Sempre que o erro for maior que o erro inicial! Critério de parada baseado na norma do erro.
          
        }


// # Ou então, alterar no próprio MAPA, se a combinação convergir, não faz nada. Se não convergir, no MAPA
// # coloca o valor do 'teto', assim, ao retorna pra CPU é só recopiar o MAPA e não precisa ter outro vetor com o mesmo tamanho
// # não precisará do 'parametersData'

        if (limiteErro && limiteIAE && limiteITAE && limiteNormErro){              
            if ((MAPA[3*idx + 0] != 0.0) && (MAPA[3*idx + 1] != 0.0) && (MAPA[3*idx + 2] != 0.0)){
                d_parametersData[3*idx + 0] = delay; //MAPA[3*idx + 0];
                d_parametersData[3*idx + 1] = MAPA[3*idx + 1];
                d_parametersData[3*idx + 2] = MAPA[3*idx + 2];
            }
            else{
                d_parametersData[3*idx + 0] = teto;
                d_parametersData[3*idx + 1] = teto;
                d_parametersData[3*idx + 2] = teto;
            }
            //return true;
        }
        else{
            d_parametersData[3*idx + 0] = teto;
            d_parametersData[3*idx + 1] = teto;
            d_parametersData[3*idx + 2] = teto;
            //return false;
        }
    }
}
// Control Variables

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
  
void initControlVar(ArDrone *drone, int threads){
    
    //--------------- Initializing Variables %%%% Starting %%%%
    //int n = 12;
    for (int j = 0; j < threads; ++j){
        for (int i = 0; i < 12; i++) {
            drone[j].pPosX[i] = 0.0;   // {0,0,0.75,0,0,0,0,0,0,0,0,0}; % Real Pose (Pose corrente)
            drone[j].pPosXa[i] = 0.0;  // {0,0,0,0,0,0,0,0,0,0,0,0};    % Previous Pose (Pose anterior)

            drone[j].pPosXd[i] = 0.0;  // {0,0,0,0,0,0,0,0,0,0,0,0};    % Desired Pose
            drone[j].pPosXda[i] = 0.0; // {0,0,0,0,0,0,0,0,0,0,0,0};    % Previous Desired Pose
            drone[j].pPosdXd[i] = 0.0; // {0,0,0,0,0,0,0,0,0,0,0,0};    % Desired first derivative Pose

            drone[j].pPosXtil[i] = 0.0;// {0,0,0,0,0,0,0,0,0,0,0,0};    % Posture Error

            drone[j].pSCXr[i] = 0.0;   // {0,0,0,0,0,0,0,0,0,0,0,0};    % Reference pose
        }

        drone[j].pPosX[2] = 0.75;    // % Start Altitude [m] 

        for (int i = 0; i < 4; i++) {
            drone[j].pSCU[i] = 0.0;    // {0,0,0,0};                    % Control Signal
            drone[j].pSCUd[i] = 0.0;   // {0,0,0,0};                    % Desired Control Signal (sent to robot)
            drone[j].pSCWd[i] = 0.0;   // {0,0,0,0};                    % Desired rotor velocity;
        }

        for (int i = 0; i < 6; i++) {
            drone[j].pSCD[i] = 0.0;    // {0,0,0,0,0,0};                % Disturbance Vector
        }
    }
    //---------------  Variables Initialization %%%% End %%%%
} 


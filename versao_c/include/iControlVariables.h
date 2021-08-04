// Control Variables

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
  
void initControlVar(ArDrone *drone){
    
    //--------------- Initializing Variables %%%% Starting %%%%

    int n = 12;
    for (int i = 0; i < n; i++) {
        drone->pPosX[i] = 0.0;   // {0,0,0.75,0,0,0,0,0,0,0,0,0}; % Real Pose (Pose corrente)
        drone->pPosXa[i] = 0.0;  // {0,0,0,0,0,0,0,0,0,0,0,0};    % Previous Pose (Pose anterior)

        drone->pPosXd[i] = 0.0;  // {0,0,0,0,0,0,0,0,0,0,0,0};    % Desired Pose
        drone->pPosXda[i] = 0.0; // {0,0,0,0,0,0,0,0,0,0,0,0};    % Previous Desired Pose
        drone->pPosdXd[i] = 0.0; // {0,0,0,0,0,0,0,0,0,0,0,0};    % Desired first derivative Pose

        drone->pPosXtil[i] = 0.0;// {0,0,0,0,0,0,0,0,0,0,0,0};    % Posture Error

        drone->pSCXr[i] = 0.0;   // {0,0,0,0,0,0,0,0,0,0,0,0};    % Reference pose
    }

    drone->pPosX[2] = 0.75;    // % Start Altitude [m] 

    n = 4;
    for (int i = 0; i < n; i++) {
        drone->pSCU[i] = 0.0;    // {0,0,0,0};                    % Control Signal
        drone->pSCUd[i] = 0.0;   // {0,0,0,0};                    % Desired Control Signal (sent to robot)
        drone->pSCWd[i] = 0.0;   // {0,0,0,0};                    % Desired rotor velocity;
    }


    n = 6;
    for (int i = 0; i < n; i++) {
        drone->pSCD[i] = 0.0;    // {0,0,0,0,0,0};                % Disturbance Vector
    }
 
    //---------------  Variables Initialization %%%% End %%%%
} 


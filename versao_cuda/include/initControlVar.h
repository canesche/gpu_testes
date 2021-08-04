
#pragma unroll
for (int i = 0; i < 12; ++i) {
    pPosX[i] = 0.0;   //# {0,0,0.75,0,0,0,0,0,0,0,0,0}; % Real Pose (Pose corrente)
    pPosXa[i] = 0.0;  //# {0,0,0,0,0,0,0,0,0,0,0,0};    % Previous Pose (Pose anterior)
    pPosXd[i] = 0.0;  //# {0,0,0,0,0,0,0,0,0,0,0,0};    % Desired Pose
    pPosXda[i] = 0.0; //# {0,0,0,0,0,0,0,0,0,0,0,0};    % Previous Desired Pose
    pPosdXd[i] = 0.0; //# {0,0,0,0,0,0,0,0,0,0,0,0};    % Desired first derivative Pose
    pPosXtil[i] = 0.0;//# {0,0,0,0,0,0,0,0,0,0,0,0};    % Posture Error
    pSCXr[i] = 0.0;   //# {0,0,0,0,0,0,0,0,0,0,0,0};    % Reference pose
    //# Pose reference
    pParXr[i]  = 0.0;                  //# {0,0,0,0,0,0,0,0,0,0,0,0}; 
    pParXra[i] = 0.0;                  //# {0,0,0,0,0,0,0,0,0,0,0,0}; 
}

pPosX[2] = 0.75;    // % Start Altitude [m] 

#pragma unroll
for (int i = 0; i < 4; ++i) {
    pSCU[i] = 0.0;         //# {0,0,0,0};  % Control Signal
    pSCUd[i] = 0.0;        //# {0,0,0,0};  % Desired Control Signal (sent to robot)
    pSCWd[i] = 0.0;        //# {0,0,0,0};  % Desired rotor velocity;
    //# Rotor velocities
    pParW[i] = 0.0;        //# {0,0,0,0}; 
}

#pragma unroll
for (int i = 0; i < 6; ++i) {
    //pSCD[i] = 0.0;    // {0,0,0,0,0,0};                % Disturbance Vector
    //# Model disturbances
    pParD[i] = 0.0;                      // {0,0,0,0,0,0};
}

 //# Model disturbances
pParQ[0] = 0.0;
pParQ[1] = 0.0;
pParQ[2] = 0.0;
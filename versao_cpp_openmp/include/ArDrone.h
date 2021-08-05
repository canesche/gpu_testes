
#include<stdio.h>
#include<stdlib.h>
#include<string.h>


// Coefficients with Array of Structure
struct ArDrone {

  /* ================================================= High-level Control Parameters */

  float pPosX[12];      // % Real Pose (Pose corrente)
  float pPosXa[12];     // % Previous Pose (Pose anterior)
  
  float pPosXd[12];     // % Desired Pose
  float pPosXda[12];    // % Previous Desired Pose
  float pPosdXd[12];    // % Desired first derivative Pose 

  float pPosXtil[12];   // % Posture Error

  float pSCU[4];        // % Control Signal
  float pSCUd[4];       // % Desired Control Signal (sent to robot)

  float pSCWd[4];       // % Desired rotor velocity;
  float pSCXr[12];      // % Reference pose

  float pSCD[6];        // % Disturbance Vector


  /* ===================================================== Physical Parameters */

  // % Sample time
  float pParTs;          //                      % ArDrone (1/30 [s])
  float pParTsm;         // pParTsm = pParTs/4;  % Motor
  
  // % Dynamic Model Parameters 
  float pParg;           // % [kg.m/s^2] Gravitational acceleration
  float pParAltmax;

  // % [kg] ArDrone mass
  float pParm;   

  // % [kg.m^2] Moments of Inertia
  float pParIxx; 
  float pParIyy; 
  float pParIzz;  

  float pParIxy;
  float pParIxz;
  float pParIyz;

  // % Rotor Parameters
  float pParr;           // % Reduction Gear
  float pParR;           // % Motor resistance
  float pParJm;          // %2.029585e-5; 
  float pParBm;          // %1.06e-3;
  float pParKm;          // %0.39;
  float pParKb;          // %8e-5;

  float pParCf; 
  float pParCt; 

  // % Low-level PD controller gains
  float pParkdp; 
  float pParkpp;
  float pParkdt;
  float pParkpt;
  float pParkds; 
  float pParkps; 
  float pParkdz; 
  float pParkpz;
  
  // % Propeller coeficients
  float pPark1; 
  float pPark2;            // const float pPark2 = (pParCt/pParCf); 

  float pParuSat[4];
  // pParuSat[0] = (15*M_PI/180);  // % Max roll  angle reference 
  // pParuSat[1] = (15*M_PI/180);  // % Max pitch angle reference 
  // pParuSat[2] = 1;              // % Max altitude rate reference 
  // pParuSat[3] = (100*M_PI/180); // % Max yaw rate reference 

  // % Pose reference
  float pParXr[12];
  float pParXra[12]; 

  // % Motor voltage in hovering stage
  float pParWo;
  
  float pParVo;

  // % Rotor velocities
  float pParW[4];

  // % Model disturbances
  float pParD[6];
  float pParQ[3];

};

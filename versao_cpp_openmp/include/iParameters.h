
// ArDrone Dynamic and Factory Parameters

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void initParameters(struct ArDrone *drone){
    
    //--------------- Initializing the UAV parameters %%%% Start %%%% 

    // % Sample time
    drone->pParTs = 0.0333333333; //                      % ArDrone (1/30 [s])
    drone->pParTsm = 0.008333333;    // pParTsm = pParTs/4;  % Motor

    // % Dynamic Model Parameters 
    drone->pParg = 9.8;           // % [kg.m/s^2] Gravitational acceleration
    drone->pParAltmax = 5000.0;

    // % [kg] ArDrone mass
    drone->pParm = 0.429;   

    // % [kg.m^2] Moments of Inertia
    drone->pParIxx = 2.237568E-3; 
    drone->pParIyy = 2.985236E-3; 
    drone->pParIzz = 4.80374E-3;  

    drone->pParIxy = 0.0;
    drone->pParIxz = 0.0;
    drone->pParIyz = 0.0;

    // % Rotor Parameters
    drone->pParr = 8.625;          // % Reduction Gear
    drone->pParR = 0.6029;         // % Motor resistance
    drone->pParJm = 0.1215;        // %2.029585e-5; 
    drone->pParBm = 3.7400;        // %1.06e-3;
    drone->pParKm = 1.3014E2;      // %0.39;
    drone->pParKb = 1.3014E-3;     // %8e-5;

    drone->pParCf = 8.0480E-6;  
    drone->pParCt = 2.423E-7; 

    // % Low-level PD controller gains
    drone->pParkdp = 0.1; 
    drone->pParkpp = 0.1;
    drone->pParkdt = 0.1;
    drone->pParkpt = 0.1;
    drone->pParkds = 0.05; 
    drone->pParkps = 0.1; 
    drone->pParkdz = 0.05; 
    drone->pParkpz = 5.0;
    
    // % Propeller coeficients
    drone->pPark1 = 0.1785; 
    drone->pPark2 = 2423.0/80840.0;  

    /* % Saturation values (angles) */
    drone->pParuSat[0] = (15.0*M_PI/180.0);           // % Max roll  angle reference 
    drone->pParuSat[1] = (15.0*M_PI/180.0);           // % Max pitch angle reference 
    drone->pParuSat[2] = 1.0;                       // % Max altitude rate reference 
    drone->pParuSat[3] = (100.0*M_PI/180.0);          // % Max yaw rate reference 

    // % Pose reference
    int n = 12;
    for (int i = 0; i < n; i++) {
        drone->pParXr[i]  = 0.0;                  // {0,0,0,0,0,0,0,0,0,0,0,0}; 
        drone->pParXra[i] = 0.0;                  // {0,0,0,0,0,0,0,0,0,0,0,0}; 
    }

    // % Motor voltage in hovering stage 
    //drone->pParWo = sqrt(drone->pParm*drone->pParg/(4*drone->pParCf));
    drone->pParWo = 361.382988; 
    
    //float aaVo = (drone->pParR*drone->pParBm/drone->pParKm + drone->pParKb)*drone->pParWo;
    //float bbVo = (drone->pParR/(drone->pParr*drone->pParKm))*drone->pParCt*drone->pParWo^2;
    drone->pParVo = 6.7318;

    // % Rotor velocities
    n = 4;
    for (int i = 0; i < n; i++) {
        drone->pParW[i] = 0.0;                      // {0,0,0,0}; 
    }

    // % Model disturbances
    n = 6;
    for (int i = 0; i < n; i++) {
        drone->pParD[i] = 0.0;                      // {0,0,0,0,0,0}; 
    }

    drone->pParQ[0] = 0.0;
    drone->pParQ[1] = 0.0;
    drone->pParQ[2] = 0.0;
    


    //--------------- Initialization of UAV parameters %%%% End %%%% 

}

// ArDrone Dynamic and Factory Parameters

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void initParameters(ArDrone *drone, int threads){
    
    //--------------- Initializing the UAV parameters %%%% Start %%%% 
    for (int j = 0; j < threads; ++j) {
        // % Sample time
        drone[j].pParTs = 0.0333333333; //                      % Ardrone[j] (1/30 [s])
        drone[j].pParTsm = 0.008333333;    // pParTsm = pParTs/4;  % Motor

        // % Dynamic Model Parameters 
        drone[j].pParg = 9.8;           // % [kg.m/s^2] Gravitational acceleration
        drone[j].pParAltmax = 5000.0;

        // % [kg] Ardrone[j] mass
        drone[j].pParm = 0.429;   

        // % [kg.m^2] Moments of Inertia
        drone[j].pParIxx = 2.237568E-3; 
        drone[j].pParIyy = 2.985236E-3; 
        drone[j].pParIzz = 4.80374E-3;  

        drone[j].pParIxy = 0.0;
        drone[j].pParIxz = 0.0;
        drone[j].pParIyz = 0.0;

        // % Rotor Parameters
        drone[j].pParr = 8.625;          // % Reduction Gear
        drone[j].pParR = 0.6029;         // % Motor resistance
        drone[j].pParJm = 0.1215;        // %2.029585e-5; 
        drone[j].pParBm = 3.7400;        // %1.06e-3;
        drone[j].pParKm = 1.3014E2;      // %0.39;
        drone[j].pParKb = 1.3014E-3;     // %8e-5;

        drone[j].pParCf = 8.0480E-6;  
        drone[j].pParCt = 2.423E-7; 

        // % Low-level PD controller gains
        drone[j].pParkdp = 0.1; 
        drone[j].pParkpp = 0.1;
        drone[j].pParkdt = 0.1;
        drone[j].pParkpt = 0.1;
        drone[j].pParkds = 0.05; 
        drone[j].pParkps = 0.1; 
        drone[j].pParkdz = 0.05; 
        drone[j].pParkpz = 5.0;
        
        // % Propeller coeficients
        drone[j].pPark1 = 0.1785; 
        drone[j].pPark2 = 2423.0/80840.0;  

        /* % Saturation values (angles) */
        drone[j].pParuSat[0] = (15.0*M_PI/180.0);           // % Max roll  angle reference 
        drone[j].pParuSat[1] = (15.0*M_PI/180.0);           // % Max pitch angle reference 
        drone[j].pParuSat[2] = 1.0;                       // % Max altitude rate reference 
        drone[j].pParuSat[3] = (100.0*M_PI/180.0);          // % Max yaw rate reference 

        // % Pose reference
        int n = 12;
        for (int i = 0; i < n; i++) {
            drone[j].pParXr[i]  = 0.0;                  // {0,0,0,0,0,0,0,0,0,0,0,0}; 
            drone[j].pParXra[i] = 0.0;                  // {0,0,0,0,0,0,0,0,0,0,0,0}; 
        }

        // % Motor voltage in hovering stage 
        //drone[j].pParWo = sqrt(drone[j].pParm*drone[j].pParg/(4*drone[j].pParCf));
        drone[j].pParWo = 361.382988; 
        
        //float aaVo = (drone[j].pParR*drone[j].pParBm/drone[j].pParKm + drone[j].pParKb)*drone[j].pParWo;
        //float bbVo = (drone[j].pParR/(drone[j].pParr*drone[j].pParKm))*drone[j].pParCt*drone[j].pParWo^2;
        drone[j].pParVo = 6.7318;

        // % Rotor velocities
        n = 4;
        for (int i = 0; i < n; i++) {
            drone[j].pParW[i] = 0.0;                      // {0,0,0,0}; 
        }

        // % Model disturbances
        n = 6;
        for (int i = 0; i < n; i++) {
            drone[j].pParD[i] = 0.0;                      // {0,0,0,0,0,0}; 
        }

        drone[j].pParQ[0] = 0.0;
        drone[j].pParQ[1] = 0.0;
        drone[j].pParQ[2] = 0.0;
    
    }

    //--------------- Initialization of UAV parameters %%%% End %%%% 

}
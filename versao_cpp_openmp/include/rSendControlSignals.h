
#include "sDynamicModel.h"

void rSendControlSignals(struct ArDrone *drone){
    
    // % Simulation Mode
    for (int ii = 0; ii < 4; ++ii ) {
        drone->pSCU[ii] = drone->pSCUd[ii];
    }

    // Modelo Dinamico
    sDynamicModel(drone);

    // % Stand-by mode
    for (int ii = 0; ii < 4; ++ii ) {
        drone->pSCUd[ii] = 0.0;
    }

       
}

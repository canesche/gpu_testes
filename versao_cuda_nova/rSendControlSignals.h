
//# Simulation Mode
#pragma unroll
for (int ii = 0; ii < 4; ++ii ) {
    pSCU[ii] = pSCUd[ii];
}

//# Modelo Dinamico
#include "/content/sDynamicModel.h"

//# Stand-by mode
#pragma unroll
for (int ii = 0; ii < 4; ++ii ) {
    pSCUd[ii] = 0.0;
}
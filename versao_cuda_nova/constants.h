
#define tmax 60.0

//# Sample time
#define pParTs 0.03333f         //# ArDrone (1/30 [s])
#define pParTsm 0.00833f        //# pParTsm = pParTs/4; % Motor

//# Dynamic Model Parameters 
#define pParg 9.8                   //# [kg.m/s^2] Gravitational acceleration
#define pParAltmax 5000.0           //# Altura maxima

//# [kg] ArDrone mass
#define pParm 0.429   

//# [kg.m^2] Moments of Inertia
#define pParIxx 2.237568E-3 
#define pParIyy 2.985236E-3 
#define pParIzz 4.80374E-3

#define pParIxy 0.0
#define pParIxz 0.0
#define pParIyz 0.0

//# Rotor Parameters
#define pParr 8.625          //# Reduction Gear
#define pParR 0.6029         //# Motor resistance
#define pParJm 0.1215        //# 2.029585e-5; 
#define pParBm 3.7400        //# 1.06e-3;
#define pParKm 1.3014E2      //# 0.39;
#define pParKb 1.3014E-3     //# 8e-5;

#define pParCf 8.0480E-6  
#define pParCt 2.423E-7 

//# Low-level PD controller gains
#define pParkdp 0.1 
#define pParkpp 0.1
#define pParkdt 0.1
#define pParkpt 0.1
#define pParkds 0.05 
#define pParkps 0.1 
#define pParkdz 0.05 
#define pParkpz 5.0

//# Propeller coeficients
#define pPark1 0.1785 
#define pPark2 0.029973    //# pPark2 = pParCt / pParCf = 2423.0 / 80840.0

//# Motor voltage in hovering stage
#define pParWo 361.382988
#define pParVo 6.7318

//# Saturation values (angles)
__constant__ float pParuSat[4];
__constant__ signed char iv0[9];
__constant__ signed char b[12];

void setup_coef_constant (void) {
    //# h_pParuSat = {Max roll angle ref, Max pitch angle ref, max altitude rate ref, Max yaw rate ref}
    const float h_pParuSat[4] = {15.0*M_PI/180.0, 15.0*M_PI/180.0, 1.0, 100.0*M_PI/180.0};
    const signed char h_iv0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };  //# eye(3,3);
    const signed char h_b[12] = { -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1 };

    CHECK(cudaMemcpyToSymbol(pParuSat, h_pParuSat, 4 * sizeof(float)));
    CHECK(cudaMemcpyToSymbol(iv0, h_iv0, 9 * sizeof(signed char)));
    CHECK(cudaMemcpyToSymbol(b, h_b, 12 * sizeof(signed char)));
}
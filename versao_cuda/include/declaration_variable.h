//# ============================================== High-level Control Parameters 

  float pPosX[12];      //# Real Pose (Pose corrente)
  float pPosXa[12];     //# Previous Pose (Pose anterior)
  
  float pPosXd[12];     //# Desired Pose
  float pPosXda[12];    //# Previous Desired Pose
  float pPosdXd[12];    //# Desired first derivative Pose 

  float pPosXtil[12];   //# Posture Error

  float pSCU[4];        //# Control Signal
  float pSCUd[4];       //# Desired Control Signal (sent to robot)

  float pSCWd[4];       //# Desired rotor velocity;
  float pSCXr[12];      //# Reference pose

  //float pSCD[6];        //# Disturbance Vector

  //# Pose reference
  float pParXr[12];
  float pParXra[12]; 

  //# Rotor velocities
  float pParW[4];

  //# Model disturbances
  float pParD[6];
  float pParQ[3];
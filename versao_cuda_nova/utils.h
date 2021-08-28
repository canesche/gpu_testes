__device__ float rt_atan2(float u0, float u1){
    if (u1 == 0.0) { 
        if (u0 > 0.0) { 
            return 0.5 * M_PI; 
        } 
        else if (u0 < 0.0) { 
            return - 0.5 * M_PI; 
        } 
        else { 
            return 0.0; 
        }
    }
    return atan2(u0, u1); 
}

__device__ void matrixMult(float *hres, float *a, float *b, int m, int n, int p) { 
    float temp;   
    for (int i = 0; i < m; i++){
        for (int j = 0; j < n; j++){
            temp = 0.0;
            for (int k = 0; k < p; k++) {
                temp += a[i*p+k] * b[k*n+j];
            }
            hres[i*n+j] = temp;
        }
    }
}

__device__ void matrixMultn(float *hres, float *a, float *b, int n){ 
    float temp;
    for (int i = 0; i < n; ++i){
        for (int j = 0; j < n; ++j){
            temp = 0.0;
            for (int k = 0; k < n; ++k) {
                temp += a[i*n+k] * b[k*n+j];
            }
            hres[i*n+j] = temp;
        }
    }
}


__device__ void Transpose(float *matrix, float *t_matrix, int m, int n) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
             t_matrix[j * m + i] = matrix[i * n + j];
        }
    }
}

__device__ void invMatrix(float *A, float *inverse, int n) {
    //# Find determinant of A[][]
    float det = 0.0;

    for (int i = 0; i < n; i++){
      det += (A[0*n + i] * (A[1*n+ (i+1)%3] * A[2*n + (i+2)%3] - A[1*n + (i+2)%3] * A[2*n + (i+1)%3]));
    }

    //# printf("Det[A] = %.10f\n\n", det);
    if (det == 0.0){
        printf("Singular matrix, can't find its inverse");
        return;
    }
  
    //# Find adjoint
    float* adj = new float[n*n];

    //# Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for (int i = 0; i < n; i++){
      for (int j = 0; j < n; j++){
        adj[i*n + j] = ((A[(j+1)%3*n + (i+1)%3] * A[(j+2)%3*n + (i+2)%3]) - (A[(j+1)%3*n + (i+2)%3] * A[(j+2)%3*n + (i+1)%3]));
        inverse[i*n + j] = adj[i*n + j]/det;
      }
    }
    delete adj;
}

//----------------------------------------------------- Exibe uma matriz na tela
__device__ void print_matrix(float *v, int m, int n) {

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            printf("%2.4f ", v[i*n+j]); // printf("%2.20f ", v[i*n+j]);
        }
        printf("\n");
    }
    printf("\n");

}
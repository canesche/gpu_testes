
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* # Sumário:
 * 1. Multiplicação de Matrizes: 'matrixMult(float *Result, float *Mat1, float *Mat1, int n)'
 * 2. Multiplicação de Matrizes de ordem diferente: 'void matrixMultMNdebug(float *hres, float *a, float *b, int m, int n, int p)' 
 * 3. Inversa de Matrizes Quadradas: 'bool invMatrix(float *A, float *inverse, int n)'
 * 4. Transposta de Matrizes: 'void Transpose(float *matrix, float *t_matrix, int m, int n)'
 * 5. Exibição de Matrizes: 'void print_matrix(float *v, int m, int n)'
 */

//------------------------------------------ Multiplicação de Matrizes Quadradas
void matrixMult(float *hres, float *a, float *b, int n){ 
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


//---------------------------------------------------- Multiplicação de Matrizes
/* A<m x p> * B<p x n> = C<m x n> */

void matrixMultMNdebug(float *hres, float *a, float *b, int m, int n, int p){ 
    float temp;                                     // 

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


//------------------------------------------------ Inversa de Matrizes Quadradas

// Function to calculate and store inverse, returns false if matrix is singular
bool invMatrix(float *A, float *inverse, int n){
    // Find determinant of A[][]
    float det = 0.0;

    for (int i = 0; i < n; i++){
      det += (A[0*n + i] * (A[1*n+ (i+1)%3] * A[2*n + (i+2)%3] - A[1*n + (i+2)%3] * A[2*n + (i+1)%3]));
    }

    //printf("Det[A] = %.10f\n\n", det);
    
    if (det == 0.0){
        printf("Singular matrix, can't find its inverse");
        return false;
    }
  
    // Find adjoint
    float* adj = new float[n*n];


    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for (int i = 0; i < n; i++){
      for (int j = 0; j < n; j++){
        adj[i*n + j] = ((A[(j+1)%3*n + (i+1)%3] * A[(j+2)%3*n + (i+2)%3]) - (A[(j+1)%3*n + (i+2)%3] * A[(j+2)%3*n + (i+1)%3]));
        inverse[i*n + j] = adj[i*n + j]/det;
      
      }
    }

    delete adj;

    return true;
}



//------------------------------------------------------- Transposta de Matrizes
/* A<m x n> ^T = C<n x m> */
void Transpose(float *matrix, float *t_matrix, int m, int n) {
 
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
             t_matrix[j * m + i] = matrix[i * n + j];
        }
    }
}


//------------------------------------- Pseudo-Inversa de Matrizes Não-quadradas  (Incompleta)
/* A<m x n> ^T = C<n x m> */  // Tranpose
/* A<m x n> * C<n x m> = D<m x m> */  


//----------------------------------------------------- Exibe uma matriz na tela
void print_matrix(float *v, int m, int n) {

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            printf("%2.4f ", v[i*n+j]); // printf("%2.20f ", v[i*n+j]);
        }
        printf("\n");
    }
    printf("\n");

}



//# Cálculo da pseudo-invera de uma matriz não-quadrada
//# G-inv (Artigo):

void ginv(float X[24], float b_ginv[24])
{
  int i0;
  int jA;
  int j;
  int jp1j;
  signed char ipiv[4];
  int iy;
  int jj;
  float C[16];
  float smax;
  int n;
  int i;
  int ix;
  int k;
  float s;

  for (i0 = 0; i0 < 6; i0++) {
    jA = i0 << 2;
    b_ginv[jA] = X[i0];
    b_ginv[1 + jA] = X[i0 + 6];
    b_ginv[2 + jA] = X[i0 + 12];
    b_ginv[3 + jA] = X[i0 + 18];
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (jp1j = 0; jp1j < 4; jp1j++) {
      iy = i0 + (jp1j << 2);
      C[iy] = 0.0;
      smax = 0.0;
      for (jA = 0; jA < 6; jA++) {
        smax += b_ginv[i0 + (jA << 2)] * X[jA + 6 * jp1j];
      }

      C[iy] = smax;
    }

    ipiv[i0] = (signed char)(1 + i0);
  }

  for (j = 0; j < 3; j++) {
    jA = j * 5;
    jj = j * 5;
    jp1j = jA + 2;
    n = 4 - j;
    iy = 0;
    ix = jA;
    smax = fabs(C[jA]);
    for (k = 2; k <= n; k++) {
      ix++;
      s = fabs(C[ix]);
      if (s > smax) {
        iy = k - 1;
        smax = s;
      }
    }

    if (C[jj + iy] != 0.0) {
      if (iy != 0) {
        iy += j;
        ipiv[j] = (signed char)(iy + 1);
        smax = C[j];
        C[j] = C[iy];
        C[iy] = smax;
        ix = j + 4;
        iy += 4;
        smax = C[ix];
        C[ix] = C[iy];
        C[iy] = smax;
        ix += 4;
        iy += 4;
        smax = C[ix];
        C[ix] = C[iy];
        C[iy] = smax;
        ix += 4;
        iy += 4;
        smax = C[ix];
        C[ix] = C[iy];
        C[iy] = smax;
      }

      i0 = (jj - j) + 4;
      for (i = jp1j; i <= i0; i++) {
        C[i - 1] /= C[jj];
      }
    }

    n = 2 - j;
    iy = jA + 4;
    jA = jj;
    for (k = 0; k <= n; k++) {
      smax = C[iy];
      if (C[iy] != 0.0) {
        ix = jj + 1;
        i0 = jA + 6;
        jp1j = (jA - j) + 8;
        for (i = i0; i <= jp1j; i++) {
          C[i - 1] += C[ix] * -smax;
          ix++;
        }
      }

      iy += 4;
      jA += 4;
    }

    if (ipiv[j] != j + 1) {
      iy = ipiv[j] - 1;
      for (k = 0; k < 6; k++) {
        jA = k << 2;
        i = j + jA;
        smax = b_ginv[i];
        jA += iy;
        b_ginv[i] = b_ginv[jA];
        b_ginv[jA] = smax;
      }
    }
  }

  for (j = 0; j < 6; j++) {
    iy = j << 2;
    if (b_ginv[iy] != 0.0) {
      for (i = 2; i < 5; i++) {
        jA = (i + iy) - 1;
        b_ginv[jA] -= b_ginv[iy] * C[i - 1];
      }
    }

    if (b_ginv[1 + iy] != 0.0) {
      for (i = 3; i < 5; i++) {
        b_ginv[(i + iy) - 1] -= b_ginv[1 + iy] * C[i + 3];
      }
    }

    if (b_ginv[2 + iy] != 0.0) {
      for (i = 4; i < 5; i++) {
        b_ginv[iy + 3] -= b_ginv[2 + iy] * C[11];
      }
    }
  }

  for (j = 0; j < 6; j++) {
    iy = j << 2;
    smax = b_ginv[3 + iy];
    if (smax != 0.0) {
      b_ginv[3 + iy] = smax / C[15];
      for (i = 0; i < 3; i++) {
        jA = i + iy;
        b_ginv[jA] -= b_ginv[3 + iy] * C[i + 12];
      }
    }

    smax = b_ginv[2 + iy];
    if (smax != 0.0) {
      b_ginv[2 + iy] = smax / C[10];
      for (i = 0; i < 2; i++) {
        b_ginv[i + iy] -= b_ginv[2 + iy] * C[i + 8];
      }
    }

    smax = b_ginv[1 + iy];
    if (smax != 0.0) {
      b_ginv[1 + iy] = smax / C[5];
      for (i = 0; i < 1; i++) {
        b_ginv[iy] -= b_ginv[1 + iy] * C[4];
      }
    }

    if (b_ginv[iy] != 0.0) {
      b_ginv[iy] /= C[0];
    }
  }
}
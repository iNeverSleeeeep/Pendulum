#ifndef MATRIX_LITE_H
#define MATRIX_LITE_H

/*
 * Lightweight matrix/vector helpers for small control-oriented systems.
 * Intended for fixed, low-dimension operations on STM32-class MCUs.
 */

static void MatLite_VecZero(float *vec, u8 len)
{
    u8 i;

    for (i = 0; i < len; ++i) {
        vec[i] = 0.0f;
    }
}

static void MatLite_VecCopy(float *dst, const float *src, u8 len)
{
    u8 i;

    for (i = 0; i < len; ++i) {
        dst[i] = src[i];
    }
}

static void MatLite_VecAdd(const float *a, const float *b, float *out, u8 len)
{
    u8 i;

    for (i = 0; i < len; ++i) {
        out[i] = a[i] + b[i];
    }
}

static void MatLite_VecSub(const float *a, const float *b, float *out, u8 len)
{
    u8 i;

    for (i = 0; i < len; ++i) {
        out[i] = a[i] - b[i];
    }
}

static void MatLite_VecScale(const float *vec, float gain, float *out, u8 len)
{
    u8 i;

    for (i = 0; i < len; ++i) {
        out[i] = vec[i] * gain;
    }
}

static float MatLite_Dot(const float *a, const float *b, u8 len)
{
    u8 i;
    float sum = 0.0f;

    for (i = 0; i < len; ++i) {
        sum += a[i] * b[i];
    }

    return sum;
}

static void MatLite_MatVecMul(const float *mat,
                              u8 rows,
                              u8 cols,
                              const float *vec,
                              float *out)
{
    u8 r;
    u8 c;

    for (r = 0; r < rows; ++r) {
        float sum = 0.0f;
        for (c = 0; c < cols; ++c) {
            sum += mat[r * cols + c] * vec[c];
        }
        out[r] = sum;
    }
}

static void MatLite_RowVecMul(const float *row,
                              u8 cols,
                              const float *vec,
                              float *out_scalar)
{
    *out_scalar = MatLite_Dot(row, vec, cols);
}

static void MatLite_MatAdd(const float *a,
                           const float *b,
                           float *out,
                           u8 rows,
                           u8 cols)
{
    u8 i;
    u8 total = (u8)(rows * cols);

    for (i = 0; i < total; ++i) {
        out[i] = a[i] + b[i];
    }
}

static void MatLite_MatSub(const float *a,
                           const float *b,
                           float *out,
                           u8 rows,
                           u8 cols)
{
    u8 i;
    u8 total = (u8)(rows * cols);

    for (i = 0; i < total; ++i) {
        out[i] = a[i] - b[i];
    }
}

/*
 * Convenience helper for state-space controller update:
 * x_next = A*x + B*u
 * y      = C*x + D*u
 */
static float MatLite_StateSpaceOutput(const float *C,
                                      u8 state_dim,
                                      const float *x,
                                      const float *D,
                                      u8 input_dim,
                                      const float *u)
{
    return MatLite_Dot(C, x, state_dim) + MatLite_Dot(D, u, input_dim);
}

static void MatLite_StateSpaceStep(const float *A,
                                   u8 state_dim,
                                   const float *x,
                                   const float *B,
                                   u8 input_dim,
                                   const float *u,
                                   float *x_next)
{
    u8 i;
    float ax[16];
    float bu[16];

    for (i = 0; i < state_dim; ++i) {
        ax[i] = 0.0f;
        bu[i] = 0.0f;
    }

    MatLite_MatVecMul(A, state_dim, state_dim, x, ax);
    MatLite_MatVecMul(B, state_dim, input_dim, u, bu);
    MatLite_VecAdd(ax, bu, x_next, state_dim);
}

#endif

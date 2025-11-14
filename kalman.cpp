#include "Kalman.h"
#include <math.h>

// -------------------- CONSTRUCTOR --------------------
Kalman::Kalman() {
    _dt = 0.00025; // 4000 Hz

    // Gyroscope rate noise spectral density (deg/s / sqrt(Hz))
    float sigma_gyro = 0.01f;
    float Q_gyro = sigma_gyro * sigma_gyro * _dt;

    // Accelerometer power spectral density (g / sqrt(Hz))
    float sigma_acc = 0.0003f;
    float R_acc = sigma_acc * sigma_acc * _dt;
    float R_gyro = Q_gyro;

    // -------------------- INITIAL STATE --------------------
    for (int i = 0; i < STATE_SIZE; i++) {
        _state[i] = 0.0f;
        for (int j = 0; j < STATE_SIZE; j++)
            _covariance[i][j] = (i == j) ? Q_gyro : 0.0f;
    }

    // -------------------- MEASUREMENT MATRIX H --------------------
    for (int i = 0; i < MEAS_SIZE; i++)
        for (int j = 0; j < STATE_SIZE; j++)
            _H[i][j] = 0.0f;

    // Map accelerometer and gyro to angles
    _H[0][0] = 1.0f; // ax -> θx
    _H[1][1] = 1.0f; // ay -> θy
    _H[3][0] = 1.0f; // gx -> θx
    _H[4][1] = 1.0f; // gy -> θy
    _H[5][2] = 1.0f; // gz -> θz

    // -------------------- MEASUREMENT NOISE R --------------------
    for (int i = 0; i < MEAS_SIZE; i++)
        for (int j = 0; j < MEAS_SIZE; j++)
            _R[i][j] = 0.0f;

    _R[0][0] = R_acc;
    _R[1][1] = R_acc;
    _R[2][2] = R_acc;

    _R[3][3] = R_gyro;
    _R[4][4] = R_gyro;
    _R[5][5] = R_gyro;

    _Q = Q_gyro;
}

// -------------------- TEMPLATE FUNCTIONS --------------------
template<int ROWS, int COLS>
void Kalman::transpose(float in[ROWS][COLS], float out[COLS][ROWS]){
    for (int i = 0; i < ROWS; i++)
        for (int j = 0; j < COLS; j++)
            out[j][i] = in[i][j];
}

template<int M, int N, int P>
void Kalman::matMultiply(float A[M][N], float B[N][P], float C[M][P]){
    for (int i = 0; i < M; i++){
        for (int j = 0; j < P; j++){
            C[i][j] = 0;
            for (int k = 0; k < N; k++)
                C[i][j] += A[i][k] * B[k][j];
        }
    }
}

template<int ROWS, int COLS>
void Kalman::matAdd(float A[ROWS][COLS], float B[ROWS][COLS], float C[ROWS][COLS]){
    for (int i = 0; i < ROWS; i++)
        for (int j = 0; j < COLS; j++)
            C[i][j] = A[i][j] + B[i][j];
}

// -------------------- INVERSE --------------------
bool Kalman::invert6x6(float A[MEAS_SIZE][MEAS_SIZE], float A_inv[MEAS_SIZE][MEAS_SIZE]){
    for (int i = 0; i < MEAS_SIZE; i++)
        for (int j = 0; j < MEAS_SIZE; j++)
            A_inv[i][j] = (i == j) ? 1.0f : 0.0f;

    for (int i = 0; i < MEAS_SIZE; i++){
        float pivot = A[i][i];
        if (fabs(pivot) < 1e-6) return false;

        for (int j = 0; j < MEAS_SIZE; j++){
            A[i][j] /= pivot;
            A_inv[i][j] /= pivot;
        }

        for (int k = 0; k < MEAS_SIZE; k++){
            if (k == i) continue;
            float f = A[k][i];
            for (int j = 0; j < MEAS_SIZE; j++){
                A[k][j] -= f * A[i][j];
                A_inv[k][j] -= f * A_inv[i][j];
            }
        }
    }
    return true;
}

// -------------------- COMPUTE KALMAN GAIN --------------------
bool Kalman::computeKalmanGain(float K[STATE_SIZE][MEAS_SIZE]){
    float H_T[STATE_SIZE][MEAS_SIZE];
    float temp1[STATE_SIZE][MEAS_SIZE];
    float temp2[MEAS_SIZE][MEAS_SIZE];
    float temp3[MEAS_SIZE][MEAS_SIZE];
    float temp4[MEAS_SIZE][MEAS_SIZE];

    transpose<MEAS_SIZE, STATE_SIZE>(_H, H_T);
    matMultiply<STATE_SIZE, STATE_SIZE, MEAS_SIZE>(_covariance, H_T, temp1);
    matMultiply<MEAS_SIZE, STATE_SIZE, MEAS_SIZE>(_H, temp1, temp2);
    matAdd<MEAS_SIZE, MEAS_SIZE>(temp2, _R, temp3);

    if(!invert6x6(temp3, temp4)) return false;

    matMultiply<STATE_SIZE, MEAS_SIZE, MEAS_SIZE>(temp1, temp4, K);
    return true;
}

// -------------------- PREDICT --------------------
void Kalman::predict(float gyro[STATE_SIZE]){
    for(int i = 0; i < STATE_SIZE; i++)
        _state[i] += gyro[i] * _dt;

    for(int i = 0; i < STATE_SIZE; i++)
        _covariance[i][i] += _Q;
}

// -------------------- UPDATE --------------------
void Kalman::update(float measurement[MEAS_SIZE]){
    float K[STATE_SIZE][MEAS_SIZE];
    if(!computeKalmanGain(K)) return;

    // Innovation y = z - H*x
    float y[MEAS_SIZE];
    for(int i = 0; i < MEAS_SIZE; i++){
        y[i] = measurement[i];
        for(int j = 0; j < STATE_SIZE; j++)
            y[i] -= _H[i][j] * _state[j];
    }

    // Update state x = x + K*y
    for(int i = 0; i < STATE_SIZE; i++)
        for(int j = 0; j < MEAS_SIZE; j++)
            _state[i] += K[i][j] * y[j];

    // Update covariance P = (I - K*H)*P
    float KH[STATE_SIZE][STATE_SIZE] = {0};
    for(int i = 0; i < STATE_SIZE; i++)
        for(int j = 0; j < STATE_SIZE; j++)
            for(int k = 0; k < MEAS_SIZE; k++)
                KH[i][j] += K[i][k] * _H[k][j];

    float I_KH[STATE_SIZE][STATE_SIZE] = {0};
    for(int i = 0; i < STATE_SIZE; i++)
        for(int j = 0; j < STATE_SIZE; j++)
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];

    float newCov[STATE_SIZE][STATE_SIZE] = {0};
    for(int i = 0; i < STATE_SIZE; i++)
        for(int j = 0; j < STATE_SIZE; j++)
            for(int k = 0; k < STATE_SIZE; k++)
                newCov[i][j] += I_KH[i][k] * _covariance[k][j];

    for(int i = 0; i < STATE_SIZE; i++)
        for(int j = 0; j < STATE_SIZE; j++)
            _covariance[i][j] = newCov[i][j];
}

// -------------------- GET STATE --------------------
void Kalman::getState(float state_out[STATE_SIZE]){
    for(int i = 0; i < STATE_SIZE; i++)
        state_out[i] = _state[i];
}

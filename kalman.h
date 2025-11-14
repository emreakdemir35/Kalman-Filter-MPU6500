#ifndef KALMAN_H
#define KALMAN_H

#define STATE_SIZE 3
#define MEAS_SIZE 6

class Kalman {
public:
    Kalman();  // default constructor

    void predict(float gyro[STATE_SIZE]);
    void update(float measurement[MEAS_SIZE]);
    void getState(float state_out[STATE_SIZE]);

private:
    float _dt;
    float _Q;
    float _state[STATE_SIZE];
    float _covariance[STATE_SIZE][STATE_SIZE];
    float _H[MEAS_SIZE][STATE_SIZE];
    float _R[MEAS_SIZE][MEAS_SIZE];

    // Matrix helper functions
    template<int ROWS, int COLS>
    void transpose(float in[ROWS][COLS], float out[COLS][ROWS]);
    template<int M, int N, int P>
    void matMultiply(float A[M][N], float B[N][P], float C[M][P]);
    template<int ROWS, int COLS>
    void matAdd(float A[ROWS][COLS], float B[ROWS][COLS], float C[ROWS][COLS]);
    bool invert6x6(float A[MEAS_SIZE][MEAS_SIZE], float A_inv[MEAS_SIZE][MEAS_SIZE]);
    bool computeKalmanGain(float K[STATE_SIZE][MEAS_SIZE]);
};

#endif

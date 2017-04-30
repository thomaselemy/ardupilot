#include "ExtendedKalmanFilter.h"
#include "AP_Math/matrixN.h"

template <uint8_t N, uint8_t M, uint8_t L>
void ExtendedKalmanFilter<N,M,L>::reset(const VectorN<float,N> &x, const MatrixN<float,N> &p, const MatrixN<float,N> q, float r)
{
    P = p;
    X = x;
    Q = q;
    R = r;
}


template <uint8_t N, uint8_t M, uint8_t L>
void ExtendedKalmanFilter<N,M,L>::update(float zt, float Vx, float Vy)
{
    MatrixN<float,N> tempM;
    VectorN<float,N> H;
    VectorN<float,N> P12;
    VectorN<float,N> K;
    VectorN<float,L> input;
    VectorN<float,M> z;

    input[0] = Vx;
    input[1] = Vy;

    z[0] = zt;

    // LINE 28
    // Estimate new state from old.

    _stateFunc(X,input);

    // LINE 33
    // Update the covariance matrix
    // P = A*ekf.P*A'+ekf.Q;
    // We know A is identity so
    // P = ekf.P+ekf.Q;
    P += Q;

    // What measurement do we expect to receive in the estimated
    // state
    // LINE 37
    // [z1,H] = ekf.jacobian_h(x1);
    VectorN<float,M> z1;
    _measFunc(X,H,z1);

    // LINE 40
    // P12 = P * H';
    P12.mult(P, H); //cross covariance 
    
    // LINE 41
    // Calculate the KALMAN GAIN
    // K = P12 * inv(H*P12 + ekf.R);                     %Kalman filter gain
    K = P12 * 1.0 / (H * P12 + R);

    // Correct the state estimate using the measurement residual.
    // LINE 44
    // X = x1 + K * (z - z1);
    X += K * (z - z1)[0];

    // Correct the covariance too.
    // LINE 46
    // NB should be altered to reflect Stengel
    // P = P_predict - K * P12';
    tempM.mult(K, P12);
    P -= tempM;
    
    P.force_symmetry();
}

template void ExtendedKalmanFilter<4,1,2>::reset(const VectorN<float,4> &x, const MatrixN<float,4> &p, const MatrixN<float,4> q, float r);
template void ExtendedKalmanFilter<4,1,2>::update(float zt, float Vx, float Vy);

/*
Extended Kalman Filter class by Sam Tabor, 2013.
* http://diydrones.com/forum/topics/autonomous-soaring
* Set up for identifying thermals of Gaussian form, but could be adapted to other
* purposes by adapting the equations for the jacobians.
*/

#pragma once

#include <AP_Math/matrixN.h>

template <uint8_t N, uint8_t M, uint8_t L>
class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter(void (*measFunc)(VectorN<float,N> X, VectorN<float,N> &A, VectorN<float,M> &z1), 
                         void (*stateFunc)(VectorN<float,N> &X, VectorN<float,L> inputs)) : _measFunc(measFunc), _stateFunc(stateFunc) {}
    void (*_measFunc)(VectorN<float,N> X, VectorN<float,N> &A, VectorN<float,M> &z1);
    void (*_stateFunc)(VectorN<float,N> &X, VectorN<float,L> inputs);
    VectorN<float,N> X;
    MatrixN<float,N> P;
    MatrixN<float,N> Q;
    float R;
    void reset(const VectorN<float,N> &x, const MatrixN<float,N> &p, const MatrixN<float,N> q, float r);
    void update(float z, float Vx, float Vy);
};

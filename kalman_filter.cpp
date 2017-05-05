#include "kalman_filter.h"

void KalmanFilter::PredictionStep()
{
   // x[k] = F[k]*x[k-1] + B[k]*u[k]
   // P[k] = F[k]*P[k-1]*F[k]^T + Q[k]
   predictX = F * x + B * u;
   predictP = F * P * F.transpose() + Q;

   return;
}

void KalmanFilter::UpdateStep()
{
   // K'    = P[k]*h[k]^T * (H[k]*P[k]*H[k]^T + R[k])^-1
   K_prime = predictP * H.transpose() * ((H * predictP * H.transpose() + R)^-1)

   // x[k]' = x[k] + K'(z[k]-H[k]*z[k])
   // P[k]' = P[k] - K'*H[k]*P[k]
   x_prime = predictX + K_prime * (z - (H * z));
   P_prime = predictP - K_prime * H * predictP;

   //set the new updates to the current position for the next iteration
   x = x_prime;
   P = P_prime;
   return;
}

void KalmanFilter::getState()
{

   return;
}


void KalmanFilter::setSensorParameters()
{

   return;
}

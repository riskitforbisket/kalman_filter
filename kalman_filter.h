#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <ecl/formatters.hpp>
#include <ecl/linear_algebra.hpp>

//current state = k-1
//next state = k

using namespace ecl::linear_algebra

class KalmanFilter
{

   //state position(mean) and its covariance(uncertainty)
   //X = [position velocity]
   Eigen::Matrix2d Xproj;
   Eigen::Matrix2d Pproj;
   Eigen::Matrix2d Xcur;
   Eigen::Matrix2d Pcur;
   Eigen::Matrix2d Xupd;
   Eigen::Matrix2d Pupd;

   //prediction matrix to predict foward in time
   Eigen::Matrix2d F;

   //control matrix B and control vector u
   //these are for the commands given to system "move foward 3 meters"
   Eigen::Matrix2d B;
   Eigen::Matrix2d u;

   //external system noise factor
   Eigen::Matrix2d Q;

   //sensor input variables from imu or encoders
   //this is expected state and uncertainty
   //H[k]
   Eigen::Matrix2d H; //sensor readings
   Eigen::Matrix2d z; //sensor readings, mean
   Eigen::Matrix2d sensor_u;
   Eigen::Matrix2d sensor_P;

   //kalman gain
   Eigen::Matrix2d K

   //members
   public:
      KalmanFilter();
      ~KalmanFilter();
      void InitializeStates();
      void PredictionStep();
      void UpdateStep();
      void getState();
      void setSensorParameters();

}

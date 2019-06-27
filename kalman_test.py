import numpy as np
from kalman_filter import KalmanFilter
import matplotlib.pyplot as plt

if __name__ == "__main__":
  KF = KalmanFilter(3, 0.05)
  
  while(1):
     KF.predict()
     KF.update()

import numpy as np
from kalman_filter import KalmanFilter
import matplotlib.pyplot as plt

if __name__ == "__main__":
  kf = KalmanFilter(3, 1.0)
  fileData = open("fakeTelemetry.txt")
  while(1):
    #set controlVectorInput and SensorReadings from file
    control, sensor = 
    
    #predict the future
    kf.predict()
    
    #simulate acceleration data
    kf.sensorReadings = np.ones(n)
    
    
    kf.update()

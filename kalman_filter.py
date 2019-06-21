import numpy as np

class KalmanFilter:

    #update rate is in hertz
    def __init__(dim=2, updateRate=5):
    
        self.timeDelta = 1.0/updateRate
    
        if dim == 2:
            # stateVector = [x, y, vx, vy, ax, ay]
            self.stateVector = np.array([[0, 0, 0, 0, 0, 0]])
        elif dim == 3:
            # stateVector = [x, y, z, vx, vy, vz, ax, ay, az]
            self.stateVector = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0]])
        else:
            print("incorrect dimensions for filter")
        return
        
    def predict():
    
        return
        
    def update():
    
        return

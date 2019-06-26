import numpy as np

class KalmanFilter:

    #update rate is in hertz
    def __init__(dim=2, updateRate=5):
    
        self.dt = 1.0/updateRate
    
        if dim == 2:
            # stateVector = [x, y, vx, vy]
            self.stateVector = np.array([[0, 0, 0, 0]])
            
            # controlVector(u) = [ax, ay]
            self.controlVector = np.array([0, 0])
            
            # stateTransitionMatrix(A)
            self.A = np.array([[1, 0, self.dt],[0, 1, , 0, self.dt],[0,0,1,0],[0,0,0,1]])
            
            #controlTransitionMat(B)
            self.B = np.array([[0.5*(self.dt**2), 0],[0, 0.5*(self.dt**2)],[self.dt,0],[0,self.dt]])
        
        elif dim == 3:
            # stateVector (x)= [x, y, z, vx, vy, vz]
            self.stateVector = np.array([[0, 0, 0, 0, 0, 0]])
            
            # controlVector (u) = [ax, ay, az]
            self.controlVector = np.array([0, 0, 0])
            
            # stateTransitionMatrix (A)
            self.A = np.eye(6)
            self.A[0,3] = self.dt
            self.A[0,4] = self.dt
            self.A[0,5] = self.dt
            
            # controlTransitionMAtrix(B)
            self.B = np.zeros((6,3))
            self.B[0,0] = 0.5*(self.dt**2)
            self.B[1,1] = 0.5*(self.dt**2)
            self.B[2,2] = 0.5*(self.dt**2)
            self.B[3,0] = self.dt
            self.B[4,1] = self.dt
            self.B[5,2] = self.dt
        else:
            print("incorrect dimensions for filter")
        return
        
    def predict():
    
        return
        
    def update():
        #objective: xk = A*xk-1 + b*uk + q(noise)
        return

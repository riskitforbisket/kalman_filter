import numpy as np

#P = state covariance matrix (error in estimate)
#Q = process noise covariance matrix (keeps
#   the state cov. from becoming too small or 0)
#R = measurement cov matrix (error in measurement)
#K = kalman gain
    #if R -> 0 then K -> 1
    #if R -> large then K -> 0
    #if P -> 0 then measurements are mostely ignored.

class KalmanFilter:

    #update rate is in hertz
    def __init__(dim=2, updateRate=5):
    
        self.dt = 1.0/updateRate
    
        if dim == 2:
            # stateVector = [x, y, vx, vy]
            self.stateVector      = np.array([[0, 0, 0, 0]])
            self.stateVectorPrior = np.array([[0, 0, 0, 0]])
            
            # sensorVector (H), this is a vector of sensor
            # measurements that make up the stateVector
            self.H = np.array([[0, 0, 0, 0]])
            
            # controlVector(u) = [ax, ay]
            self.controlVector = np.array([0, 0])
            
            # stateTransitionMatrix(A)
            self.A = np.array([[1, 0, self.dt],[0, 1, , 0, self.dt],[0,0,1,0],[0,0,0,1]])
            
            #controlTransitionMat(B)
            self.B = np.array([[0.5*(self.dt**2), 0],[0, 0.5*(self.dt**2)],[self.dt,0],[0,self.dt]])
            
            # Uncertainty Matrix
            self.P = np.ones((4,4))*0.5
        
            #kalmain Gain
            self.K = np.ones((4,4))*0.5
            
            #C, H
            self.C = np.eye(4)
            self.H = np.eye(4)
            
            # R = sensor noise matrix
            # Q = action uncertainty
            self.R = np.eye(4)
            self.Q = np.eye(4)*0.01
        elif dim == 3:
            # stateVector (x)= [x, y, z, vx, vy, vz]
            self.stateVector = np.array([[0, 0, 0, 0, 0, 0]])
            
            # sensorVector (H), this is a vector of sensor
            # measurements that make up the stateVector
            self.H = np.array([[0, 0, 0, 0, 0, 0]])
            
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
            
            # Uncertainty Matrix
            self.P      = np.ones((6,6))*0.5
            self.Pprior = np.ones((6,6))*0.5
        
            #kalmain Gain
            self.K = np.ones((6,6))*0.5
            
            #C, H
            self.C = np.eye(6)
            self.H = np.eye(6)
            
            # R = sensor noise matrix
            # Q = action uncertainty
            self.R = np.eye(6)
            self.Q = np.eye(6)*0.01
        else:
            print("incorrect dimensions for filter")
        return
        
    def predict():
        #xk = A*xk-1 + b*uk + q(noise)
        #P = A*Pprior*A.T + Qk
        self.stateVector = self.A@self.stateVectorPrior + self.B@self.controlVector + self.q
        self.P = self.A@self.Pprior@self.A.T + self.Q
        return
        
    def update():
        self.K = (self.P@self.H.T)@np.linalg.inverse(self.H@self.P@self.H.T + self.R)
        return

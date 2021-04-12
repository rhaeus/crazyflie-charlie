#!/usr/bin/env python
import rospy
import numpy as np
from aruco_msgs.msg import MarkerArray
import tf.transformations 


class Kalman:
    #https://github.com/xaedes/ROS-Kalman-Filter-for-IMU/blob/master/scripts/kalman.py
    #https://en.wikipedia.org/wiki/Kalman_filter
    def __init__(self,xcord=[10,10,1],P =np.identity(3),Q=np.identity(3),R=np.identity(3)*0.1):
        self.x = np.array(xcord)
        self.P = P                  # P = covariance of estimate
        self.Q = Q                  # Q = covariance of process noise
        self.R = R                  # R = covariance of measurment noise
        self.A = np.identity(3)     # A = state transition model, identity 
        self.C = np.ones(3)         # C = Observation model
        self.xbar = None
        self.Pbar = None
        self.K = None
        self.verbose = 0
    
    def predict(self):
        
        self.xbar = np.matmul(self.A,self.x) # no motion model B*u 
        self.Pbar = np.matmul(self.A,np.matmul(self.P,self.A.T)) + self.Q

        if self.verbose == 1:
            print("---------------------------")
            print("---------New reading-------")
            print("---------------------------")
            print("Q")
            print(self.Q)
            print("predict")
            print(self.xbar)
            print(self.Pbar)
            print("")

    def Kgain(self):
        # method 1
        #S = np.linalg.inv(np.dot(self.C,np.dot(self.Pbar,np.transpose(self.C)))+self.R)
        #self.K = np.dot(self.Pbar,np.dot(np.transpose(self.C),S))

        # method 2
        #S2 = np.linalg.inv(np.dot(np.dot(self.C,self.Pbar),np.transpose(self.C))+self.R)
        #K2 = np.dot(np.dot(self.Pbar,np.transpose(self.C)),S)

        # method 3
        S= np.linalg.inv(np.matmul(self.C,np.matmul(self.Pbar,self.C.T))+self.R)
        self.K = np.matmul(self.Pbar,np.matmul(self.C.T,S))

        # method 4
        #S = self.C*self.Pbar*self.C.T+self.R 
        #self.K = np.matmul(np.ones(3),self.Pbar*self.C.T*np.linalg.inv(S))

        if self.verbose == 1:
            print("Kalman Gain")
            print(self.K)
            print(" ")
        

    def update(self,msg):
        xt = msg.transform.translation.x
        yt = msg.transform.translation.y
        angles = tf.transformations.euler_from_quaternion([msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w])
        Z = np.array([xt,yt,angles[2]]) # Z = measurement

        ybar = Z-self.C*self.x
        if np.linalg.norm(ybar[0:2]) >1:
            print("OUTLIER")

        self.x = self.x + self.K*ybar
        self.P = (np.identity(3)-self.K*self.C)*self.Pbar

        
        msg.transform.translation.x = self.x[0]
        msg.transform.translation.y = self.x[1]

        x,y,z,w = tf.transformations.quaternion_from_euler(0,0,self.x[2])
        
        msg.transform.rotation.x = x
        msg.transform.rotation.y = y
        msg.transform.rotation.z = z
        msg.transform.rotation.w = w

        if self.verbose == 1:

            print("update")
            print("self.x")
            print(self.x)
            print("self.P")
            print(self.P)
            print(" ")

        return msg



def main():
    A = np.random.rand(3,3)
    B = np.random.rand(3,1)
    print(np.ones(3))

    C = np.dot(A,B)
    C2 = np.matmul(A,B)
    C3 = A*B


 


if __name__ == "__main__":
    main()

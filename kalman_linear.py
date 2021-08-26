#!/usr/bin/env python

'''
Very simple linear Kalman filter that estimates a single value Xest. 
The value is measured as Z and the value is changed by control input u. 
As the control input either decreases or increases the value X, noisy 
measurements for Z arrive. Instead of relying on noisy measurements
the filter can be used to estimate the true value of Xest given the
known change in time dt and the known control inputs u. 

Import the KalmanLinear class from this module and use it to predict
the state of a single value. Values within the module will need to 
be customized for your own application (A,IR,DR,H,Q,R). 
'''

class KalmanLinear(object):
    def execute(self,Z,Xe,Pe,u,dt):
        '''
        inputs:
        Z = measurement
        Xe = initial/previous state estimate
        Pe = initial/previous covariance estimate
        u = control input [increase, decrease]
        dt = timestep

        outputs:
        Xest = state estimate advanced by dt
        Pest = state covariance estimate advanced by dt
        '''
        A = 1
        IR = 2.9 # rate of increase
        DR = 1.2 # rate of decrease
        B = [IR*dt, DR*-dt]        
        Xp0 = Xe # initial state prediction
        Pp0 = Pe # initial covariance matrix (ic standard deviations squared)
        H = 1 # measurement matrix
        Q = 0.001 # process noise matrix
        R = 3.7 # measurement-noise matrix, obtained from characterizing actual sensor noise
        Xp = A*Xp0+(B[0]*u[0]+B[1]*u[1]) # state prediction
        Zp = H*Xp # measurement prediction
        Re = Z-Zp # measurement residual
        Pp = A*Pp0*A+Q # state prediction covariance
        S = H*Pp*H+R # measurement prediction covariance
        K = Pp*H*(S**-1) # Kalman Filter gain
        Xest = Xp+K*Re # updated state estimate
        Pest = Pp-K*S*K # updated state covariance

        return Xest, Pest

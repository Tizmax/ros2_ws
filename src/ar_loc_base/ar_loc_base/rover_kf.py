#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos, atan2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import threading
from rover_driver_base.rover_kinematics import RoverKinematics
from ar_loc_base.rover_odo import RoverOdo, DeltaOdo

class DeltaKF(DeltaOdo):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node,initial_pose, initial_uncertainty)
        self.X = mat(vstack(initial_pose))
        self.P = mat(diag(initial_uncertainty))
        self.ellipse_pub = node.create_publisher(Marker,"~/ellipse",1)
        self.pose_with_cov_pub = node.create_publisher(PoseWithCovarianceStamped,"~/pose_with_covariance",1)

    def getRotationFromWorldToRobot(self):
        return self.getRotation(-self.X[2,0])

    def predict_delta(self, logger, DeltaX, CovDeltaX, lock=True):
        if lock:
            self.lock.acquire()
        
        # TODO: Implement Kalman prediction here
        # Xk = f(Xk-1, DeltaX)
        # Xk = Xk-1  + Rtheta DeltaX
        # Uncertainty in Xk-1, S
        # Cov(F(X,Y)) = dF/dX Cov(X) dF/dX.T + dF/dY Cov(Y) dF/dY.T

        # Kalman prediction
        theta = self.X[2,0]
        c, s = cos(theta), sin(theta)
        Rtheta = mat([[c, -s, 0], 
                      [s,  c, 0],
                      [0,  0, 1]])

        # Jacobian w.r.t. X
        # Xk = Xk-1 + R(theta) * DeltaX
        dx, dy = DeltaX[0,0], DeltaX[1,0]
        F = mat([[1, 0, -s*dx - c*dy],
                 [0, 1,  c*dx - s*dy],
                 [0, 0, 1]])

        self.X = self.X + Rtheta @ DeltaX
        self.P = F @ self.P @ F.T + Rtheta @ CovDeltaX @ Rtheta.T

        if lock:
            self.lock.release()
        return (self.X,self.P)

    def update_ar(self, logger, Z, L, uncertainty):
        self.lock.acquire()
        # TODO
        logger.info("Update: L="+str(L.T)+" X="+str(self.X.T))
        # TODO
        
        Rtheta = self.getRotationFromWorldToRobot()

        Zpred = Rtheta @ (L - self.X[0:2,0]) 
        # cos lx - x - sin ly - y
        # sin lx - x + cos ly - y

        # -cos , sin
        # -sin , -cos

        theta = -self.X[2,0]
        H = mat([[-cos(theta),  sin(theta), sin(theta)*(L[0,0]-self.X[0,0]) + cos(theta)*(L[1,0]-self.X[1,0])],
                 [-sin(theta), -cos(theta),-cos(theta)*(L[0,0]-self.X[0,0]) + sin(theta)*(L[1,0]-self.X[1,0])]])

        K = self.P @ H.T @ inv(H @ self.P @ H.T + uncertainty*eye(2))

        self.X = self.X + K @ (Z - Zpred)
        self.P = (eye(3) - K @ H) @ self.P
        self.lock.release()

        return (self.X,self.P)

    def update_compass(self, logger, Z, uncertainty):
        self.lock.acquire()
        # TODO
        logger.info("Update: S="+str(Z)+" X="+str(self.X.T))
        # Implement kalman update using compass here
        # TODO
        H = mat([[0, 0, 1]])
        K = self.P @ H.T @ inv(H @ self.P @ H.T + uncertainty)
        self.X = self.X + K * (self.normAngle(Z - self.X[2,0]))
        self.P = (eye(3) - K @ H) @ self.P
        self.lock.release()
        return (self.X,self.P)

    def publish(self, pose_pub, odom_pub, target_frame, stamp, child_frame):
        pose_simple = super().publish(pose_pub, odom_pub, target_frame, stamp, child_frame)
        pose = PoseWithCovarianceStamped()
        pose.header = pose_simple.header
        pose.pose.pose = pose_simple.pose
        C = [0.]*36
        C[ 0] = self.P[0,0]; C[ 1] = self.P[0,1]; C[ 5] = self.P[0,2]
        C[ 6] = self.P[1,0]; C[ 7] = self.P[1,1]; C[11] = self.P[1,2]
        C[30] = self.P[2,0]; C[31] = self.P[2,1]; C[35] = self.P[2,2]
        pose.pose.covariance = C
        self.pose_with_cov_pub.publish(pose)

        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose.pose
        marker.scale.x = 3*sqrt(self.P[0,0])
        marker.scale.y = 3*sqrt(self.P[1,1]);
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        self.ellipse_pub.publish(marker)


class RoverKF(DeltaKF):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node,initial_pose,initial_uncertainty)
        self.kinematics=RoverKinematics()

    def predict(self, logger, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.kinematics.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return (self.X, self.P)
        # print "-"*32
        # then compute odometry using least square
        iW = self.kinematics.prepare_inversion_matrix(drive_cfg)
        S = self.kinematics.prepare_displacement_matrix(self.kinematics.motor_state,motor_state,drive_cfg)
        self.kinematics.motor_state.copy(motor_state)
        # TODO 
        DeltaX = iW @ S
        CovDeltaX = encoder_precision**2 * iW @ iW.T

        # Implement Kalman prediction here
        self.predict_delta(logger, DeltaX, CovDeltaX, False)
        self.lock.release()
        return (self.X,self.P)


        


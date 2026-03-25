#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from numpy.linalg import inv
from math import pi, sin, cos,hypot,sqrt
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import threading
from rover_driver_base.rover_kinematics import RoverKinematics, RoverMotors
from ar_loc_base.rover_odo import RoverOdo



class MappingKF(RoverOdo):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node,initial_pose,initial_uncertainty)
        self.motor_state = RoverMotors()
        self.lock = threading.Lock()
        self.X = np.mat(np.vstack(initial_pose))
        self.P = np.mat(np.diag(initial_uncertainty))
        self.idx = {}
        self.marker_pub = node.create_publisher(MarkerArray,"~/landmarks",1)

    def predict_rover(self, logger, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return (self.X, self.P)
        # print("-"*32)
        # then compute odometry using least square
        iW = self.kinematics.prepare_inversion_matrix(drive_cfg)
        S = self.kinematics.prepare_displacement_matrix(self.motor_state,motor_state,drive_cfg)
        self.motor_state.copy(motor_state)
        
        # Implement Kalman prediction here
        # Compute the update in the body frame and the resulting uncertainty in the body frame
        DeltaX = iW*S
        DeltaP = np.zeros((3,3))

        DeltaP = encoder_precision**2 * iW @ iW.T
        
        self.lock.release()
        return self.predict_delta(logger,DeltaX,DeltaP)

    def predict_delta(self, logger, DeltaX, DeltaP):
        self.lock.acquire()
        # Update the state using the provided displacement, but we only need to deal with a subset of the state
        # Assumption: deltaX and deltaP are defined in the body frame and need to be rotated to account for the jacobian 
        # of the transfer function
        # TODO
        theta = self.X[2,0]
        Rtheta = np.mat([[cos(theta), -sin(theta), 0], 
                      [sin(theta),  cos(theta), 0],
                      [         0,           0, 1]]);
         
        self.X[0:3,0] = self.X[0:3,0] + Rtheta @ DeltaX

        dx, dy = DeltaX[0,0], DeltaX[1,0]
        c, s = cos(theta), sin(theta)
        F = np.mat([[1, 0, -s*dx - c*dy],
                     [0, 1,  c*dx - s*dy],
                     [0, 0, 1]])
        
        self.P[0:3,0:3] = F @ self.P[0:3,0:3] @ F.T + Rtheta @ DeltaP @ Rtheta.T

        self.lock.release()
        return (self.X,self.P)


    def update_ar(self, logger, Z, id, uncertainty):
        # Z = vstack([x,y])
        self.lock.acquire()
        # TODO
        logger.info("Update: Z="+str(Z.T)+" X="+str(self.X.T)+" Id="+str(id))
        print("Update: Z="+str(Z.T)+" X="+str(self.X.T)+" Id="+str(id))
        # Update the full state self.X and self.P based on landmark id
        # be careful that this might be the first time that id is observed
        # TODO


        if id not in self.idx:
            x = self.X[0,0]
            y = self.X[1,0]
            theta = self.X[2,0]


            c = cos(theta)
            s = sin(theta)
            zx = Z[0,0]
            zy = Z[1,0]
            
            lx = x + c*zx - s*zy
            ly = y + s*zx + c*zy
            
            # Add to state vector
            n = self.X.shape[0]
            self.X = np.vstack((self.X, [[lx], [ly]]))
            self.idx[id] = n
            
            Jr = np.mat([[1, 0, -s*zx - c*zy],
                         [0, 1,  c*zx - s*zy]])
            
            Jz = np.mat([[c, -s],
                         [s,  c]])
            
            # P_new = [[P_old,   P_old[:,0:3] @ Jr.T],
            #          [Jr @ P_old[0:3,:], Jr @ P_old[0:3,0:3] @ Jr.T + Jz @ Q @ Jz.T]]
            
            P_rr = self.P[0:3, 0:3]
            P_rx = self.P[0:3, :]
            
            Q = np.mat(np.diag([uncertainty, uncertainty]))
            
            P_ll = Jr @ P_rr @ Jr.T + Jz @ Q @ Jz.T
            P_lx = Jr @ P_rx
            
            self.P = np.vstack((self.P, P_lx))
            self.P = np.hstack((self.P, np.vstack((P_lx.T, P_ll))))
            
        else:
            # Update existing landmark
            l = self.idx[id]
            L = self.X[l:l+2, 0]
            
            theta = self.X[2,0]
            c = cos(theta)
            s = sin(theta)
            
            # Predicted measurement
            # Zpred = R(-theta) * (L - X_r)
            dx = L[0,0] - self.X[0,0]
            dy = L[1,0] - self.X[1,0]
            
            z_pred_x = c*dx + s*dy
            z_pred_y = -s*dx + c*dy
            Zpred = np.mat([[z_pred_x], [z_pred_y]])
            
            # Jacobians
            # H_R (robot)
            # dh_x/dtheta = z_pred_y
            # dh_y/dtheta = -z_pred_x
            HR = np.mat([[-c, -s, z_pred_y],
                         [ s, -c, -z_pred_x]])
            
            # H_L (landmark)
            HL = np.mat([[c, s],
                         [-s, c]])
            
            # H matrix
            H = np.mat(np.zeros((2, self.X.shape[0])))
            H[:, 0:3] = HR
            H[:, l:l+2] = HL
            
            # Kalman Gain
            Q = np.mat(np.diag([uncertainty, uncertainty]))
            S = H @ self.P @ H.T + Q
            K = self.P @ H.T @ inv(S)
            
            # Update state
            Y = Z - Zpred
            self.X = self.X + K @ Y
            
            # Normalize angle
            self.X[2,0] = self.normAngle(self.X[2,0])
            
            # Update covariance
            I = np.eye(self.P.shape[0])
            self.P = (I - K @ H) @ self.P


        self.lock.release()
        return (self.X,self.P)

    def update_compass(self, logger, Z, uncertainty):
        self.lock.acquire()
        # TODO
        logger.info("Update: S="+str(Z)+" X="+str(self.X.T))
        # Update the full state self.X and self.P based on compass measurement
        # TODO
        
        theta = self.X[2,0]
        innovation = self.normAngle(Z - theta)
        
        S = self.P[2,2] + uncertainty
        K = self.P[:, 2] / S
        
        self.X = self.X + K * innovation
        self.X[2,0] = self.normAngle(self.X[2,0]) # Normalize theta
        
        self.P = self.P - K @ self.P[2, :]

        self.lock.release()
        return (self.X,self.P)


    def publish(self, pose_pub, odom_pub, target_frame, stamp, child_frame):
        pose = super().publish(pose_pub, odom_pub, target_frame, stamp, child_frame)
        ma = MarkerArray()
        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_y"
        marker.id = 5000
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z = -0.1
        marker.scale.x = 3*sqrt(self.P[0,0])
        marker.scale.y = 3*sqrt(self.P[1,1]);
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        ma.markers.append(marker)
        for id in self.idx:
            marker = Marker()
            marker.header = pose.header
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            l = self.idx[id]
            marker.pose.position.x = self.X[l,0]
            marker.pose.position.y = self.X[l+1,0]
            marker.pose.position.z = -0.1
            marker.pose.orientation.x = 0.
            marker.pose.orientation.y = 0.
            marker.pose.orientation.z = 1.
            marker.pose.orientation.w = 0.
            marker.scale.x = 3*sqrt(self.P[l,l])
            marker.scale.y = 3*sqrt(self.P[l+1,l+1]);
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime = rclpy.time.Duration(seconds=3.).to_msg()
            ma.markers.append(marker)
            marker = Marker()
            marker.header = pose.header
            marker.ns = "landmark_kf"
            marker.id = 1000+id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = self.X[l+0,0]
            marker.pose.position.y = self.X[l+1,0]
            marker.pose.position.z = 1.0
            marker.pose.orientation.x = 0.
            marker.pose.orientation.y = 0.
            marker.pose.orientation.z = 1.
            marker.pose.orientation.w = 0.
            marker.text = str(id)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.2
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.lifetime = rclpy.time.Duration(seconds=3.).to_msg()
            ma.markers.append(marker)
        self.marker_pub.publish(ma)


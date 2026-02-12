#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos, atan2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import bisect
import threading
from rover_driver_base.rover_kinematics import RoverKinematics
from ar_loc_base.rover_odo import RoverOdo, DeltaOdo
import numpy as np

class DeltaPF(DeltaOdo):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node,initial_pose, initial_uncertainty)
        self.N = 500
        self.particles = [self.X + self.drawNoise(initial_uncertainty) for i in range(0,self.N)]
        self.pa_pub = node.create_publisher(PoseArray,"~/particles",1)
        # print self.particles

    def getRotationFromWorldToRobot(self):
        return self.getRotation(-self.X[2,0])

    def drawNoise(self, norm):
        if type(norm)==list:
            return mat(vstack(norm)*(2*random.rand(len(norm),1)-vstack([1]*len(norm))))
        else:
            return mat(multiply(norm,((2*random.rand(3,1)-vstack([1,1,1])))))

    def applyDisplacement(self,X,DeltaX,Uncertainty):
        # TODO: apply the displacement DeltaX, in the robot frame, to the particle X expressed in the world frame,
        # including the uncertainty present in variable uncertainty
        return X 


    def predict_delta(self, logger, DeltaX, Uncertainty, lock=True):
        if lock:
            self.lock.acquire()
        noise=np.diag(Uncertainty).reshape((3,1))
        # Apply the particle filter prediction step here
        # TODO

        # DeltaX = iW*S
        # Note, using the function applyDisplacement could be useful to compute the new particles
        # self.particles = ...
        self.updateMean(logger)
        if lock:
            self.lock.release()

    def evalParticleAR(self,X, Z, L, Uncertainty):
        # Returns the fitness of a particle with state X given observation Z of landmark L
        return 0

    def evalParticleCompass(self,X, Value, Uncertainty):
        # Returns the fitness of a particle with state X given compass observation value
        # Beware of the module when computing the difference of angles
        return 0

    def update_ar(self, logger, Z, L, Uncertainty):
        self.lock.acquire()
        # TODO
        logger.info("Update: L="+str(L.T)+" X="+str(self.X.T))
        # Implement particle filter update using landmarks here. Using the function evalParticleAR could be useful

        # TODO

        # self.particles = ...
        
        self.updateMean(logger)
        self.lock.release()

    def update_compass(self, logger, angle, Uncertainty):
        self.lock.acquire()
        # TODO
        # print self.particles
        logger.info("Update: S="+str(angle)+" X="+str(self.X.T))
        # Implement particle filter update using landmarks here. Using the function evalParticleCompass could be useful

        # TODO

        # self.particles = ...
        
        self.updateMean(logger)
        self.lock.release()

    def updateMean(self,logger):
        X = mat(zeros((4,1)))
        for x in self.particles:
            y=np.mat([[x[0,0],x[1,0],np.cos(x[2,0]),np.sin(x[2,0])]]).T
            X += y
        X = X / len(self.particles)

        self.X = np.mat([[X[0,0],X[1,0],atan2(X[3,0],X[2,0])]]).T

        # logger.info("Mean theta: %f (%f,%f)" % (self.X[2,0],np.cos(self.X[2,0]),np.sin(self.X[2,0])))
        
        return self.X

    def publish(self, pose_pub, odom_pub, target_frame, stamp, child_frame):
        pose = super().publish(pose_pub, odom_pub, target_frame, stamp, child_frame)

        pa = PoseArray()
        pa.header = pose.header
        for p in self.particles:
            po = Pose()
            po.position.x = p[0,0]
            po.position.y = p[1,0]
            q = self.quaternion_from_euler(0, 0, p[2,0])
            po.orientation.x = q[0]
            po.orientation.y = q[1]
            po.orientation.z = q[2]
            po.orientation.w = q[3]
            pa.poses.append(po)
        self.pa_pub.publish(pa)


class RoverPF(DeltaPF):
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
            return 
        # print "-"*32
        # then compute odometry using least square
        iW = self.kinematics.prepare_inversion_matrix(drive_cfg)
        S = self.kinematics.prepare_displacement_matrix(self.kinematics.motor_state,motor_state,drive_cfg)
        self.kinematics.motor_state.copy(motor_state)
        DeltaX = iW @ S
        Uncertainty = iW @ mat(vstack([encoder_precision] * len(S)))

        self.predict_delta(logger,DeltaX,Uncertainty,False)
        self.lock.release()



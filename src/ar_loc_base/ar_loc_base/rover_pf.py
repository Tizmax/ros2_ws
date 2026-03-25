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
from copy import deepcopy

class DeltaPF(DeltaOdo):
    def __init__(self, node, initial_pose, initial_uncertainty):
        super().__init__(node,initial_pose, initial_uncertainty)
        self.N = 500
        self.particles = [self.X + self.drawNoise(initial_uncertainty) for i in range(0,self.N)]
        self.pa_pub = node.create_publisher(PoseArray,"~/particles",1)
        # print(self.particles)

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
        theta = X[2,0]
        Rtheta = mat([[cos(theta), -sin(theta), 0], 
                      [sin(theta),  cos(theta), 0],
                      [         0,           0, 1]])

        X = X + Rtheta @ DeltaX + self.drawNoise(Uncertainty)
        X[2,0] = self.normAngle(X[2,0])
        return X


    def predict_delta(self, logger, DeltaX, Uncertainty, lock=True):
        if lock:
            self.lock.acquire()
        if type(Uncertainty) == numpy.ndarray:
            if len(Uncertainty.shape)>1:
                noise=np.diag(Uncertainty).reshape((3,1))
            else:
                noise=Uncertainty.reshape((3,1))
        else:
            noise=np.array([[Uncertainty,Uncertainty,Uncertainty]]).T

        # print(self.particles)
        # print("="*1500)
        # Apply the particle filter prediction step here
        # TODO

        # print("-"*500)
        # print("noise: %s" % (str(noise.T)))
        # print("uncertainty: %s" % (str(Uncertainty)))

        new_particles = []
        for p in self.particles:
            new_p = self.applyDisplacement(deepcopy(p),DeltaX,noise)
            new_particles.append(deepcopy(new_p))

        # print(self.particles)

        # DeltaX = iW*S
        # Note, using the function applyDisplacement could be useful to compute the new particles
        self.particles = new_particles
        self.updateMean(logger)
        if lock:
            self.lock.release()

    def evalParticleAR(self,X, Z, L, Uncertainty):
        # Returns the fitness of a particle with state X given observation Z of landmark L
        dx = L[0,0] - X[0,0]
        dy = L[1,0] - X[1,0]

        R = self.getRotation(-X[2,0])
        Zp = R @ mat([[dx],[dy]])

        #error between predicted and actual Z
        error = np.linalg.norm(Z - Zp)

        weight = exp(-0.5*error**2/Uncertainty)
        return weight

    def evalParticleCompass(self,X, Value, Uncertainty):
        # Returns the fitness of a particle with state X given compass observation value
        # Beware of the module when computing the difference of angles
        
        error = self.normAngle(Value - X[2,0])
        weight = exp(-0.5*error**2/Uncertainty)
        return weight

    def update_ar(self, logger, Z, L, Uncertainty):
        self.lock.acquire()
        # TODO
        logger.info("Update: L="+str(L.T)+" X="+str(self.X.T))
        # Implement particle filter update using landmarks here. Using the function evalParticleAR could be useful

        # TODO
        weights = []

        for p in self.particles:
            w = self.evalParticleAR(p,Z,L,Uncertainty)
            weights.append(w)

        if sum(weights)>0:
            weights = weights/sum(weights)
        else:
            weights = [1.0 / self.N for i in range(self.N)]

        # resample particles according to weights
        new_particles = []

        for i in range(0,self.N):
            idx = np.random.choice(range(0,self.N), p = weights)
            new_particles.append(deepcopy(self.particles[idx]))

        self.particles = new_particles
        
        self.updateMean(logger)
        self.lock.release()

    def update_compass(self, logger, angle, Uncertainty):
        self.lock.acquire()
        # TODO
        # print self.particles
        logger.info("Update: S="+str(angle)+" X="+str(self.X.T))
        # Implement particle filter update using landmarks here. Using the function evalParticleCompass could be useful

        # TODO
        weights = []

        for p in self.particles:
            w = self.evalParticleCompass(p,angle,Uncertainty)
            weights.append(w)

        if sum(weights)>0:
            weights = weights/sum(weights)
        else:
            weights = [1.0 / self.N for i in range(0,self.N)]

        # resample particles according to weights
        new_particles = []

        for i in range(0,self.N):
            idx = np.random.choice(range(0,self.N), p = weights)
            new_particles.append(deepcopy(self.particles[idx]))
        self.particles = new_particles
        
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



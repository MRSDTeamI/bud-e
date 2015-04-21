#!/usr/bin/python

# Given list of thetas, return poses of each joint and final pose of end effector.

import numpy as np
from scipy.linalg import expm
import sys

# Use debug values
debug = 0

class ForwardKin:
    '''
    Class that figures out forward kinematics for Crustcrawler AX-12 4 DoF arm.

    '''
    def __init__(self, j_vals=None):
        self.NUM_JOINTS = 4

        if type(j_vals) is list:
            #print len(j_vals)
            if len(j_vals) == self.NUM_JOINTS:
                self.joint_vals = j_vals
            else:
                print "ERROR: Need to specify %d joint values" % self.NUM_JOINTS
                sys.exit(-1)
        else:
            self.joint_vals = [0.5, 0.6, 0.7, 0.8]

        # Set initial values
        self.set_init()
    
    def hat_operator(self, in_data):
        out_data = np.array(
            [[0, -in_data[2], in_data[1]],
            [in_data[2], 0, -in_data[0]],
            [-in_data[1], in_data[0], 0]] );
        return out_data

    def set_init(self):
        '''
        Set initital tip position, screw/twist parameters.

        '''
        # Initial tip position
        if debug:
            self.tip_position = np.array([[0.61, 0.72, 2.376]]).transpose()
        else:
            self.tip_position = np.array([[0.3, 0, 0.1]]).transpose()
        #print self.tip_position
        
        # Set Twist/Screw parameters
        # initialize twist
        # Test values
        if debug:
            self.j1 = np.array([[0, 0, 1]]).transpose()
            self.j2 = np.array([[-1, 0, 0]]).transpose()
            self.j3 = np.array([[0, 0, 1]]).transpose()
            self.j4 = np.array([[-1, 0, 0]]).transpose()
        else:
            # Crustcrawler values
            self.j1 = np.array([[0, 0, 1]]).transpose()
            self.j2 = np.array([[0, 1, 0]]).transpose()
            self.j3 = np.array([[0, 1, 0]]).transpose()
            self.j4 = np.array([[1, 0, 0]]).transpose()
        
        # Test values
        if debug:
            self.q1 = np.array([[0.61, 0.72, 0]]).transpose()
            self.q2 = np.array([[0, 0.72, 1.346]]).transpose()
            self.q3 = np.array([[0.61, 0.72, 0]]).transpose()
            self.q4 = np.array([[0, 0.765, 1.896]]).transpose()
        else:
            # Crustcrawler values
            self.q1 = np.array([[0, 0, 0.03]]).transpose()
            self.q2 = np.array([[0.07, 0, 0.115]]).transpose()
            self.q3 = np.array([[0.24, 0, 0.115]]).transpose()
            self.q4 = np.array([[0.35, 0, 0.13]]).transpose()

        self.t1 = np.cross(-self.j1, self.q1, axis=0)
        self.t2 = np.cross(-self.j2, self.q2, axis=0)
        self.t3 = np.cross(-self.j3, self.q3, axis=0)
        self.t4 = np.cross(-self.j4, self.q4, axis=0)
        
        self.c1 = np.concatenate((self.t1, self.j1))
        self.c2 = np.concatenate((self.t2, self.j2))
        self.c3 = np.concatenate((self.t3, self.j3))
        self.c4 = np.concatenate((self.t4, self.j4))
        
        self.j = np.concatenate((self.j1, self.j2, self.j3, self.j4), axis=1)
        self.c = np.concatenate((self.c1, self.c2, self.c3, self.c4), axis=1)
        #self.j = np.concatenate((self.j1, self.j2, self.j3), axis=1)
        #self.c = np.concatenate((self.c1, self.c2, self.c3), axis=1)
        
    def set_joints(self, j_vals):
        '''
        Set the joint angles for the object.

        '''
        if isinstance(j_vals, (list, np.ndarray)):
            if len(j_vals) == self.NUM_JOINTS:
                self.joint_vals = j_vals
            else:
                print "ERROR: Need to specify %d joint values" % self.NUM_JOINTS
                sys.exit(-1)
        else:
            print "ERROR: Input needs to be a list or numpy array"
            print j_vals
            sys.exit(-1)

    def solve_fk(self):
        '''
        Solve for forward kinematics given the joint angles we have.

        RETURN:
            tip_pose - 4x4 matrix of the tip orientation.
                       The 4x4 matrix has a 3x3 rotation matrix and 3x1
                       translation vector.
        '''
        T = np.identity(4)
        eye3 = np.identity(3)
        filler = np.array([[0, 0, 0, 1]])
        gstip = np.concatenate((eye3, self.tip_position), axis=1)
        gstip = np.concatenate((gstip, filler), axis=0)
        
        # Turn into matrix so we can index easier in the for-loop
        self.j_mat = np.asmatrix(self.j)
        self.c_mat = np.asmatrix(self.c)
        
        # Loop through each of the joints
        for i in range(self.NUM_JOINTS):
            #print i

            # Calculate matrix exponential
            # this is a 3x3 matrix
            mat_exp = expm(np.multiply(self.hat_operator(self.j_mat[:,i]), self.joint_vals[i]))
            # calculate the 3x1 vector to append to the right of mat_exp
            jc_cross = np.cross(self.j_mat[:,i], self.c_mat[0:3,i], axis=0)
            term1 = np.dot((eye3-mat_exp), jc_cross)
            tmp = np.multiply(self.j_mat[:,i], self.j_mat[:,i].T)
            term2 = np.multiply(np.dot(tmp, self.c_mat[0:3,i]), self.joint_vals[i])
            append_vec = np.add(term1, term2)   # this is a 3x1
        
            # Calculated matrix
            # mat_exp with append_vec makes a 4 x 3, then pad with row of [0 0 0 1]
            calc_mat = np.concatenate((mat_exp, append_vec), axis=1)
            calc_mat = np.concatenate((calc_mat, filler), axis=0)
            #print calc_mat
        
            T = np.dot(T, calc_mat)
            #print T
        
        tip_pose = np.dot(T, gstip)
        #print tip_pose

        return tip_pose

if __name__ == "__main__":
    # INPUT
    my_fk = ForwardKin([0.5, 0.6, 0.7, 0.8])
    my_fk.solve_fk()

    my_fk.set_joints([0.4, 0.3, 0.4, 0.5])
    my_fk.solve_fk()

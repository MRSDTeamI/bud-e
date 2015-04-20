#!/usr/bin/python

# Given final position and orientation, find the joint angles

import numpy as np
from scipy.linalg import expm
from numpy.linalg import inv
import sys

import ForwardKin as fk

# Use debug values
debug = 1
# Use DLS method instead of screw theory.
# This may prevent singular matrix error.
use_dls = 1

###
# Tweak values at end of __init__
###

class InverseKin:
    '''
    Class that figures out inverse kinematics for Crustcrawler AX-12 4 DoF arm.

    '''
    def __init__(self, f_pose=None):
        self.NUM_POSE = 7  # 3 position 4 orientation

        if type(f_pose) is list:
            #print len(f_pose)
            if len(f_pose) == self.NUM_POSE:
                self.final_pose = f_pose
            else:
                print ("ERROR: Need to specify %d pose (3 position, 4 orientation)" % 
                    self.NUM_POSE)
                sys.exit(-1)

        # Set initial values
        self.theta = [0.0, 0.0, 0.0]  # initial joint angles

        ## These values can be tweaked to for IK performance
        self.num_iter = 1000  # 1000 loops to find the inverse kinematics
        self.time_step = 0.01
        self.dls_lambda = 0.01

        # Desired final position, orientation
        # NOTE: this needs to be updated if desired pose is updated in a function later
        pose_mat = np.matrix(f_pose)
        self.des_pos = pose_mat[0, 0:3]  # desired position
        self.des_q_r = pose_mat[0, 6]    # desired real part of quaternion
        self.des_q_i = pose_mat[0, 3:6]  # desired imaginary parts of quaternion
        # FK solver
        self.fk = fk.ForwardKin()

    def adjoint(self, g_mat):
        p_hat = self.fk.hat_operator(g_mat[0:3, 3])
        term = np.dot(p_hat, g_mat[0:3, 0:3])
        term2 = np.concatenate((np.zeros((3, 3)), g_mat[0:3, 0:3]), axis=1)
        output_mat = np.concatenate((g_mat[0:3, 0:3], term), axis=1)
        output_mat = np.concatenate((output_mat, term2), axis=0)

        return output_mat

    def jacobian_mat_gen(self, joint_theta):

        eye3 = np.identity(3)
        eye4 = np.identity(4)
        T_mat = eye4
        filler = np.array([[0, 0, 0, 1]])

        j_mat = self.fk.j_mat

        # Make copy so we don't change the original c
        arm_jacobian = np.copy(self.fk.c)
        for i in range(1, self.fk.NUM_JOINTS):
            for j in range(0, i+1):
                mat_exp = expm(np.multiply(self.fk.hat_operator(j_mat[:, j]), joint_theta[j]))
                # calculate the 3x1 vector to append to the right of mat_exp
                jc_cross = np.cross(self.fk.j_mat[:, j], self.fk.c_mat[0:3, j], axis=0)

                term1 = np.dot((eye3 - mat_exp), jc_cross)
                tmp = np.multiply(self.fk.j_mat[:, j], self.fk.j_mat[:, j].T)
                term2 = np.multiply(np.dot(tmp, self.fk.c_mat[0:3, j]), joint_theta[j])
                append_vec = np.add(term1, term2)  # this is a 3x1

                # Calculated matrix
                # mat_exp with append_vec makes a 4 x 3, then pad with row of [0 0 0 1]
                calc_mat = np.concatenate((mat_exp, append_vec), axis=1)
                calc_mat = np.concatenate((calc_mat, filler), axis=0)

                T_mat = np.dot(T_mat, calc_mat)
            #tmp_c = self.adjoint(T_mat)* self.fk.c_mat[:,i]
            tmp_c = np.dot(self.adjoint(T_mat), self.fk.c_mat[:,i])
            tmp_c = tmp_c.getA1()    # flatten it out or else it won't work
            arm_jacobian[:,i] = tmp_c
            T_mat = eye4

        return arm_jacobian

    def rotationM2Q(self, rot_mat):
        '''
        Calculate the quaternion for the rotation matrix that is passed in.

        RETURN:
            quaternion ([w x y z]) of the rotation matrix

        '''
        tr = np.trace(rot_mat)
        m00 = rot_mat[0,0]
        m01 = rot_mat[0,1]
        m02 = rot_mat[0,2]

        m10 = rot_mat[1,0]
        m11 = rot_mat[1,1]
        m12 = rot_mat[1,2]

        m20 = rot_mat[2,0]
        m21 = rot_mat[2,1]
        m22 = rot_mat[2,2]

        if tr > 0:
            S = np.sqrt(tr+1.0) * 2   # S=4*qw
            qw = 0.25 * S
            qx = (m21 - m12) / S
            qy = (m02 - m20) / S
            qz = (m10 - m01) / S
        elif (m00 > m11) and (m00 > m22):
            S = np.sqrt(1.0 + m00 - m11 - m22) * 2   # S=4*qx
            qw = (m21 - m12) / S
            qx = 0.25 * S
            qy = (m01 + m10) / S
            qz = (m02 + m20) / S
        elif (m11 > m22):
            S = np.sqrt(1.0 + m11 - m00 - m22) * 2   # S=4*qy
            qw = (m02 - m20) / S
            qx = (m01 + m10) / S
            qy = 0.25 * S
            qz = (m12 + m21) / S
        else:
            S = np.sqrt(1.0 + m22 - m00 - m11) * 2   # S=4*qz
            qw = (m10 - m01) / S
            qx = (m02 + m20) / S
            qy = (m12 + m21) / S
            qz = 0.25 * S

        quaternion = np.array([qx, qy, qz, qw]).transpose()
        return quaternion

    def quat_mult(self, q1, q2):
        '''
        Multiply 2 quaternions

        RETURN:
            quaternion [w x y z]
    
        '''
        w1 = q1[0]; x1 = q1[1]; y1 = q1[2]; z1 = q1[3]
        w2 = q2[0]; x2 = q2[1]; y2 = q2[2]; z2 = q2[3]

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2;
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2;

        quaternion = np.array([w, x, y, z])
        return quaternion
    
    def solve_ik(self):

        for i in range(self.num_iter):
            self.fk.set_joints(self.theta)
            tip_pose = self.fk.solve_fk()  # tip_pose is 4x4 with rotation and translation
       
            tmp_pos = tip_pose[0:3,3]    # Get the translation part of the matrix (3x1)
            tmp_rot = tip_pose[0:3,0:3]  # Get the rotation part of the matrix (3x3)
            tmp_quat = self.rotationM2Q(tmp_rot)  # Change rotation matrix to quaternion

            tmp_quat = np.asmatrix(tmp_quat)
            tmp_q_r = tmp_quat[0,3]    # real part is the last one
            tmp_q_i = tmp_quat[0:2]

            inv_tmp_q_i = -tmp_q_i

            # getA1() gets a flattened array from matrix
            # Create [des_q_r des_q_i] array
            q1 = np.concatenate(([self.des_q_r], self.des_q_i.getA1()))
            #q1 = np.insert(self.des_q_i, 0, self.des_q_r) 
            # Create [tmp_q_r inv_tmp_q_i] array
            q2 = np.concatenate(([tmp_q_r], inv_tmp_q_i.T.getA1()))

            # Calculate Error
            err_rot = self.quat_mult(q1, q2)
            # 3x1
            err_pos = -np.cross(err_rot[1:4].T, tmp_pos, axis=0) + self.des_pos.T - tmp_pos

            # The error matrix of each moment.
            # 6x1
            e_matrix = np.concatenate((err_pos, np.array([err_rot[1:4]]).T), axis=0)

            # Calculate jacobian
            # 6x4 matrix
            tmp_jacobian = self.jacobian_mat_gen(self.theta)

            # Calculate the pose of next frame
            # 6x6 matrix
            sq_mat = np.dot(tmp_jacobian, tmp_jacobian.transpose())
            if use_dls:
                print sq_mat
                sq_mat = np.add(sq_mat, np.dot(self.dls_lambda**2, np.eye(6)))

            inv_jac = inv(sq_mat)
            #print np.allclose(np.dot(sq_mat, inv_jac), np.eye(6))
            #print "inv"
            #print inv_jac
            #print "dot"
            #print sq_mat

            # Calc d_theta
            # 4x6 matrix
            tmp1 = np.dot(tmp_jacobian.transpose(), inv_jac)
            d_theta = np.dot(tmp1, e_matrix)
            #print "d_theta"
            #print e_matrix
            #print d_theta

            # Calc theta
            tmp1 = np.multiply(d_theta, self.time_step)
            #print "theta"
            #print self.theta
            #print tmp1
            self.theta = self.theta + tmp1.getA1()

        print self.theta

if __name__ == "__main__":
    ik = InverseKin([0.1420, 0.0, 0.2696, 0.0, 0.0, 0.0, 1.0])
    ik.solve_ik()

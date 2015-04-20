#!/usr/bin/python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Johnny Wang
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Johnny Wang

# Given final position and orientation, find the joint angles

import numpy as np
from scipy.linalg import expm
from numpy.linalg import inv
import sys
import rospy
import geometry_msgs.msg

import ForwardKin as fk
import jw_joint_ctrl as joint_ctrl


# Use debug values instead of listening to ROS topic
debug = 0
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

        # Need these values to solve for IK. Decalre them here so we can error check
        # at the start of solve_ik.
        self.des_pos = None
        self.des_q_r = None
        self.des_q_i = None

        # Only listen to topic for bottle coordinate if we're not debugging
        if not debug:
            rospy.Subscriber("bottle_center", geometry_msgs.msg.Vector3, self.callback)

        if type(f_pose) is list:
            #print len(f_pose)
            if len(f_pose) == self.NUM_POSE:
                self.final_pose = f_pose
            else:
                print ("ERROR: Need to specify %d pose (3 position, 4 orientation)" % 
                    self.NUM_POSE)
                sys.exit(-1)

            # Desired final position, orientation
            # NOTE: this needs to be updated if desired pose is updated in a function later
            pose_mat = np.matrix(f_pose)
            self.des_pos = pose_mat[0, 0:3]  # desired position
            self.des_q_r = pose_mat[0, 6]    # desired real part of quaternion
            self.des_q_i = pose_mat[0, 3:6]  # desired imaginary parts of quaternion

        # Set initial values
        self.theta = [0.0, 0.0, 0.0]  # initial joint angles

        ## These values can be tweaked to for IK performance
        self.num_iter = 1000  # 1000 loops to find the inverse kinematics
        self.time_step = 0.01
        self.dls_lambda = 0.01

        # FK solver
        self.fk = fk.ForwardKin()

        # Only spin and wait if we're not debugging
        if not debug:
            # spin
            while not rospy.is_shutdown():
                rospy.sleep(1)

    def adjoint(self, g_mat):
        p_hat = self.fk.hat_operator(g_mat[0:3, 3])
        term = np.dot(p_hat, g_mat[0:3, 0:3])
        term2 = np.concatenate((np.zeros((3, 3)), g_mat[0:3, 0:3]), axis=1)
        output_mat = np.concatenate((g_mat[0:3, 0:3], term), axis=1)
        output_mat = np.concatenate((output_mat, term2), axis=0)

        return output_mat

    def callback(self, end_coord):
        # Set the desired final position, orientation
        # Since we are taking in only position, we set orientation to [0 0 0 1]
        f_pose = [end_coord.x, end_coord.y, end_coord.z, 0, 0, 0, 1]
        pose_mat = np.matrix(f_pose)
        self.des_pos = pose_mat[0, 0:3]  # desired position
        self.des_q_r = pose_mat[0, 6]    # desired real part of quaternion
        self.des_q_i = pose_mat[0, 3:6]  # desired imaginary parts of quaternion

        # Get list of joint angles
        joint_angles = self.solve_ik()

        self.set_joint_angles(joint_angles)

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

        if self.des_pos == None or self.des_q_r == None or self.des_q_i == None:
            print "ERROR: Need to set desired position before solving IK."
            sys.exit(-1)

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
                #print sq_mat
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
        return self.theta

    def set_joint_angles(self, angles):
        '''
        Set joint angles of the arm through dynamixel helper class.
        
    
        '''
        if isinstance(angles, np.ndarray):
            angles = angles.tolist()

        # If using only 3 DoF solver of IK, we will only get 3 joint angles.
        # The jw_joint_ctrl class expects 4 joint angles for:
        #   shoulder_pan_joint, shoulder_pitch_joint, elbow_flex_joint, wrist_roll_joint
        # We can generally leave wrist to be 0
        if len(angles) == 3:
            angles.extend([0.0])

        # Looks like the IK values are opposite of actual joint angles
        angles = [-1*a for a in angles]
        print angles

        jctrl = joint_ctrl.Joint()
        jctrl.move_joint(angles)

if __name__ == "__main__":

    rospy.init_node('ik_solver', anonymous=True)

    try:
        if debug:
            ik = InverseKin([0.1420, 0.0, 0.2696, 0.0, 0.0, 0.0, 1.0])
            ik.solve_ik()
        else:
            ik = InverseKin()

    except rospy.ROSInterruptException:
         sys.exit(-1)

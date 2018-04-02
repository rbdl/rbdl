#!/usr/bin/python
# 
# RBDL - Rigid Body Dynamics Library
# Copyright (c) 2011-2015 Martin Felis <martin@fysx.org>
# 
# Licensed under the zlib license. See LICENSE for more details.

import unittest

import math
import numpy as np
from numpy.testing import *
import rbdl

class JointTests (unittest.TestCase):
    def test_JointConstructorAxesSimple(self):
        axis = np.asarray([[1., 0., 0., 0., 0., 0.]])
        joint_rot_x = rbdl.Joint.fromJointAxes (axis)

        assert_equal (joint_rot_x.getJointAxis(0), axis[0])

    def test_JointConstructorAxes6DoF(self):
        axis = np.asarray([
            [1., 0., 0., 0., 0., 0.],
            [0., 1., 0., 0., 0., 0.],
            [0., 0., 1., 0., 0., 0.],
            [0., 0., 0., 1., 0., 0.],
            [0., 0., 0., 0., 1., 0.],
            [0., 0., 0., 0., 0., 1.],
            ])
        joint = rbdl.Joint.fromJointAxes (axis)

        for i in range (axis.shape[0]):
            assert_equal (joint.getJointAxis(i), axis[i])

class SampleModel3R (unittest.TestCase):
    """ Example planar triple pendulum 
   
    - All joints along the positive Z axis in rest position
    - All joints revolute y joints
    - all "links" are 1 unit length
    """
    def setUp(self):
        self.model = rbdl.Model()
        joint_rot_y = rbdl.Joint.fromJointType ("JointTypeRevoluteY")
        self.body = rbdl.Body.fromMassComInertia (1., np.array([0., 0.0, 0.5]), np.eye(3) *
                0.05)
        self.xtrans = rbdl.SpatialTransform()
        self.xtrans.r = np.array([0., 0., 1.])
        
        self.body_1 = self.model.AppendBody (rbdl.SpatialTransform(), joint_rot_y, self.body)
        self.body_2 = self.model.AppendBody (self.xtrans, joint_rot_y, self.body)
        self.body_3 = self.model.AppendBody (self.xtrans, joint_rot_y, self.body)

        self.q = np.zeros (self.model.q_size)
        self.qdot = np.zeros (self.model.qdot_size)
        self.qddot = np.zeros (self.model.qdot_size)
        self.tau = np.zeros (self.model.qdot_size)

    def test_CoordinateTransformBodyBase (self):
        """
        Checks whether CalcBodyToBaseCoordinates and CalcBaseToBodyCoordinates
        give the right results.
        """
        q = np.random.rand(self.model.q_size)
        point_local = np.array ([1., 2., 3.])
        point_base = rbdl.CalcBodyToBaseCoordinates (
                self.model,
                q,
                self.body_3,
                point_local)
        point_local_2 = rbdl.CalcBaseToBodyCoordinates (
                self.model,
                q,
                self.body_3,
                point_base)
      
        assert_almost_equal (point_local, point_local_2)

    def test_CalcPointVelocity (self):
        """
        Checks whether CalcBodyToBaseCoordinates and CalcBaseToBodyCoordinates
        give the right results.
        """
        q = np.zeros(self.model.q_size)
        qdot = np.zeros(self.model.q_size)
        qdot[0] = 1.
        point_local = np.array ([0., 0., 0.])
        point_vel = rbdl.CalcPointVelocity (
                self.model,
                q,
                qdot,
                self.body_3,
                point_local
                )

        assert_almost_equal (np.array([2., 0., 0.]), point_vel)

    def test_CalcCenterOfMass (self):
        """ Tests calculation of center of mass
        TODO: add checks for angular momentum
        """

        com = np.array ([-1., -1., -1.])
        com_vel = np.array([-2., -2., -2.])
        ang_mom = np.array([-3., -3., -3.])
        self.qdot[0] = 1.

        mass = rbdl.CalcCenterOfMass (
                self.model,
                self.q,
                self.qdot,
                None,
                com,
                None,
                None 
                )
        self.assertEqual (3, mass)
        assert_almost_equal (np.array([0., 0., 1.5]), com)
        assert_almost_equal (np.array([0., 0., 1.5]), com)

        mass = rbdl.CalcCenterOfMass (
                self.model,
                self.q,
                self.qdot,
                None,
                com,
                com_vel,
                None 
                )
        self.assertEqual (3, mass)
        assert_almost_equal (np.array([0., 0., 1.5]), com)
        assert_almost_equal (np.array([1.5, 0., 0.0]), com_vel)

        mass = rbdl.CalcCenterOfMass (
                self.model,
                self.q,
                self.qdot,
                None,
                com,
                com_vel,
                ang_mom
                )
        self.assertEqual (3, mass)
        assert_almost_equal (np.array([0., 0., 1.5]), com)

    def test_DynamicsConsistency (self):
        """ Checks whether forward and inverse dynamics are consistent """
        q = np.random.rand (self.model.q_size)
        qdot = np.random.rand (self.model.q_size)
        qddot = np.random.rand (self.model.q_size)

        tau = np.random.rand (self.model.q_size)

        rbdl.ForwardDynamics (
                self.model,
                q,
                qdot,
                tau,
                qddot)

        tau_id = np.zeros ((self.model.q_size))
        rbdl.InverseDynamics (
                self.model,
                q,
                qdot, 
                qddot,
                tau_id
                )

        assert_almost_equal (tau, tau_id)

    def test_NonlinearEffectsConsistency (self):
        """ Checks whether NonlinearEffects is consistent with InverseDynamics """
        q = np.random.rand (self.model.q_size)
        qdot = np.random.rand (self.model.q_size)

        nle_id = np.random.rand (self.model.q_size)

        rbdl.InverseDynamics(
                self.model,
                q,
                qdot,
                np.zeros (self.model.qdot_size),
                nle_id)

        nle = np.zeros ((self.model.q_size))
        rbdl.NonlinearEffects (
                self.model,
                q,
                qdot, 
                nle
                )

        assert_almost_equal (nle_id, nle)

    def test_CalcPointJacobian (self):
        """ Computes point Jacobian and checks whether G * qdot is consistent
        with CalcPointVelocity. """
        q = np.zeros (self.model.q_size)
        G = np.zeros ([3, self.model.q_size])
        point_coords = np.array ([0., 0., 1.])

        rbdl.CalcPointJacobian (
                self.model,
                q,
                self.body_3,
                point_coords,
                G
                )

        qdot = np.ones(self.model.qdot_size)
        point_vel = rbdl.CalcPointVelocity (
                self.model,
                q,
                qdot,
                self.body_3,
                point_coords
                )

        jac_point_vel = np.dot (G, qdot)
        assert_almost_equal (jac_point_vel, point_vel)

    def test_CalcPointJacobianNonSquare (self):
        """ Computes point Jacobian and checks whether G * qdot is consistent
        with CalcPointVelocity. """

        self.model = rbdl.Model()
        joint_trans_xyz = rbdl.Joint.fromJointType ("JointTypeTranslationXYZ")

        self.body_1 = self.model.AppendBody (rbdl.SpatialTransform(),
                joint_trans_xyz, self.body)

        self.body_4 = self.model.AppendBody (rbdl.SpatialTransform(),
                joint_trans_xyz, self.body)

        point_coords = np.array ([0., 0., 1.])
        q = np.zeros (self.model.q_size)
        G = np.zeros ([3, self.model.q_size])

        rbdl.CalcPointJacobian (
                self.model,
                q,
                self.body_4,
                point_coords,
                G
                )

        qdot = np.ones(self.model.qdot_size)
        jac_point_vel = np.dot (G, qdot)

        point_vel = rbdl.CalcPointVelocity (
                self.model,
                q,
                qdot,
                self.body_4,
                point_coords
                )
       
        assert_almost_equal (jac_point_vel, point_vel)

class FloatingBaseModel (unittest.TestCase):
    """ Model with a floating base
    """
    def setUp(self):
        self.model = rbdl.Model()
        joint_rot_y = rbdl.Joint.fromJointType ("JointTypeFloatingBase")
        self.body = rbdl.Body.fromMassComInertia (1., np.array([0., 0.0, 0.5]), np.eye(3) *
                0.05)
        self.xtrans = rbdl.SpatialTransform()
        self.xtrans.r = np.array([0., 0., 0.])
        
        self.body_1 = self.model.AppendBody (rbdl.SpatialTransform(), joint_rot_y, self.body)

        self.q = np.zeros (self.model.q_size)
        self.qdot = np.zeros (self.model.qdot_size)
        self.qddot = np.zeros (self.model.qdot_size)
        self.tau = np.zeros (self.model.qdot_size)

    def test_Dimensions (self):
        """
        Checks whether the dimensions of q and qdot are correct
        """

        q = np.random.rand(self.model.q_size)
        self.assertEqual (7, self.model.q_size)
        self.assertEqual (6, self.model.qdot_size)

    def test_SetQuaternion (self):
        mat = np.asarray ([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
        rbdl_quat = rbdl.Quaternion.fromPythonMatrix (mat)
        ref_q = self.q.copy()

        self.model.SetQuaternion (2, rbdl_quat.toNumpy(), self.q)

        ref_q[3:6] = rbdl_quat[0:3]
        ref_q[-1] = rbdl_quat[3]

        assert_array_equal (ref_q, self.q)

    def test_GetQuaternion (self):
        mat = np.asarray ([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
        rbdl_quat = rbdl.Quaternion.fromPythonMatrix (mat)

        self.assertEqual (4, len(rbdl_quat))
        self.q[5] = math.sqrt(2.) * 0.5
        self.q[6] = math.sqrt(2.) * 0.5

        ref_quat = [0., 0., math.sqrt(2.) * 0.5, math.sqrt(2.) * 0.5]
        quat = self.model.GetQuaternion (2, self.q)

        assert_array_equal (np.asarray(ref_quat), quat)

class ConstraintSetTests (unittest.TestCase):
    def test_Simple (self):
        # only tests whether the API seems to work. No functional
        # tests yet.

        cs = rbdl.ConstraintSet()
        idx = cs.AddContactConstraint (1, [1., 2., 3.], [4., 5., 6.])
        assert_equal (0, idx)

        X = rbdl.SpatialTransform()
        sv = rbdl.SpatialVector.fromPythonArray ([1., 2., 3., 4., 5., 6.])
        idx2 = cs.AddLoopConstraint (1, 2, X, X, sv, 1.)
        assert_equal (1, idx2)

if __name__ == '__main__':
    unittest.main()

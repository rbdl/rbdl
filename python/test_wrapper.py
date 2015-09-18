#!/usr/bin/python
# 
# RBDL - Rigid Body Dynamics Library
# Copyright (c) 2011-2015 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
# 
# Licensed under the zlib license. See LICENSE for more details.

import unittest

import numpy as np
from numpy.testing import *
import rbdl

class SampleModel3R (unittest.TestCase):
    """ Example planar triple pendulum 
   
    - All joints along the positive Z axis in rest position
    - All joints revolute y joints
    - all "links" are 1 unit length
    """
    def setUp(self):
        self.model = rbdl.Model()
        joint_rot_y = rbdl.Joint.fromJointType ("JointTypeRevoluteY")
        body = rbdl.Body.fromMassComInertia (1., np.array([0., 0.0, 0.5]), np.eye(3) *
                0.05)
        xtrans = rbdl.SpatialTransform()
        xtrans.r = np.array([0., 0., 1.])
        
        self.body_1 = self.model.AppendBody (rbdl.SpatialTransform(), joint_rot_y, body)
        self.body_2 = self.model.AppendBody (xtrans, joint_rot_y, body)
        self.body_3 = self.model.AppendBody (xtrans, joint_rot_y, body)

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

if __name__ == '__main__':
    unittest.main()

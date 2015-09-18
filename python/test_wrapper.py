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

if __name__ == '__main__':
    unittest.main()

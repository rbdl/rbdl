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
        joint_rot_x_type = rbdl.Joint.fromJointType (rbdl.PJointType.PJointTypeRevoluteX)
        
        assert_equal (joint_rot_x.getJointAxis(0), axis[0])
        assert_equal (joint_rot_x_type.getJointAxis(0), axis[0])
        assert_equal (joint_rot_x.mDoFCount, 1)
        assert_equal (joint_rot_x_type.mDoFCount, 1)

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
        joint2 = rbdl.Joint.fromJointType (rbdl.PJointType.PJointTypeFloatingBase)

        
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
        joint_rot_y = rbdl.Joint.fromJointType (rbdl.PJointType.PJointTypeRevoluteY)
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
                com
                )
                
        self.assertEqual (3, mass)
        assert_almost_equal (np.array([0., 0., 1.5]), com)
        assert_almost_equal (np.array([0., 0., 1.5]), com)

        mass = rbdl.CalcCenterOfMass (
                self.model,
                self.q,
                self.qdot,
                com,
                None,
                com_vel
                )
        self.assertEqual (3, mass)
        assert_almost_equal (np.array([0., 0., 1.5]), com)
        assert_almost_equal (np.array([1.5, 0., 0.0]), com_vel)

        mass = rbdl.CalcCenterOfMass (
                self.model,
                self.q,
                self.qdot,
                com,
                None,
                com_vel,
                None,
                ang_mom,
                None
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
                qddot
                )

        tau_id = np.zeros ((self.model.q_size))
        rbdl.InverseDynamics (
                self.model,
                q,
                qdot, 
                qddot,
                tau_id
                )

        assert_almost_equal (tau, tau_id)
        
        
    def test_Dynamics_fextConsistency (self):
        """ Checks whether forward and inverse dynamics are consistent """
        q = np.random.rand (self.model.q_size)
        qdot = np.random.rand (self.model.q_size)
        qddot = np.random.rand (self.model.q_size)
        qddot_fext = qddot

        tau = np.random.rand (self.model.q_size)
        
        forceA = np.zeros (6)
        forceB = np.zeros (6)
        forceC = np.zeros (6)
        forceD = np.zeros (6)
        
        fext = np.array([forceA, forceB, forceC, forceD])

        rbdl.ForwardDynamics (
                self.model,
                q,
                qdot,
                tau,
                qddot,
                )
        rbdl.ForwardDynamics (
                self.model,
                q,
                qdot,
                tau,
                qddot_fext,
                fext
                )

        tau_id = np.zeros ((self.model.q_size))
        tau_id_fext = tau_id
        
        rbdl.InverseDynamics (
                self.model,
                q,
                qdot, 
                qddot,
                tau_id,
                )
        rbdl.InverseDynamics (
                self.model,
                q,
                qdot, 
                qddot,
                tau_id_fext,
                fext
                )
        
        assert_almost_equal (qddot, qddot_fext)
        assert_almost_equal (tau_id, tau_id_fext)
        
    def test_DynamicsConsistency_with_fext (self):
        """ Checks whether forward and inverse dynamics are consistent """
        q = np.random.rand (self.model.q_size)
        qdot = np.random.rand (self.model.q_size)
        qddot = np.random.rand (self.model.q_size)

        tau = np.random.rand (self.model.q_size)
        
        forceA = np.zeros (6)
        forceB = np.random.rand (6)
        forceC = np.random.rand (6)
        forceD = np.random.rand (6)
        
        fext = np.array([forceA, forceB, forceC, forceD])

        rbdl.ForwardDynamics (
                self.model,
                q,
                qdot,
                tau,
                qddot,
                fext
                )

        tau_id = np.zeros ((self.model.q_size))
        rbdl.InverseDynamics (
                self.model,
                q,
                qdot, 
                qddot,
                tau_id,
                fext
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
        joint_trans_xyz = rbdl.Joint.fromJointType (rbdl.PJointType.PJointTypeTranslationXYZ)

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
    
    def test_InverseKinematics (self):
        """ Checks whether inverse kinematics methods are consistent """
        
        q = np.random.rand (self.model.q_size)
        q_res = np.random.rand (self.model.q_size)
        qCS_res = np.random.rand (self.model.q_size)

        target_body_ids = np.array([self.body_3]) 
        body_points = np.array([np.zeros(3)])
        body_points[0][2] = 1.0
        target_positions = np.array([np.zeros(3)])
        target_positions[0][2] = 1.0
        target_positions[0][0] = 2.0
        ori_matrix = np.array([[0., 0., -1.], [0., 1., 0.], [1., 0., 0.]])
        
        CS = rbdl.InverseKinematicsConstraintSet()
        CS_size = CS.AddPointConstraint (target_body_ids[0],
                body_points[0],
                target_positions[0]
                )        
        CS.AddOrientationConstraint (self.body_1, ori_matrix)
                
        CS.dlambda = 0.9   
        CS.max_steps = 2000  
        CS.step_tol = 1.0e-8
        
               
        rbdl.InverseKinematics ( self.model, q, target_body_ids, 
                    body_points, target_positions, q_res, 
                    CS.step_tol, CS.dlambda, CS.max_steps 
                    )
        rbdl.InverseKinematicsCS ( self.model, q, CS, qCS_res )
        
        res_point = rbdl.CalcBodyToBaseCoordinates(self.model, 
                                    q_res, self.body_3, body_points[0])
        res_point2 = rbdl.CalcBodyToBaseCoordinates(self.model, 
                                    qCS_res, self.body_3, body_points[0])
        res_ori = rbdl.CalcBodyWorldOrientation (self.model, 
                                    qCS_res, self.body_1)
        
        assert_almost_equal (target_positions[0], res_point)
        assert_almost_equal (target_positions[0], res_point2)
        assert_almost_equal (ori_matrix, res_ori)

class FloatingBaseModel (unittest.TestCase):
    """ Model with a floating base
    """
    def setUp(self):
      
        self.model = rbdl.Model()
        joint_rot_y = rbdl.Joint.fromJointType (rbdl.PJointType.PJointTypeFloatingBase)
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
    


class FloatingBaseModel2 (unittest.TestCase):
    
    """ Model with a floating base
    """
    def setUp(self):
      
        axis = np.asarray([
            [0., 0., 0., 1., 0., 0.],
            [0., 0., 0., 0., 1., 0.],
            [0., 0., 0., 0., 0., 1.],
            [0., 0., 1., 0., 0., 0.],
            [0., 1., 0., 0., 0., 0.],
            [1., 0., 0., 0., 0., 0.],
            ])
     
      
        self.model = rbdl.Model()
        self.model.gravity = np.array ([0., -9.81, 0.])
        joint_rot_y = rbdl.Joint.fromJointAxes (axis)
        self.body = rbdl.Body.fromMassComInertia (1., np.array([0., 0.0, 0.0]), np.eye(3))
        self.xtrans = rbdl.SpatialTransform()
        self.xtrans.r = np.array([0., 0., 0.])
        
        self.body_1 = self.model.AppendBody (rbdl.SpatialTransform(), joint_rot_y, self.body)

        self.q = np.zeros (self.model.q_size)
        self.qdot = np.zeros (self.model.qdot_size)
        self.qddot = np.zeros (self.model.qdot_size)
        self.tau = np.zeros (self.model.qdot_size)
    
    def test_UpdateKinematicsConsistency (self):
      
        contact_body_id = self.body_1
        contact_point = np.array( [0., -1., 0.]);
        
        self.q[0] = 0.1
        self.q[1] = 0.2
        self.q[2] = 0.3
        self.q[3] = 0.4
        self.q[4] = 0.5
        self.q[5] = 0.6
        
        rbdl.UpdateKinematics (self.model, self.q, self.qdot, self.qddot)
        point1 = rbdl.CalcBodyToBaseCoordinates(self.model, 
                                      self.q, contact_body_id, contact_point)
        rbdl.UpdateKinematicsCustom (self.model, self.q)
        point2 = rbdl.CalcBodyToBaseCoordinates(self.model, 
                                      self.q, contact_body_id, contact_point)
        
        self.qdot[0] = 1.1
        self.qdot[1] = 1.2
        self.qdot[2] = 1.3
        self.qdot[3] = -1.4
        self.qdot[4] = -1.5
        self.qdot[5] = -1.6
        
        rbdl.UpdateKinematics (self.model, self.q, self.qdot, self.qddot)
        point_velocity1 = rbdl.CalcPointVelocity (self.model, self.q, 
                                self.qdot, contact_body_id, contact_point);
        rbdl.UpdateKinematicsCustom (self.model, self.q, self.qdot)
        point_velocity2 = rbdl.CalcPointVelocity (self.model, self.q, 
                                self.qdot, contact_body_id, contact_point);
        
        
        self.qdot[0] = 10.1
        self.qdot[1] = 10.2
        self.qdot[2] = 10.3
        self.qdot[3] = -10.4
        self.qdot[4] = -10.5
        self.qdot[5] = -10.6
        
        rbdl.UpdateKinematics (self.model, self.q, self.qdot, self.qddot)
        point_acceleration1 = rbdl.CalcPointAcceleration (self.model, 
            self.q, self.qdot, self.qddot, contact_body_id, contact_point);
        rbdl.UpdateKinematicsCustom (self.model, self.q, self.qdot, self.qddot)
        point_acceleration2 = rbdl.CalcPointAcceleration (self.model, 
            self.q, self.qdot, self.qddot, contact_body_id, contact_point);
            
            
        assert_almost_equal (point1, point2)
        assert_almost_equal (point_velocity1, point_velocity2)
        assert_almost_equal (point_acceleration1, point_acceleration2)

        
    
    # ForwardDynamicsConstraintsDirect 
    def test_ForwardDynamicsConstraintsDirectSimple (self):
      
        self.q[1] = 1.
        self.qdot[0] = 1.
        self.qdot[3] = -1.

        contact_body_id = self.body_1
        contact_point = np.array( [0., -1., 0.]);

        constraint_set = rbdl.ConstraintSet()

        constraint_set.AddContactConstraint (contact_body_id, contact_point, np.array ([1., 0., 0.]), "ground_x");
        constraint_set.AddContactConstraint (contact_body_id, contact_point, np.array ([0., 1., 0.]), "ground_y");
        constraint_set.AddContactConstraint (contact_body_id, contact_point, np.array ([0., 0., 1.]), "ground_z");

        constraint_set.Bind (self.model);

        rbdl.ForwardDynamicsConstraintsDirect (self.model, self.q, self.qdot, self.tau, constraint_set, self.qddot);
        point_acceleration = rbdl.CalcPointAcceleration (self.model, self.q, self.qdot, self.qddot, contact_body_id, contact_point);

        assert_almost_equal( np.array([0., 0., 0.]), point_acceleration)
  

    def test_ForwardDynamicsConstraintsDirectMoving (self):
      
      
        self.q[0] = 0.1
        self.q[1] = 0.2
        self.q[2] = 0.3
        self.q[3] = 0.4
        self.q[4] = 0.5
        self.q[5] = 0.6
        
        self.qdot[0] = 1.1
        self.qdot[1] = 1.2
        self.qdot[2] = 1.3
        self.qdot[3] = -1.4
        self.qdot[4] = -1.5
        self.qdot[5] = -1.6

        contact_body_id = self.body_1
        contact_point = np.array( [0., -1., 0.]);

        constraint_set = rbdl.ConstraintSet()
        constraint_set.AddContactConstraint (contact_body_id, contact_point, np.array ([1., 0., 0.]), "ground_x");
        constraint_set.AddContactConstraint (contact_body_id, contact_point, np.array ([0., 1., 0.]), "ground_y");
        constraint_set.AddContactConstraint (contact_body_id, contact_point, np.array ([0., 0., 1.]), "ground_z");

        constraint_set.Bind (self.model);
        
        rbdl.ForwardDynamicsConstraintsDirect (self.model, self.q, self.qdot, self.tau, constraint_set, self.qddot);

        point_acceleration = rbdl.CalcPointAcceleration (self.model, self.q, self.qdot, self.qddot, contact_body_id, contact_point);
        
        assert_almost_equal( np.array([0., 0., 0.]), point_acceleration)
        
    
    def test_CompositeRigidBodyAlgorithm_NonlinearEffects (self):
      
      
        self.q = np.random.rand (self.model.q_size)
        self.qdot = np.random.rand (self.model.q_size)
        self.qddot = np.random.rand (self.model.q_size)
        
        tau_nonlin = np.zeros(self.model.q_size)
        H = np.zeros( (self.model.q_size, self.model.q_size) )
        
        rbdl.InverseDynamics (self.model, self.q, self.qdot, self.qddot, self.tau)
        
        rbdl.CompositeRigidBodyAlgorithm (self.model, self.q, H)        
        rbdl.NonlinearEffects (self.model, self.q, self.qdot, tau_nonlin)
        tau_comp = H.dot(self.qddot) + tau_nonlin

        
        assert_almost_equal( tau_comp, self.tau)


class ConstraintSetTests (unittest.TestCase):
    def test_Simple (self):
        # only tests whether the API seems to work. No functional
        # tests yet.

        cs = rbdl.ConstraintSet()
        idx = cs.AddContactConstraint (1, [1., 2., 3.], [4., 5., 6.])
        assert_equal (0, idx)

        X = rbdl.SpatialTransform()
        sv = rbdl.SpatialVector.fromPythonArray ([1., 2., 3., 4., 5., 6.])
        idx2 = cs.AddLoopConstraint (1, 2, X, X, sv, True, 1.)
        assert_equal (1, idx2)

if __name__ == '__main__':
    unittest.main()

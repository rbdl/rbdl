import numpy as np
import rbdl

model = rbdl.Model()

joint_rot_y = rbdl.Joint.fromJointType ("JointTypeRevoluteY")
body = rbdl.Body.fromMassComInertia (1., np.array([0., 0.5, 0.]), np.eye(3) *
    0.05)
xtrans= rbdl.SpatialTransform()
xtrans.r = np.array([0., 1., 0.])

# print (joint_rot_y)
# print (model)
# print (body)
# print (body.mInertia)
# print (xtrans)
# 
body_1 = model.AppendBody (rbdl.SpatialTransform(), joint_rot_y, body)
body_2 = model.AppendBody (xtrans, joint_rot_y, body)
body_3 = model.AppendBody (xtrans, joint_rot_y, body)

q = np.zeros (model.q_size)
qdot = np.zeros (model.qdot_size)
qddot = np.zeros (model.qdot_size)
tau = np.zeros (model.qdot_size)

q[0] = 1.3
q[1] = -0.5
q[2] = 3.2

point_local = np.array([1., 2., 3.])
point_base = rbdl.CalcBodyToBaseCoordinates (model, q, body_3, point_local)
point_local_2 = rbdl.CalcBaseToBodyCoordinates (model, q, body_3, point_base)
print (point_local - point_local_2)



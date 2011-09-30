function [qdd] = benchmark_test ()

model = CreateModel();

null_body = CreateBody (0, [0., 0., 0.], diag([0., 0., 0.]));
base_body = CreateBody (1, [0, 0.5, 0], diag([1, 1, 1]));

joint_rot_z = CreateJoint ([0, 0, 1]', 0);
joint_rot_y = CreateJoint ([0, 1, 0]', 0);
joint_rot_x = CreateJoint ([1, 0, 0]', 0);

[model, body_id_rot_z] = AddBody (model, 0, joint_rot_z, Xtrans([0, 0, 0]), null_body);
[model, body_id_rot_y] = AddBody (model, body_id_rot_z, joint_rot_y, Xtrans([0, 0, 0]), null_body);
[model, body_id_rot_x] = AddBody (model, body_id_rot_y, joint_rot_x, Xtrans([0, 0, 0]), base_body);

model

q = zeros(3,1);
qdot = zeros(3,1);
tau = zeros(3,1);

q(1) = 1.;
% [qdd] = FDcrb (model, q, qdot, tau, {}, [0, -9.81, 0]');
[qdd] = FDab (model, q, qdot, tau, {}, [0, -9.81, 0]');

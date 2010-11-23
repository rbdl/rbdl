function [afb, qdd] = TestCalcDynamicTree3DFloat (nb)

if nargin < 1
	nb = 5
end

remaining_body_count = nb;

model = CreateModel();

if remaining_body_count > 0
	remaining_body_count = remaining_body_count - 1;
	body_base = CreateBody (1, [1, 0, 0], diag([1, 1, 1]));
	joint_base = CreateJoint ([1, 0, 0]', 0);
	[model, base_id] = AddBody (model, 0, joint_base, Xtrans([0, 0, 0]),
	body_base);
end

if remaining_body_count > 0
	remaining_body_count = remaining_body_count - 1;
	body_2 = CreateBody (1, [0, 1, 0], diag([1, 1, 1]));
	joint_2 = CreateJoint ([0, 1, 0]', 0);
	[model, body_2_id] = AddBody (model, base_id, joint_2, Xtrans([1, 0, 0]),
	body_2);
end

if remaining_body_count > 0
	remaining_body_count = remaining_body_count - 1;
	body_3 = CreateBody (1, [0, 0, 1], diag([1, 1, 1]));
	joint_3 = CreateJoint ([1, 0, 0]', 0);
	[model, body_3_id] = AddBody (model, body_2_id, joint_3, Xtrans([0, 1, 0]),
	body_3);
end

if remaining_body_count > 0
	remaining_body_count = remaining_body_count - 1;
	body_4 = CreateBody (1, [0, 1, 0], diag([1, 1, 1]));
	joint_4 = CreateJoint ([0, 1, 0]', 0);
	[model, body_4_id] = AddBody (model, base_id, joint_4, Xtrans([-0.5, 0, 0]),
	body_4);
end

if remaining_body_count > 0
	remaining_body_count = remaining_body_count - 1;
	body_5 = CreateBody (1, [0, 0, 1], diag([1, 1, 1]));
	joint_5 = CreateJoint ([1, 0, 0]', 0);
	[model, body_5_id] = AddBody (model, body_4_id, joint_5, Xtrans([0, -0.5, 0]),
	body_5);
end

floatmodel = floatbase(model);

pos_base = [0, 0, 0];
rot_base = [0, 0, 0];

X_B = Xtrans (pos_base) * Xrotz(rot_base(1)) * Xroty(rot_base(2)) * Xrotx(rot_base(3));

v_B = zeros (6,1);
f_B = zeros (6,1);
a_B = zeros (6,1);
q = zeros(nb, 1);
qdot = zeros(nb, 1);
tau = zeros(nb, 1);

v_B = ones (6,1);
tau = ones (nb, 1);

% qdot(1) = 1;
% tau(1) = 1;

[afb, qdd] = FDf (floatmodel, X_B, v_B, q, qdot, tau, {}, [0, -9.81, 0]');

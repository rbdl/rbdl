function [afb, qdd] = customfloat()

nb = 1;
bf = 0;
skew = 0;
taper = 0;
afb = [];
qdd = [];

model.NB = nb;
model.pitch = zeros(1,model.NB);

% 1st body
i = 1;
model.parent(i) = 0;
model.Xtree{i} = Xtrans([0 0 0]);
model.jaxis{i} = [0 0 1]';
mass = 1.;
CoM = [0 1 0];
Icm = diag([1 1 1]);
model.I{i} = mcI(mass, CoM, Icm);

% i = 2;
% model.parent(i) = 1;
% model.Xtree{i} = Xtrans([2 0 0]);
% model.jaxis{i} = [0 0 1]';
% mass = 1.;
% CoM = [1 0 0];
% Icm = diag([1 1 1]);
% model.I{i} = mcI(mass, CoM, Icm);

floatmodel = floatbase (model);

q = zeros(nb,1);
qd = zeros(nb,1);
tau = zeros(nb,1);

X_B = eye(6);
v_B = zeros(6,1);

tau(1) = 1;
v_B = [0 0 -1 1 0 0]';
[afb, qdd] = FDf (floatmodel, X_B, v_B, q, qd, tau, {}, [0; -9.81; 0]);


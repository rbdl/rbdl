function [afb, qdd] = customtest3dtreefloat(nb)

if nargin < 1
	nb = 3
end

bf = 0;
skew = 0;
taper = 0;

model.NB = nb;
model.pitch = zeros(1,model.NB);

% 1st body
model.parent(1) = 0;
model.Xtree{1} = Xtrans([0 0 0]);
model.jaxis{1} = [0 0 1]';
mass = 1.;
CoM = [1 0 0];
Icm = diag([1 1 1]);
model.I{1} = mcI(mass, CoM, Icm);

model.parent(2) = 1;
model.Xtree{2} = Xtrans([1 0 0]);
model.jaxis{2} = [0 1 0]';
mass = 1.;
CoM = [0 1 0];
Icm = diag([1 1 1]);
model.I{2} = mcI(mass, CoM, Icm);

model.parent(3) = 2;
model.Xtree{3} = Xtrans([0 1 0]);
model.jaxis{3} = [1 0 0]';
mass = 1.;
CoM = [0 0 1];
Icm = diag([1 1 1]);
model.I{3} = mcI(mass, CoM, Icm);

model.parent(4) = 1;
model.Xtree{4} = Xtrans([-0.5 0 0]);
model.jaxis{4} = [0 1 0]';
mass = 1.;
CoM = [0 1 0];
Icm = diag([1 1 1]);
model.I{4} = mcI(mass, CoM, Icm);

model.parent(5) = 4;
model.Xtree{5} = Xtrans([0 -0.5 0]);
model.jaxis{5} = [1 0 0]';
mass = 1.;
CoM = [0 0 1];
Icm = diag([1 1 1]);
model.I{5} = mcI(mass, CoM, Icm);

floatmodel = floatbase(model)

q = zeros(nb,1);
qd = zeros(nb,1);
tau = zeros(nb,1);
tau(1) = 1
%tau = [1, 1, 1, 1, 1, 1];
%q(1) = pi * 0.5;
%qd(1) = 10;

% q = [1.23, -0.02, 6];

X_B = eye(6);
v_B = zeros(6,1);

afb = [];
qdd = [];

[afb, qdd] = FDf (floatmodel, X_B, v_B, q, qd, tau, {}, [0; -9.81; 0]);

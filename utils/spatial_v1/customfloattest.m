function [afb, qdd] = customfloattest(nb)

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
model.appearance{1} = { { 'box', [-0.2, 0.2; -0.3, 0.3; -0.2, -0.06] } };

model.parent(2) = 1;
model.Xtree{2} = Xtrans([1 0 0]);
model.jaxis{2} = [0 0 1]';
mass = 1.;
CoM = [1 0 0];
Icm = diag([1 1 1]);
model.I{2} = mcI(mass, CoM, Icm);
model.appearance{2} = { { 'box', [-0.2, 0.2; -0.3, 0.3; -0.2, -0.06] } };

model.parent(3) = 2;
model.Xtree{3} = Xtrans([1 0 0]);
model.jaxis{3} = [0 0 1]';
mass = 1.;
CoM = [1 0 0];
Icm = diag([1 1 1]);
model.I{3} = mcI(mass, CoM, Icm);
model.appearance{3} = { { 'box', [-0.2, 0.2; -0.3, 0.3; -0.2, -0.06] } };

q = zeros(nb,1);
qd = zeros(nb,1);
tau = zeros(nb,1);

%q(1) = pi * 0.5;
%qd(1) = 10;

model = floatbase (model);

tau = [1; 2; 3];

[afb, qdd] = FDf (model, eye(6), zeros(6,1), q, qd, tau, {}, [0; -9.81; 0]);

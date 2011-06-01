function [res] = custommanualtree()

nb = 6;

bf = 0;
skew = 0;
taper = 0;

model.NB = nb;
model.pitch = zeros(1,model.NB);

% 1st body
model.parent(1) = 0;
model.Xtree{1} = Xtrans([0 0 0]);
model.jaxis{1} = [1 0 0]';
mass = 0.;
CoM = [1 0 0];
Icm = diag([1 1 1]);
model.pitch(1) = inf;
model.I{1} = mcI(mass, CoM, Icm);

model.parent(2) = 1;
model.Xtree{2} = Xtrans([0 0 0]);
model.jaxis{2} = [0 1 0]';
mass = 0.;
CoM = [0 1 0];
Icm = diag([1 1 1]);
model.pitch(2) = inf;
model.I{2} = mcI(mass, CoM, Icm);

model.parent(3) = 2;
model.Xtree{3} = Xtrans([0 0 0]);
model.jaxis{3} = [0 0 1]';
mass = 0.;
CoM = [0 0 1];
Icm = diag([1 1 1]);
model.pitch(3) = inf;
model.I{3} = mcI(mass, CoM, Icm);

model.parent(4) = 3;
model.Xtree{4} = Xtrans([0 0 0]);
model.jaxis{4} = [0 0 1]';
mass = 0.;
CoM = [0 1 0];
Icm = diag([1 1 1]);
model.I{4} = mcI(mass, CoM, Icm);

model.parent(5) = 4;
model.Xtree{5} = Xtrans([0 0 0]);
model.jaxis{5} = [0 1 0]';
mass = 0.;
CoM = [1 0 0];
Icm = diag([1 1 1]);
model.I{5} = mcI(mass, CoM, Icm);

model.parent(6) = 5; 
model.Xtree{6} = Xtrans([0 0 0]);
model.jaxis{6} = [1 0 0]';
mass = 1.;
CoM = [1 0 0];
Icm = diag([1 1 1]);
model.I{6} = mcI(mass, CoM, Icm);

Q = zeros(nb,1);
QDot = zeros(nb,1);
QDDot = zeros(nb,1);
Tau = zeros(nb,1);
%tau(1) = 1
%tau = [1, 1, 1, 1, 1, 1];
%q(1) = pi * 0.5;
%qd(1) = 10;

Q(1+0) = 1.1;
Q(1+1) = 1.2;
Q(1+2) = 1.3;
Q(1+3) = 0.1;
Q(1+4) = 0.2;
Q(1+5) = 0.3;

QDot(1+0) = 1.1;
QDot(1+1) = -1.2;
QDot(1+2) = 1.3;
QDot(1+3) = -0.1;
QDot(1+4) = 0.2;
QDot(1+5) = -0.3;

Tau(1+0) = 2.1;
Tau(1+1) = 2.2;
Tau(1+2) = 2.3;
Tau(1+3) = 1.1;
Tau(1+4) = 1.2;
Tau(1+5) = 1.3;

qdd = FDab (model, Q, QDot, Tau, {}, [0; -9.81; 0])
disp ("id -------------------");
Tau2 = ID (model, Q, QDot, qdd, {}, [0; -9.81; 0])

res = Tau - Tau2

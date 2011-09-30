function [body] = CreateBody (mass, CoM, Icm)

% function [body] = CreateBody (mass, CoM, Icm)
%
% Creates a body for a given mass, center of mass and inertia tensor at the
% center of mass
%

body.inertia = mcI (mass, CoM, Icm);

function  [Xj,S] = jcalc( pitch, q, axis )

% jcalc  Calculate joint transform and motion subspace.
% [Xj,S]=jcalc(pitch,q) calculates the joint transform and motion subspace
% matrices for a revolute (pitch==0), prismatic (pitch==inf) or helical
% (pitch==any other value) joint.  For revolute and helical joints, q is
% the joint angle.  For prismatic joints, q is the linear displacement.

if nargin < 3
	axis = [0;0;1];
end

% type information:
% 0 rotz
% 1 roty
% 2 rotx

if pitch == 0				% revolute joint
	if axis == [0;0;1]
  	Xj = Xrotz(q);
  	S = [0;0;1;0;0;0];
	elseif axis == [0;1;0]
  	Xj = Xroty(q);
  	S = [0;1;0;0;0;0];
	elseif axis == [1;0;0]
  	Xj = Xrotx(q);
  	S = [1;0;0;0;0;0];
	else
		disp("invalid joint axis!")
		return
	end
elseif pitch == inf			% prismatic joint
  Xj = Xtrans([0 0 q]);
  S = [0;0;0;0;0;1];
else					% helical joint
  Xj = Xrotz(q) * Xtrans([0 0 q*pitch]);
  S = [0;0;1;0;0;pitch];
end

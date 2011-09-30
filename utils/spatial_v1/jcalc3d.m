function  [Xj,S] = jcalc3d( jaxis, pitch, q )

% jcalc  Calculate joint transform and motion subspace.
% [Xj,S]=jcalc(pitch,q) calculates the joint transform and motion subspace
% matrices for a revolute (pitch==0), prismatic (pitch==inf) or helical
% (pitch==any other value) joint.  For revolute and helical joints, q is
% the joint angle.  For prismatic joints, q is the linear displacement.

if pitch == 0				% revolute joint
	if jaxis == [1, 0, 0]
		Xj = Xrotx(q);
	elseif jaxis == [0, 1, 0]
		Xj = Xroty(q);
	elseif jaxis == [0, 0, 1]
		Xj = Xrotz(q);
	else
		error ("invalid joint axis")
	end

  S = [jaxis';0;0;0];
elseif pitch == inf			% prismatic joint
	if jaxis == [1, 0, 0]
		Xj = Xtrans(jaxis * q);
	elseif jaxis == [0, 1, 0]
		Xj = Xtrans(jaxis * q);
	elseif jaxis == [0, 0, 1]
		Xj = Xtrans(jaxis * q);
	else
		error ("invalid joint axis")
	end

  S = [0;0;0;jaxis'];
else					% helical joint
	error ("invalid joint axis")
end

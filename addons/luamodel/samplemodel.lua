inertia = { 
	{1.1, 0.1, 0.2},
	{0.3, 1.2, 0.4},
	{0.5, 0.6, 1.3}
}

pelvis = { mass = 9.3, com = { 1.1, 1.2, 1.3}, inertia = inertia }
thigh = { mass = 4.2, com = { 1.1, 1.2, 1.3}, inertia = inertia }
shank = { mass = 4.1, com = { 1.1, 1.2, 1.3}, inertia = inertia }
foot = { mass = 1.1, com = { 1.1, 1.2, 1.3}, inertia = inertia }

bodies = {
	pelvis = pelvis,
	thigh_right = thigh,
	shank_right = shank,
	thigh_left = thigh,
	shank_left = shank
}

joints = {
	freeflyer = {
		{ 0., 0., 0., 1., 0., 0.},
		{ 0., 0., 0., 0., 1., 0.},
		{ 0., 0., 0., 0., 0., 1.},
		{ 0., 0., 1., 0., 0., 0.},
		{ 0., 1., 0., 0., 0., 0.},
		{ 1., 0., 0., 0., 0., 0.}
	},
	spherical_zyx = {
		{ 0., 0., 1., 0., 0., 0.},
		{ 0., 1., 0., 0., 0., 0.},
		{ 1., 0., 0., 0., 0., 0.}
	},
	rotational_y = {
		{ 0., 1., 0., 0., 0., 0.}
	},
	fixed = {}
}

model = {
	frames = {
		{
			name = "pelvis",
			parent = "ROOT",
			body = bodies.pelvis,
			joint = joints.freeflyer,
		},
		{
			name = "thigh_right",
			parent = "pelvis",
			body = bodies.thigh_right,
			joint = joints.spherical_zyx,
		},
		{
			name = "shank_right",
			parent = "thigh_right",
			body = bodies.thigh_right,
			joint = joints.rotational_y
		},
		{
			name = "foot_right",
			parent = "shank_right",
			body = bodies.thigh_right,
			joint = joints.fixed
		},
		{
			name = "thigh_left",
			parent = "pelvis",
			body = bodies.thigh_left,
			joint = joints.spherical_zyx
		},
		{
			name = "shank_left",
			parent = "thigh_left",
			body = bodies.thigh_left,
			joint = joints.rotational_y
		},
		{
			name = "foot_left",
			parent = "shank_left",
			body = bodies.thigh_left,
			joint = joints.fixed
		},
	}
}

return model

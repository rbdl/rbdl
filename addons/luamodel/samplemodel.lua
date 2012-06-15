print ("heyho!")

inertia = { 
	{1.1, 0.1, 0.2},
	{0.3, 1.2, 0.4},
	{0.5, 0.6, 1.3}
}

pelvis = { name = "pelvis", mass = 1.1, com = { 1.1, 1.2, 1.3}, inertia = inertia }
thigh_right = { name = "thigh_right", mass = 91., com = { 1.1, 1.2, 1.3}, inertia = inertia }
shank_right = { name = "shank_right", mass = 91., com = { 1.1, 1.2, 1.3}, inertia = inertia }
thigh_left = { name = "thigh_left", mass = 91., com = { 1.1, 1.2, 1.3}, inertia = inertia }
shank_left = { name = "shank_left", mass = 91., com = { 1.1, 1.2, 1.3}, inertia = inertia }

bodies = {
	pelvis = pelvis,
	thigh_right = thigh_right,
	shank_right = shank_right,
	thigh_left = thigh_left,
	shank_left = shank_left
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
	fixed = {}
}

model = {
	frames = {
		{
			name = "pelvis",
			parent_body = 0,
			child_body = bodies.pelvis,
			joint = joints.freeflyer,
		},
		{
			name = "thigh_right",
			parent_body = bodies.pelvis,
			child_body = bodies.thigh_right,
			joint = joints.fixed
		},
	}
}

return model

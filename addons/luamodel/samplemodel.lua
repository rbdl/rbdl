print ("Hello, this is output from the model Lua file!")

inertia = { 
	{1.1, 0.1, 0.2},
	{0.3, 1.2, 0.4},
	{0.5, 0.6, 1.3}
}

pelvis = { mass = 1.1, com = { 1.1, 1.2, 1.3}, inertia = inertia }
thigh_right = { mass = 91., com = { 1.1, 1.2, 1.3}, inertia = inertia }
shank_right = { mass = 91., com = { 1.1, 1.2, 1.3}, inertia = inertia }
thigh_left = { mass = 91., com = { 1.1, 1.2, 1.3}, inertia = inertia }
shank_left = { mass = 91., com = { 1.1, 1.2, 1.3}, inertia = inertia }

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
			parent_frame = "BASE",
			body = bodies.pelvis,
			joint = joints.freeflyer,
		},
		{
			name = "thigh_right",
			parent_frame = "pelvis",
			body = bodies.thigh_right,
			joint = joints.spherical_zyx
		},
	}
}

return model

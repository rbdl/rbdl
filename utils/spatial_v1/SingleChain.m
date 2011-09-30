function qdd = SingleChain ()

model = CreateModel ();
body = CreateBody (1., [1.5, 0., 0.], diag ([1., 1., 1.]));
joint = CreateJoint ([0., 0., 1.]', 0.);

model = AddBody (model, 0, joint, Xtrans([0., 0., 0.]), body);

q = [0.];
qd = [0.];
tau = [0.];
f_ext = {};
grav_accn = [0., -9.81, 0.];

qdd = FDab (model, q, qd, tau, f_ext, grav_accn);

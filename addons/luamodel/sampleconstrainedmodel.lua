-- 5b3d.lua
-- Copyright (c) 2016 Davide Corradi <davide.corradi@iwr.uni-heidelberg.de>

-- Parameters

m1 = 2
l1 = 2
r1 = 0.2
Izz1 = m1 * l1 * l1 / 3 

m2 = 2
l2 = 2
r2 = 0.2
Izz2 = m2 * l2 * l2 / 3

bodies = {

  virtual = {
    mass = 0,
    com = {0, 0, 0},
    inertia = {
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0},
    },
  },

  link1 = {
    mass = m1,
    com = {l1/2, 0, 0},
    inertia = {
      {1, 0, 0},
      {0, 1, 0},
      {0, 0, Izz1},
    },
  },

  link2 = {
    mass = m2,
    com = {l2/2, 0, 0},
    inertia = {
      {1, 0, 0},
      {0, 1, 0},
      {0, 0, Izz2},
    },
  },

}

joints = {

  revZ = {
    {0, 0, 1, 0, 0, 0},
  },

  trnXYZ = {
    {0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 1},
  },

}

meshes = {
  link1 = {
    name = 'link1',
    dimensions = {l1, r1, r1},
    color = {1, 0, 0},
    src = 'unit_cylinder_medres_z.obj',
    mesh_center = {l1/2, 0, 0},
  },
  link2 = {
    name = 'link2',
    dimensions = {l2, r2, r2},
    color = {0, 1, 0},
    src = 'unit_cylinder_medres_z.obj',
    mesh_center = {l2/2, 0, 0},
  },
}

model = {

  gravity = {0, 0, 0},

   configuration = {
    axis_front = { 1.,  0.,  0.},
    axis_right = { 0., -1.,  0.},
    axis_up    = { 0.,  0.,  1.},
  },

  frames = {
    
    {
      name = 'base',
      parent = 'ROOT',
      body = bodies.virtual,
      joint = joints.trnXYZ,
    },
    {
      name = 'l11',
      parent = 'base',
      body = bodies.link1,
      joint = joints.revZ,
      visuals = { meshes.link1 },
    },
    {
      name = 'l12',
      parent = 'l11',
      body = bodies.link2,
      joint = joints.revZ,
      joint_frame = {
        r = {l1, 0, 0},
      },
      visuals = { meshes.link2 },
    },
    {
      name = 'l21',
      parent = 'base',
      body = bodies.link1,
      joint = joints.revZ,
      visuals = { meshes.link1 },
    },
    {
      name = 'l22',
      parent = 'l21',
      body = bodies.link2,
      joint = joints.revZ,
      joint_frame = {
        r = {l1, 0, 0},
      },
      visuals = { meshes.link2 },
    },

  },

  constraint_sets = {
    loop_constraints = {
      {
        constraint_type = 'loop',
        predecessor_body = 'l12',
        successor_body = 'l22',
        predecessor_transform = {
          E = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1},
          },
          r = {l2, 0, 0},
        },
        successor_transform = {
          E = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1},
          },
          r = {0, 0, 0},
        },
        axis = {0, 0, 0, 1, 0, 0},
        stabilization_coefficient = 0.1,
        name = 'linkTX',
      },

      {
        constraint_type = 'loop',
        predecessor_body = 'l12',
        successor_body = 'l22',
        predecessor_transform = {
          E = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1},
          },
          r = {l2, 0, 0},
        },
        successor_transform = {
          E = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1},
          },
          r = {0, 0, 0},
        },
        axis = {0, 0, 0, 0, 1, 0},
        stabilization_coefficient = 0.1,
        name = 'linkTY',
      },
    },

    all_constraints = {
      {
        constraint_type = 'contact',
        body = 'base',
        point = {0, 0, 0},
        normal = {1, 0, 0},
        name = 'baseTX',
        normal_acceleration = 0,
      },

      {
        constraint_type = 'contact',
        body = 'base',
        normal = {0, 1, 0},
        name = 'baseTY',
      },

      {
        constraint_type = 'contact',
        body = 'base',
        normal = {0, 0, 1},
        name = 'baseTZ',
      },

      {
        constraint_type = 'loop',
        predecessor_body = 'l12',
        successor_body = 'l22',
        predecessor_transform = {
          E = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1},
          },
          r = {l2, 0, 0},
        },
        successor_transform = {
          E = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1},
          },
          r = {0, 0, 0},
        },
        axis = {0, 0, 0, 1, 0, 0},
        stabilization_coefficient = 0.1,
        name = 'linkTX',
      },

      {
        constraint_type = 'loop',
        predecessor_body = 'l12',
        successor_body = 'l22',
        predecessor_transform = {
          E = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1},
          },
          r = {l2, 0, 0},
        },
        successor_transform = {
          E = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1},
          },
          r = {0, 0, 0},
        },
        axis = {0, 0, 0, 0, 1, 0},
        stabilization_coefficient = 0.1,
        name = 'linkTY',
      },
    },
  },

}

return model
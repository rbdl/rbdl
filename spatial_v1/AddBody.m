function [model_out, body_id] = AddBody (model, parent_id, joint_info, joint_transform, body_info)

% function [model_out, body_id] = AddBody (model, parent_id, joint_info, joint_transform, body_info)
%
% Adds a body to the given model.

model.NB = model.NB + 1;
body_id = model.NB;

model.parent = [model.parent, parent_id];
model.Xtree{body_id} = joint_transform;
model.jaxis{body_id} = joint_info.axis;
model.pitch = [model.pitch joint_info.pitch];
model.I{body_id} = body_info.inertia;

model_out = model;

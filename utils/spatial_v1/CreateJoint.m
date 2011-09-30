function [joint] = CreateJoint (joint_axis, joint_pitch)

% function [joint] = CreateJoint (joint_axis, joint_pitch)
%
% Creates a joint for a given axis and pitch.

joint.axis = joint_axis';
joint.pitch = joint_pitch;

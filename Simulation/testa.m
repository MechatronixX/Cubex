simulation_time = 3;
deg_ = pi/180;
s_ = 0;

% time, angle, release
% time: starting time of the current pose
% angle: angle of cube
% release: if 1, the cube is allowed to move according to the physics
%          if 0, it is still
procedure = [
    0*s_,   -45*deg_,           0
    5*s_,   cube_start_angle,   1
    ];

% Read data
% Find current procedure row
r = find(procedure(:,2) <= simulation_time,1,'last');
if isempty(r)
    r = 1;
    disp('Warning: no valid procedure.')
end
cube_angle = procedure(r,2);

% Write data
cube_states = [cube_angle;0;0;0;0];

release = procedure(r,3);

if release
    x0 = [cube_angle;0;0];
else
    x0 = zeros(3,1);
end
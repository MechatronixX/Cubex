function cube_states = GetCubeStates2DStruct(state_vector)
%cube_states = GetCubeStates2DStruct(state_vector)
%
% Create cube state 2D struct from vector
%
% Verkar inte fungera i Matlab function block tillsammans med switch

if length(state_vector) ~= 5
    error('Incorrect state vector size')
end

cube_states.cube_angle                  = state_vector(1);
cube_states.cube_angular_velocity       = state_vector(2);
cube_states.cube_angular_acceleration   = state_vector(3);
cube_states.wheel_angular_velocity      = state_vector(4);
cube_states.wheel_angular_acceleration  = state_vector(5);
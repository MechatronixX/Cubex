function cube_states = GetCubeStates2DVector(state_struct)
%cube_states = GetCubeStates2DVector(state_struct)
%
% Create cube state 2D struct from vector

if length(state_vector) ~= 5
    error('Incorrect state vector size')
end

cube_states = [state_struct.cube_angle;
               state_struct.cube_angular_velocity;
               state_struct.cube_angular_acceleration;
               state_struct.wheel_angular_velocity;
               state_struct.wheel_angular_acceleration];

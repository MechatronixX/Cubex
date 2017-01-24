close all

t                           = cube_states.cube_angle.Time;
cube_angle                  = cube_states.cube_angle.Data;
cube_angular_velocity       = cube_states.cube_angular_velocity.Data;
cube_angular_acceleration   = cube_states.cube_angular_acceleration.Data;

x_cog = -l_corner2cog*sin(cube_angle);
y_cog =  l_corner2cog*cos(cube_angle);

x_dot_cog = -l_corner2cog*cube_angular_velocity.*cos(cube_angle);
y_dot_cog = -l_corner2cog*cube_angular_velocity.*sin(cube_angle);

x_dot_cog_calc = diff(x_cog)./diff(t);
y_dot_cog_calc = diff(y_cog)./diff(t);

figure
plot(x_cog,y_cog)
axis equal
grid on

figure
title('Position')
subplot(2,1,1)
plot(t,x_cog)
grid on
subplot(2,1,2)
plot(t,y_cog)
grid on

title('Velocity')
figure
subplot(2,1,1)
hold all
plot(t,x_dot_cog)
plot(t(1:end-1),x_dot_cog_calc)
grid on
subplot(2,1,2)
hold all
plot(t,y_dot_cog)
plot(t(1:end-1),y_dot_cog_calc)
grid on
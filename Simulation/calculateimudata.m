close all

t                           = cube_states.cube_angle.Time;
cube_angle                  = cube_states.cube_angle.Data;
cube_angular_velocity       = cube_states.cube_angular_velocity.Data;
cube_angular_acceleration   = cube_states.cube_angular_acceleration.Data;

w_imu_s = zeros(3,length(t));
a_imu_s = zeros(3,length(t));
a_part1 = zeros(3,length(t));
a_part2 = zeros(3,length(t));
a_part3 = zeros(3,length(t));
v_cog   = zeros(3,length(t));

for i = 1:length(t)
    w = [0;0;cube_angular_velocity(i)];
    w_dot = [0;0;cube_angular_acceleration(i)];
    r_vec = e2t([0;0;cube_angle(i)])*l_corner2cog*[0;1;0];
    g_vec = g*[0;1;0];
    
    C_i_b = e2t([0;0;cube_angle(i)]); % Verifiera tecken
    C_b_s = e2t(imu_misalignment); % Verifiera tecken
    
    % Body reference frame, inertial coordinates
    w_imu_i = [0;0;cube_angular_velocity(i)];
    a_imu_i = cross(w_dot,r_vec) + cross(w,cross(w,r_vec)) + g_vec;
    
    % Body reference frame, sensor coordinates
    w_imu_s(:,i) = w_imu_i; % Rotating around the same axis
    a_imu_s(:,i) = C_b_s*C_i_b*a_imu_i;
    
    % Save for plotting
    v_cog(:,i)   = cross(w,r_vec); % Save for plotting
    a_part1(:,i) = cross(w_dot,r_vec);
    a_part2(:,i) = cross(w,cross(w,r_vec));
    a_part3(:,i) = g_vec;
end

%% Plot

x_cog = -l_corner2cog*sin(cube_angle);
y_cog =  l_corner2cog*cos(cube_angle);

x_dot_cog = -l_corner2cog*cube_angular_velocity.*cos(cube_angle);
y_dot_cog = -l_corner2cog*cube_angular_velocity.*sin(cube_angle);

cube_coordinates = [0,0;1/sqrt(2)*side_length,1/sqrt(2)*side_length;0,sqrt(2)*side_length;-1/sqrt(2)*side_length,1/sqrt(2)*side_length;0,0]';

quiver_stretch = 0.02;
i = 1;
figure
hold all
plot(x_cog,y_cog)
plot(0,0,'ok','linewidth',5)
h0=plot(cube_coordinates(1,:),cube_coordinates(2,:),'k');
h1=plot(x_cog(i),y_cog(i),'o');
% h2=quiver(x_cog(i),y_cog(i),v_cog(1,i),v_cog(2,i))
h3=quiver(x_cog(i),y_cog(i),a_part1(1,i)+a_part2(1,i)+a_part3(1,i),a_part1(2,i)+a_part2(2,i)+a_part3(2,i),quiver_stretch);
h4=quiver(x_cog(i),y_cog(i),a_part1(1,i),a_part1(2,i),quiver_stretch);
h5=quiver(x_cog(i),y_cog(i),a_part2(1,i),a_part2(2,i),quiver_stretch);
h6=quiver(x_cog(i),y_cog(i),a_part3(1,i),a_part3(2,i),quiver_stretch);
legend([h1,h3,h4,h5,h6],'COG','a imu','d/dt(\omega) x r_{vec}','\omega x v','g','Location','EastOutside')
grid on
axis equal
% axis([-0.2 0.2 -0.2 0.2])

for i = 150:length(t)
    i2b = e2t([0;0;cube_angle(i)]);
    new_cube_coordinates = zeros(size(cube_coordinates));
    for i_c = 1:size(cube_coordinates,2)
        temp = i2b*[cube_coordinates(:,i_c);0];
        new_cube_coordinates(:,i_c) = temp(1:2);
    end
    set(h0,'XData',new_cube_coordinates(1,:));
    set(h0,'YData',new_cube_coordinates(2,:));
    set(h1,'XData',x_cog(i))
    set(h1,'YData',y_cog(i))
    % set(h2,'XData',x_cog(i))
    % set(h2,'YData',y_cog(i))
    % set(h2,'UData',v_cog(1,i))
    % set(h2,'VData',v_cog(2,i))
    set(h3,'XData',x_cog(i))
    set(h3,'YData',y_cog(i))
    set(h3,'UData',a_part1(1,i)+a_part2(1,i)+a_part3(1,i))
    set(h3,'VData',a_part1(2,i)+a_part2(2,i)+a_part3(2,i))
    set(h4,'XData',x_cog(i))
    set(h4,'YData',y_cog(i))
    set(h4,'UData',a_part1(1,i))
    set(h4,'VData',a_part1(2,i))
    set(h5,'XData',x_cog(i))
    set(h5,'YData',y_cog(i))
    set(h5,'UData',a_part2(1,i))
    set(h5,'VData',a_part2(2,i))
    set(h6,'XData',x_cog(i))
    set(h6,'YData',y_cog(i))
    set(h6,'UData',a_part3(1,i))
    set(h6,'VData',a_part3(2,i))
    
    drawnow
end


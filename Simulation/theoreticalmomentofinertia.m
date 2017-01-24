% Ber�kna kuben som sex sidor, roterandes kring en kant
% Tv� sidor �r ortogonal (o) mot kanten, fyra sidor �r parallella (p)

mass_cube = 3.487;
mass_wheel = 0.15; % Estimated
radius_wheel = 0.13;
side_length = 0.15;

m_s = mass_cube/6; % Massa av sida
s = side_length;     % Sidl�ngd
% m_s = sym('m_s'); % Massa av sida
% s = sym('s');     % Sidl�ngd

I_p = m_s*s^2/6;
I_o = m_s*s^2/12;

% Anv�nd Steiners sats (parallel axis theorem)
I_c = 2*(I_p + m_s*2*(s/2)^2) ...
    + 2*(I_o + m_s*s^2/4) ...
    + 2*(I_o + m_s*((s/2)^2+s^2))
    
% double(I_c)

I_w = mass_wheel*(radius_wheel*0.9)^2+mass_wheel*side_length^2/2
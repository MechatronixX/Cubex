% Beräkna kuben som sex sidor, roterandes kring en kant
% Två sidor är ortogonal (o) mot kanten, fyra sidor är parallella (p)

mass_cube = 3.487;
mass_wheel = 0.15; % Estimated
radius_wheel = 0.13;
side_length = 0.15;

m_s = mass_cube/6; % Massa av sida
s = side_length;     % Sidlängd
% m_s = sym('m_s'); % Massa av sida
% s = sym('s');     % Sidlängd

I_p = m_s*s^2/6;
I_o = m_s*s^2/12;

% Använd Steiners sats (parallel axis theorem)
I_c = 2*(I_p + m_s*2*(s/2)^2) ...
    + 2*(I_o + m_s*s^2/4) ...
    + 2*(I_o + m_s*((s/2)^2+s^2))
    
% double(I_c)

I_w = mass_wheel*(radius_wheel*0.9)^2+mass_wheel*side_length^2/2
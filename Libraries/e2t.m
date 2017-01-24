function t = e2t(euler_angles)
%t = E2T(euler_angles)
% Calculates transformation matrix from Euler angles.
% Does not take vectors as input.
% euler_angles = [phi,theta,psi]

N = length(euler_angles);
if N ~= 3
    error( ' ***  Fel argument till e2t !  *** ' );
end
phi   = euler_angles(1);
theta = euler_angles(2);
psi   = euler_angles(3);

cos_psi = cos(psi) ;
sin_psi = sin(psi) ;
cos_theta  = cos(theta) ;
sin_theta  = sin(theta) ;
cos_phi  = cos(phi) ;
sin_phi  = sin(phi) ;

t = [cos_psi*cos_theta -sin_psi*cos_phi+cos_psi*sin_theta*sin_phi  sin_psi*sin_phi+cos_psi*sin_theta*cos_phi
    sin_psi*cos_theta  cos_psi*cos_phi+sin_psi*sin_theta*sin_phi -cos_psi*sin_phi+sin_psi*sin_theta*cos_phi
    -sin_theta          cos_theta*sin_phi                       cos_theta*cos_phi          ];


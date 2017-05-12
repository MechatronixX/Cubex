function eulerZXZ = quatern2eulerZXZ(q)
%QUATERN2EULERZXZ Converts a quaternion orientation to ZXZ Euler angles
%
%   eulerZXZ = quatern2eulerZXZ(q)
%
%   Converts a quaternion orientation to ZXZ Euler angles where phi is a
%   rotation around X, theta around Y and psi around Z.
%
%	Date      
%	09/05/2017    

    R = zeros(3,3,'single');    % Added 3 dimensional result array to make
                                % it c-code generation friendly
    
    R(1,3,:) = 2.*(q(:,2).*q(:,4)-q(:,1).*q(:,3));
    R(2,3,:) = 2.*(q(:,3).*q(:,4)+q(:,1).*q(:,2));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;

    phi = -atan2(R(1,3,:), R(2,3,:) );
    theta = atan( sqrt(1-R(3,3,:).^2) ./ R(3,3,:)); %Use 'acos(R(3,3,:) );' 
                                                    %depending on witch
                                                    %quadrant is needed
    psi = atan2(R(3,1,:), R(3,2,:) );

    eulerZXZ = [phi(1,:)' theta(1,:)' psi(1,:)'];
end

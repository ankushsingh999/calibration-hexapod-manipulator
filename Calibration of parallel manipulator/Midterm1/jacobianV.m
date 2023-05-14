

% Velocity Jacobian
function J = jacobianV1(P)
[l n R S] = NIK(P);
J = [n(:,1)' , cross(R * S(:,1), n(:,1))';
     n(:,2)' , cross(R * S(:,2), n(:,2))';
     n(:,3)' , cross(R * S(:,3), n(:,3))';
     n(:,4)' , cross(R * S(:,4), n(:,4))';
     n(:,5)' , cross(R * S(:,5), n(:,5))';
     n(:,6)' , cross(R * S(:,6), n(:,6))'];
 end
% Forward Kinematics
function NFK
% Step #1: Initial Guess

P0 = [0 0 150 0 0 0]';
%Declaring the leg lengths 
%lg = [262.9644  255.2174  266.0690  275.3756  279.1254  245.1648]';
nk = [92.1597 84.4488 0 305.4001 111.1565 0 604.8652;
       27.055 122.037 0 -56.4357 320.0625 0 604.8652;
      -119.2146 37.5882 0 -248.9644 208.9060 0 604.8652;
      -119.2146 -37.5882 0 -248.9644 -208.9060 0 604.8652;
       27.055 -122.037 0 -56.4357 -320.0625 0 604.8652;
       92.1597 -84.4488 0 305.4001 -111.1565 0 604.8652;]';
% ns = nk(1:3,:); %nominal s
% nu = nk(4:6,:);% nominal u
%lg = nk(7,:)' %nominal Leg length
%lg = [266.6702  346.0546  381.6413  527.7681  521.1529  300.1173]'
P(:,1) = P0;
i = 2;
dl = 1;
while dl > 0.0001
   % Step #2
   J = jacobianV(P(:,i-1));

   % Step #3: Calculate T
   a = P(4,i-1)*pi/180;
   b = P(5,i-1)*pi/180;
   c = P(6,i-1)*pi/180;
   B = [1       0      sin(b);
        0     cos(a)  -sin(a)*cos(b);
        0     sin(a)   cos(a)*cos(b)];
   T = [eye(3)       zeros(3,3)
        zeros(3,3)   B];

   % Step #4: IK
   [l n R s] = NIK(P(:,i-1));

   Dl = lg - l';
   dl = norm(Dl , 2);

   % Step #5
   P(:,i) = P(:,i-1) + pinv(J*T) * Dl;
   %Dp = norm(P(:,1) - P(:,1) , 2);
  i = i+1;

end
P(:,i-1)


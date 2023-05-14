%Inverse Kinematics of Parallel Robots with Prismatic Legs

function [l, n, R, S, U] = NIK(P)
%Used for testing the code
%% Desired Pose (Given)
% P = [4 5 782 9 2 3]';
%P = [ 10.0000 20.0000 150.0000 0.0961 0.2000 0.3039]';

%% Extracting Position and Euler Angle Information from the given desired pose

o=P(1:3,1); 
a=P(4)*pi/180; 
b=P(5)*pi/180; 
c=P(6)*pi/180;

%% Calculating Rotation Matrix from Euler Angles. XYZ Euler angles.

R1=[1,0,0;0,cos(a),-sin(a);0,sin(a),cos(a)]; % Rx,a
R2=[cos(b),0,sin(b);0,1,0;-sin(b),0,cos(b)]; % Ry,b
R3=[cos(c),-sin(c),0;sin(c),cos(c),0;0,0,1]; % Rz,c

R = R1*R2*R3;
%nominal values from the paper
nk = [92.1597 84.4488 0 305.4001 111.1565 0 604.8652;
       27.055 122.037 0 -56.4357 320.0625 0 604.8652;
      -119.2146 37.5882 0 -248.9644 208.9060 0 604.8652;
      -119.2146 -37.5882 0 -248.9644 -208.9060 0 604.8652;
       27.055 -122.037 0 -56.4357 -320.0625 0 604.8652;
       92.1597 -84.4488 0 305.4001 -111.1565 0 604.8652;]';
%the first three values are the s vector, next three the u vector and last
%value th eleg length
ns = nk(1:3,:); %nominal s
nu = nk(4:6,:);% nominal u
nl = nk(7,:); %nominal Leg length

%% Calculating upper joint positions wrt. the upper coordinate frame

S = [ns(:,1) , ns(:,2) , ns(:,3), ns(:,4) , ns(:,5) , ns(:,6)];


%% Calculating lower joint positions wrt. the lower corrdinate frame

 
U = [nu(:,1) , nu(:,2) , nu(:,3) , nu(:,4) , nu(:,5) , nu(:,6)];


%% Put it in the IK equation

for i = 1:6
    L (: , i) = o + R * S(: , i) - U (: , i);
    l(i) = norm (L (: , i));
    n(:,i) = L (: , i)/l(i);

end
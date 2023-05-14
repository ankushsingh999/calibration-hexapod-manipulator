%Inverse Kinematics of Parallel Robots with Prismatic Legs

function  [l n R S] = RIK(P)

%% Desired Pose (Given)
%P = [10 20 150 7 3 5]';
%P = [ 10.0000 20.0000 150.0000 0.0961 0.2000 0.3039]';


%% Robot Parameters (Given)
Rm=250/2;
Rf=650/2;
alpha=40*pi/180;
beta=80*pi/180;

%% Extracting Position and Euler Angle Information from the given desired pose

o=P(1:3,1); 
a=P(4)*pi/180; 
b=P(5)*pi/180; 
c=P(6)*pi/180;

%% Calculating Rotation Matrix from Euler Angles

R1=[1,0,0;0,cos(a),-sin(a);0,sin(a),cos(a)]; % Rx,a
R2=[cos(b),0,sin(b);0,1,0;-sin(b),0,cos(b)]; % Ry,b
R3=[cos(c),-sin(c),0;sin(c),cos(c),0;0,0,1]; % Rz,c

%R = R1*R2*R3; % Rxyz
R = R1*R2*R3; % Rzyz

rk = [96.6610 81.7602 1.0684 305.2599 115.0695 2.6210 604.4299;
       22.2476 125.2511 -0.5530 -55.2814 322.9819 4.2181 607.2473;
      -122.4519 36.6453 4.3547 -244.7954 208.0087 3.9365 600.4441;
      -120.6859 -34.4565 -4.9014 -252.5755 -211.8783 -3.0128 605.9031;
       24.7769 -125.0489 -4.8473 -53.9678 -320.6115 4.3181 604.5251;
       91.3462 -80.9866 0.2515 302.4266 -109.4351 3.3812 600.0616;]';
rs = rk(1:3,:);%real s
ru = rk(4:6,:);%real k
rl = rk(7,:);%real l
%% Calculating upper joint positions wrt. the upper coordinate frame

S = rs;
%% Calculating lower joint positions wrt. the lower corrdinate frame

 
U = ru;


%% Put it in the IK equation

for i = 1:6
    L (: , i) = o + R * S(: , i) - U (: , i);
    l(i) = norm (L (: , i));
    n(:,i) = L (: , i)/l(i);

end
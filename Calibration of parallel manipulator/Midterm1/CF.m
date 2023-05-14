
function F = CF(X)
%The Cf takes the initial guess as input.
%% Taking M configurations

%Random Configurations choosen. 
% pt = [
%    0 0 800 0 0 0 
%    120 380 782 2 3 7
%    -67 -78 800 0 0 0
%    45 76 1000 0 0 0
%    56 80 750 3 5 6
%    45 3  816 9 8 5
%    32 28 760 1 0 8
%    0 0 1045 8 7 5
%    9 8 800 9 8 3
%    54 -45 900 0 4 5
%    5 67 900 8 7 2
%    4 5 782 9 2 3
%    ];

%Configurations closer to the boundary of the workspace. 16 configurations
%are taken in total
pt = [
    210 532 750 0 0 0
    -500 312 750 0 0 0
    -370 -518 750 0 0 0
    385 -428 750 0 0 0
    -85 542 800 0 0 0
    -535 -3 800 0 0 0
    -60 -543 800 0 0 0
     595 2 800 0 0 0
    -190 172 1000 0 0 0
    -195 -158 1000 0 0 0
    170 -163 1000 0 0 0
    205 127 1000 0 0 0
    -15 117 1050 0 0 0
    -105 -3 1050 0 0 0
    -10 -113 1050 0 0 0
    140 2 1050 0 0 0 
    ];
%% Nominal Leg length
% Value is taken from the nominal values given to us. All the six nominal
% leg lengths are equal.

nor_l =  604.8652;
% m gives the largest dimension of the matrix pt which is the no.of
% configurations
m = length(pt);

%% Measured Pose
% Using the nominal inverse kinematics and cascading the result to real
% forward kinematics, the measured pose is obtained.

for i = 1:m
    pm(i,:) = (RFK(NIK(pt(i,:)')))';
end

%% Cost (Error) function
% j will be the number of error equations we will have 
j = 0;
for k = 1:m
    for i= 1:6
        j = j+1;
        %finding out the leg length from IK
        [l n R S U] = NIK(pt(k,:)');
        %Calculating the Rotation matrix from the last three values of the
        %initiial guess. XYZ euler angles.
        a = pm(k,4)*pi/180;
        b = pm(k,5)*pi/180;
        c = pm(k,6)*pi/180;
        R1=[1,0,0;0,cos(a),-sin(a);0,sin(a),cos(a)]; % Rx,a
        R2=[cos(b),0,sin(b);0,1,0;-sin(b),0,cos(b)]; % Ry,b
        R3=[cos(c),-sin(c),0;sin(c),cos(c),0;0,0,1]; % Rz,c
        R = R1*R2*R3;
        %Finding out the delta l required to add to the nominal leg length
        %to reach the desired pose
        del_l = l - nor_l;
        %Cost Function 
        F(j) = ((norm(pm(k,1:3)'+R*X(1:3,i)-X(4:6,i))).^2 - (norm(X(7,i)+del_l(i))).^2).^2;
    end
end

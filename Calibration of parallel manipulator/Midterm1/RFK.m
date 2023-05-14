% Forward Kinematics
function P = RFK(lg)
% Step #1: Initial Guess
P0 = [0 0 150 0 0 0]';

%Used for testing the code
%lg = [257.8396  250.6678  263.2683  279.7943  271.3325  242.5687]

%Simulated real parameters given in the table
rk = [96.6610 81.7602 1.0684 305.2599 115.0695 2.6210 604.4299;
       22.2476 125.2511 -0.5530 -55.2814 322.9819 4.2181 607.2473;
      -122.4519 36.6453 4.3547 -244.7954 208.0087 3.9365 600.4441;
      -120.6859 -34.4565 -4.9014 -252.5755 -211.8783 -3.0128 605.9031;
       24.7769 -125.0489 -4.8473 -53.9678 -320.6115 4.3181 604.5251;
       91.3462 -80.9866 0.2515 302.4266 -109.4351 3.3812 600.0616;]';
%Nominal Values given in the table 
nk = [92.1597 84.4488 0 305.4001 111.1565 0 604.8652;
       27.055 122.037 0 -56.4357 320.0625 0 604.8652;
      -119.2146 37.5882 0 -248.9644 208.9060 0 604.8652;
      -119.2146 -37.5882 0 -248.9644 -208.9060 0 604.8652;
       27.055 -122.037 0 -56.4357 -320.0625 0 604.8652;
       92.1597 -84.4488 0 305.4001 -111.1565 0 604.8652;]';

%Taking the real and nominal leg length
rlg = rk(7,:);
nlg = nk(7,:);

%finding the difference between real and nominal leg length, which will
%give an error to be added to the leg lengths which will be given as
%inputs.
e_lg = rlg - nlg;
lg = lg + e_lg;

P(:,1) = P0;
i = 2;
dl = 1;

%Considering the leg length error
while dl > 0.000001
   % Step #2
   J = jacobianRV(P(:,i-1));
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
   [l n R s] = RIK(P(:,i-1));

   Dl = lg' - l';
   dl = norm(Dl , 2);

   % Step #5
   P(:,i) = P(:,i-1) + pinv(J*T) * Dl;
   %Dp = norm(P(:,1) - P(:,1) , 2);
  i = i+1;

end
P = P(:,i-1);


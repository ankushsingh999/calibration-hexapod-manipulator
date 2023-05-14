
%%
% *RBE 521- Legged Robotics - Midterm - Fall 2022* 
%%
% *Calibration of a Hexapod Table by simulation* 


function calibrate 
%% Initial Guess
% %% The initialguess are the nominal values given in the table 
Initialguess = [92.1597 84.4488 0 305.4001 111.1565 0 604.8652;
       27.055 122.037 0 -56.4357 320.0625 0 604.8652;
      -119.2146 37.5882 0 -248.9644 208.9060 0 604.8652;
      -119.2146 -37.5882 0 -248.9644 -208.9060 0 604.8652;
       27.055 -122.037 0 -56.4357 -320.0625 0 604.8652;
       92.1597 -84.4488 0 305.4001 -111.1565 0 604.8652;]';
%% Minimization of Cost Function 
% lsqnonlin is used to minimize the cost function by taking the initial
% guess and predicting the kinematic parameters
IdentifiedValues = lsqnonlin(@CF , Initialguess)'

%% Given Real and Nominal Values
% Declaring the real and nominal values for comparision 

%simulated real kinematic parameters
real_values  = [96.6610 81.7602 1.0684 305.2599 115.0695 2.6210 604.4299;
       22.2476 125.2511 -0.5530 -55.2814 322.9819 4.2181 607.2473;
      -122.4519 36.6453 4.3547 -244.7954 208.0087 3.9365 600.4441;
      -120.6859 -34.4565 -4.9014 -252.5755 -211.8783 -3.0128 605.9031;
       24.7769 -125.0489 -4.8473 -53.9678 -320.6115 4.3181 604.5251;
       91.3462 -80.9866 0.2515 302.4266 -109.4351 3.3812 600.0616;]
%nominal kinematic parameters
nominal_values = [92.1597 84.4488 0 305.4001 111.1565 0 604.8652;
       27.055 122.037 0 -56.4357 320.0625 0 604.8652;
      -119.2146 37.5882 0 -248.9644 208.9060 0 604.8652;
      -119.2146 -37.5882 0 -248.9644 -208.9060 0 604.8652;
       27.055 -122.037 0 -56.4357 -320.0625 0 604.8652;
       92.1597 -84.4488 0 305.4001 -111.1565 0 604.8652;];

%% Bar graph for the values obtained 
error_real_nominal = real_values-nominal_values
error_values = real_values - IdentifiedValues
figure
graphs = [abs(reshape(error_values,[],1)),abs(reshape(error_real_nominal,[],1))];
bar3(graphs);
title('IDENTIFIED ERRORS IN KINEMATIC PARAMETERS')
zlabel('Error(mm)') 
ylabel('Kinematic Parameters') 
legend({'before calibration','after calibration'},'Location','northeast')
end
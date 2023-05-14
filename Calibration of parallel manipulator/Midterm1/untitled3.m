funtion config

nk = [92.1597 84.4488 0 305.4001 111.1565 0 604.8652;
       27.055 122.037 0 -56.4357 320.0625 0 604.8652;
      -119.2146 37.5882 0 -248.9644 208.9060 0 604.8652;
      -119.2146 -37.5882 0 -248.9644 -208.9060 0 604.8652;
       27.055 -122.037 0 -56.4357 -320.0625 0 604.8652;
       92.1597 -84.4488 0 305.4001 -111.1565 0 604.8652;]';

ns = nk(1:3,:); %nominal s
nu = nk(4:6,:);% nominal u
nl = nk(7,:); %nominal Leg length

csn = nu - ns;


% at height 300
rwc1 = sqrt(lmax.^2- 300.^2);

ip1 = [];
for i = 1:5
    [x,y]= circcirc(csn(1,i), csn(2,i),rwc,csn(1,i+1), csn(2,i+1),rwc1);
    for i =1:2
    ip = vertcat(ip,[x(i),y(i)]);
    i = i+1;
    end
end
[x,y] = circcirc(csn(1,1), csn(2,1),rwc,csn(1,6), csn(2,6),rwc1);
    for i =1:2
    ip = vertcat(ip,[x(i),y(i)]);
    end
    ip;
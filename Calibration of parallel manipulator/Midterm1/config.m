function config

nk = [92.1597 84.4488 0 305.4001 111.1565 0 604.8652;
       27.055 122.037 0 -56.4357 320.0625 0 604.8652;
      -119.2146 37.5882 0 -248.9644 208.9060 0 604.8652;
      -119.2146 -37.5882 0 -248.9644 -208.9060 0 604.8652;
       27.055 -122.037 0 -56.4357 -320.0625 0 604.8652;
       92.1597 -84.4488 0 305.4001 -111.1565 0 604.8652;]';

ns = nk(1:3,:); %nominal s
nu = nk(4:6,:);% nominal u
nl = nk(7,:); %nominal Leg length
lmax = 1100; 
csn = nu - ns;

%------------------------------------------------------------------------
%------------------Height 800--------------------------------------------
% at height 800
rwc1 = sqrt(lmax.^2- 800.^2);

ip1 = [];
for i = 1:5
    [x,y]= circcirc(csn(1,i), csn(2,i),rwc1,csn(1,i+1), csn(2,i+1),rwc1);
    for i =1:2
    ip1 = vertcat(ip1,[x(i),y(i)]);
    i = i+1;
    end
end
[x,y] = circcirc(csn(1,1), csn(2,1),rwc1,csn(1,6), csn(2,6),rwc1);
    for i =1:2
    ip1 = vertcat(ip1,[x(i),y(i)]);
    end
    ip1

xr = -865:5:967;
yr = -838:5:838;
mesh_pts = [];
graph = NaN(size(xr,2),size(yr,2));
for i = 1:size(xr,2)
    for j = 1:size(yr,2)
        ct = 0;
        for k = 1:6
            if (((xr(i)-csn(1,k))^2 + (yr(j)-csn(2,k))^2) <= (rwc1)^2)
                ct = ct + 1;
            end
        end
        if (ct >= 6)
            mesh_pts = vertcat(mesh_pts,[xr(i),yr(j)]);
            graph(i,j) = 1;
        end
    end
end
mesh_pts;
 
%Plots the worksoace plane at height Z= 800
figure
plot(mesh_pts(:,1),mesh_pts(:,2),'r*')
title("Workspace Plane of the Hexapod at Height 800")
%----------------------------------------------------------------------------
%--------------------------------------------------------------------------
%------------------------------------------------------------------------
%------------------Height 1000--------------------------------------------
% at height 800
rwc2 = sqrt(lmax.^2- 1000.^2);

ip2 = [];
for i = 1:5
    [x,y]= circcirc(csn(1,i), csn(2,i),rwc2,csn(1,i+1), csn(2,i+1),rwc2);
    for i =1:2
    ip2 = vertcat(ip2,[x(i),y(i)]);
    i = i+1;
    end
end
[x,y] = circcirc(csn(1,1), csn(2,1),rwc2,csn(1,6), csn(2,6),rwc2);
    for i =1:2
    ip2 = vertcat(ip2,[x(i),y(i)]);
    end
    ip2

xr = -865:5:967;
yr = -838:5:838;
mesh_pts2 = [];
graph = NaN(size(xr,2),size(yr,2));
for i = 1:size(xr,2)
    for j = 1:size(yr,2)
        ct = 0;
        for k = 1:6
            if (((xr(i)-csn(1,k))^2 + (yr(j)-csn(2,k))^2) <= (rwc2)^2)
                ct = ct + 1;
            end
        end
        if (ct >= 6)
            mesh_pts2 = vertcat(mesh_pts2,[xr(i),yr(j)]);
            graph(i,j) = 1;
        end
    end
end
mesh_pts;
 
%Plots the worksoace plane at height Z= 1000
figure
plot(mesh_pts2(:,1),mesh_pts2(:,2),'b*')
title("Workspace Plane of the Hexapod at Height 1000")
%title("Workspace Plane of the Hexapod at Height 1000")
%--------------------------------------------------------------------------
% %--------------------------------------------------------------------------
% %------------------------------------------------------------------------
% %------------------Height 1050--------------------------------------------
% % at height 1050
rwc3 = sqrt(lmax.^2- 1055.^2);

ip3 = [];
for i = 1:5
    [x,y]= circcirc(csn(1,i), csn(2,i),rwc3,csn(1,i+1), csn(2,i+1),rwc3);
    for i =1:2
    ip3 = vertcat(ip3,[x(i),y(i)]);
    i = i+1;
    end
end
[x,y] = circcirc(csn(1,1), csn(2,1),rwc3,csn(1,6), csn(2,6),rwc3);
    for i =1:2
    ip3 = vertcat(ip3,[x(i),y(i)]);
    end
    ip3

xr = -865:5:967;
yr = -838:5:838;
mesh_pts3 = [];
graph = NaN(size(xr,2),size(yr,2));
for i = 1:size(xr,2)
    for j = 1:size(yr,2)
        ct = 0;
        for k = 1:6
            if (((xr(i)-csn(1,k))^2 + (yr(j)-csn(2,k))^2) <= (rwc3)^2)
                ct = ct + 1;
            end
        end
        if (ct >= 6)
            mesh_pts3 = vertcat(mesh_pts3,[xr(i),yr(j)]);
            graph(i,j) = 1;
        end
    end
end
mesh_pts3;
 
%Plots the worksoace plane at height Z= 1050
figure
plot(mesh_pts3(:,1),mesh_pts3(:,2),'r*')
title("Workspace Plane of the Hexapod at Height 1055")
% %--------------------------------------------------------------------------
% %--------------------------------------------------------------------------
% %------------------------------------------------------------------------
% %------------------Height 1080--------------------------------------------
% % at height 
rwc4 = sqrt(lmax.^2- 750.^2);

ip4 = [];
for i = 1:5
    [x,y]= circcirc(csn(1,i), csn(2,i),rwc4,csn(1,i+1), csn(2,i+1),rwc4);
    for i =1:2
    ip4 = vertcat(ip4,[x(i),y(i)]);
    i = i+1;
    end
end
[x,y] = circcirc(csn(1,1), csn(2,1),rwc4,csn(1,6), csn(2,6),rwc4);
    for i =1:2
    ip4 = vertcat(ip4,[x(i),y(i)]);
    end
    ip4

xr = -865:5:967;
yr = -838:5:838;
mesh_pts4 = [];
graph = NaN(size(xr,2),size(yr,2));
for i = 1:size(xr,2)
    for j = 1:size(yr,2)
        ct = 0;
        for k = 1:6
            if (((xr(i)-csn(1,k))^2 + (yr(j)-csn(2,k))^2) <= (rwc4)^2)
                ct = ct + 1;
            end
        end
        if (ct >= 6)
            mesh_pts4 = vertcat(mesh_pts4,[xr(i),yr(j)]);
            graph(i,j) = 1;
        end
    end
end
mesh_pts4;
 
%Plots the worksoace plane at height Z= 1050
figure
plot(mesh_pts4(:,1),mesh_pts4(:,2),'r*')
title("Workspace Plane of the Hexapod at Height 750")

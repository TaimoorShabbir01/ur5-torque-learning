%% 00_quick_visualizations.m
% Quick plots to sanity‑check UR5 model & workspace

ur5 = loadrobot('universalUR5','DataFormat','row','Gravity',[0,0,-9.81]); 
initialConfig = homeConfiguration(ur5);

figure('Name','UR5 Manipulator'); clf
show(ur5, initialConfig, 'Visuals','on','Frames','on');
title('UR5 Manipulator');
axis equal; hold on; grid on
axis([-1.5 1.5 -1.5 1.5 0 1.5]);

figure('Name','UR5 Workspace'); clf
[x,y,z] = sphere(20);
z(z<0) = NaN;
surf(x,y,z,'FaceAlpha',0.1,'EdgeColor','none');
title('UR5 Workspace (Upper Hemisphere)');
xlabel X; ylabel Y; zlabel Z; grid on; axis equal

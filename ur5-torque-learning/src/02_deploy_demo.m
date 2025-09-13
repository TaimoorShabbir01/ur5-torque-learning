%% 02_deploy_demo.m
% Deploy immediately with model from workspace (assumes you've just run 01_*)

assert(exist('net','var')==1 && exist('inputPs','var')==1 && exist('outputPs','var')==1, ...
    'Run 01_generate_data_and_train.m first to have net/inputPs/outputPs in workspace.');

ur5 = loadrobot('universalUR5','DataFormat','row','Gravity',[0,0,-9.81]); 
initialConfig = homeConfiguration(ur5);

figure('Name','UR5 Deployment'); clf
show(ur5, initialConfig, 'Visuals','on','Frames','on');
title('UR5 Manipulator - Deployment');
axis([-1.5 1.5 -1.5 1.5 0 1.5]); hold on; grid on

currentConfig = initialConfig;
targetPos     = [0.5, 0.4, 0.5];

stateHistory = [];
iteration = 0;
maxIterations = 2000;
timeStep = 0.01;
configThreshold = 1e-3;

disp('Starting model deployment...');
while norm(tform2trvec(getTransform(ur5, currentConfig, 'tool0','base')) - targetPos) > 0.1 ...
        && iteration < maxIterations

    iteration = iteration + 1;
    jointVel = zeros(1,6);
    jointAcc = zeros(1,6);

    inputVec = [currentConfig, jointVel, jointAcc, targetPos];
    inputVecNorm = mapminmax('apply', inputVec', inputPs)'; 
    torquesNorm = double(predict(net, inputVecNorm));
    torques = mapminmax('reverse', torquesNorm', outputPs)';
    torques = max(min(torques, 1), -1);

    newConfig = currentConfig + torques * timeStep;

    if norm(newConfig - currentConfig) < configThreshold
        disp('Configuration update too small; stopping to prevent stalling.');
        break;
    end

    currentConfig = newConfig;
    stateHistory  = [stateHistory; currentConfig];

    if mod(iteration,10)==0
        show(ur5, double(currentConfig), 'PreservePlot', false, 'Visuals','on','Frames','off');
        drawnow;
    end

    if mod(iteration,50)==0
        eePos = tform2trvec(getTransform(ur5, currentConfig, 'tool0','base'));
        disp(['Iter ', num2str(iteration), ' EE pos: ', mat2str(eePos)]);
        disp(['Target: ', mat2str(targetPos)]);
    end
end

finalDist = norm(tform2trvec(getTransform(ur5, currentConfig, 'tool0','base')) - targetPos);
if iteration == maxIterations
    disp('Max iterations reached. Target may be unreachable.');
elseif finalDist <= 0.1
    disp('Target reached successfully!');
else
    disp('Deployment terminated due to small configuration updates.');
end

if ~isempty(stateHistory)
    endEffPositions = arrayfun(@(i) tform2trvec(getTransform(ur5, stateHistory(i,:), 'tool0','base')), ...
        1:size(stateHistory,1), 'UniformOutput', false);
    endEffPositions = cell2mat(endEffPositions');

    figure('Name','End-Effector Trajectory'); clf
    plot3(endEffPositions(:,1), endEffPositions(:,2), endEffPositions(:,3), '-o'); hold on
    plot3(targetPos(1),targetPos(2),targetPos(3),'rx','MarkerSize',10,'LineWidth',2);
    xlabel X; ylabel Y; zlabel Z; grid on; legend('Trajectory','Target');
    title('Manipulator End-Effector Trajectory');
end

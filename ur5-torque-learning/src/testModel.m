%% ---------------- Load Trained Model ----------------
% (Make sure 'myTrainedModel.mat' is in your path)
load('myTrainedModel.mat','net','inputPs','outputPs');

%% ---------------- UR5 Setup ----------------
ur5 = loadrobot('universalUR5','DataFormat','row','Gravity',[0,0,-9.81]); 
initialConfig = homeConfiguration(ur5);

% Visualization figure
figure('Name','UR5 Deployment');
show(ur5, initialConfig, 'Visuals','on','Frames','on');
title('UR5 Manipulator - Deployment');
hold on;
axis([-1.5 1.5 -1.5 1.5 0 1.5]);

%% ---------------- Deployment Parameters ----------------
% Start at home configuration
currentConfig = initialConfig;   
targetPos     = [0.5, 0.5, 0.5];  % Example target in Cartesian space

% Deployment loop setup
stateHistory  = [];              
iteration     = 0;               
maxIterations = 2000;            
timeStep      = 0.01;            
configThreshold = 1e-3;          

disp('Starting model deployment...');

while norm(tform2trvec(getTransform(ur5, currentConfig, 'tool0','base')) - targetPos) > 0.1 ...
      && iteration < maxIterations
  
    iteration = iteration + 1;
    
    % Assume zero velocity and acceleration for this simple example
    jointVel = zeros(1,6);
    jointAcc = zeros(1,6);
    
    % Prepare the input for the trained network
    % IMPORTANT: we must include the target so that the network “knows” it
    % was trained on (jointState + jointVel + jointAcc + target)
    inputVec = [currentConfig, jointVel, jointAcc, targetPos];

    % Normalize the input using the SAME mapminmax parameters from training
    inputVecNorm = mapminmax('apply', inputVec', inputPs);
    inputVecNorm = inputVecNorm'; 
    
    % Predict normalized torques using the trained model
    torquesNorm = double(predict(net, inputVecNorm));
    
    % Convert the predicted torques back to the original scale
    torques = mapminmax('reverse', torquesNorm', outputPs);
    torques = torques';
    
    % (Optionally) clamp torques to some safe range
    torques = max(min(torques, 1), -1);
    
    % Update joint configuration (simple Euler step)
    newConfig = currentConfig + torques*timeStep;
    
    % Check if the update is too small => possible stalling
    if norm(newConfig - currentConfig) < configThreshold
        disp('Configuration update is too small, stopping to prevent stalling.');
        break;
    end
   

    % Update the current configuration
    currentConfig = newConfig;
    stateHistory  = [stateHistory; currentConfig];
    
    % Visualize every 10 iterations
    if mod(iteration, 10) == 0
        show(ur5, currentConfig, 'PreservePlot', false, 'Visuals','on','Frames','off');
        drawnow;
    end
    
    % Debug info every 50 iterations
    if mod(iteration, 50) == 0
        eePos = tform2trvec(getTransform(ur5, currentConfig, 'tool0','base'));
        disp(['Iteration ', num2str(iteration), ...
              ': End-Effector Position: ', mat2str(eePos)]);
        disp(['Target Position: ', mat2str(targetPos)]);
    end
end

% Check if target is reached
finalDist = norm(tform2trvec(getTransform(ur5, currentConfig, 'tool0','base')) - targetPos);
if iteration == maxIterations
    disp('Max iterations reached. Target may be unreachable.');
elseif finalDist <= 0.1
    disp('Target reached successfully!');
else
    disp('Deployment terminated due to small configuration updates.');
end

%% ---------------- Visualize Trajectory ----------------
if ~isempty(stateHistory)
    endEffPositions = arrayfun(@(i) tform2trvec(getTransform(ur5, ...
        stateHistory(i,:), 'tool0','base')), 1:size(stateHistory,1), 'UniformOutput', false);
    endEffPositions = cell2mat(endEffPositions');

    figure('Name','End-Effector Trajectory');
    plot3(endEffPositions(:,1), endEffPositions(:,2), endEffPositions(:,3), '-o');
    hold on;
    plot3(targetPos(1), targetPos(2), targetPos(3), 'rx','MarkerSize',10,'LineWidth',2);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Manipulator End-Effector Trajectory');
    legend('Trajectory','Target');
    grid on;
end

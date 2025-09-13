%% ---------------- Load the UR5 Manipulator ----------------
% Load full UR5 model
ur5 = loadrobot('universalUR5','DataFormat','row','Gravity',[0,0,-9.81]); 

% Define a home (initial) configuration
initialConfig = homeConfiguration(ur5);

% Figure 1: Visualize UR5 manipulator
figure('Name','UR5 Manipulator');
show(ur5, initialConfig, 'Visuals', 'on', 'Frames', 'on');
title('UR5 Manipulator');
hold on;
axis([-1.5 1.5 -1.5 1.5 0 1.5]);
disp('UR5 manipulator loaded and visualized successfully.');

% Figure 2: Visualize workspace (a semisphere)
figure('Name','UR5 Workspace');
[x, y, z] = sphere(20);
z(z < 0) = NaN; % Remove the lower hemisphere
surf(x, y, z, 'FaceAlpha', 0.1, 'EdgeColor', 'none');
title('UR5 Workspace');
xlabel('X'); ylabel('Y'); zlabel('Z');
disp('Workspace visualized successfully.');

%% ---------------- Data Generation with Target [0, 1] ----------------
numSamples = 5000;  % Increased number of samples

% Preallocate arrays
jointStates        = zeros(numSamples, 6);   % Random joint positions
jointVelocities    = zeros(numSamples, 6);   % Random joint velocities
jointAccelerations = zeros(numSamples, 6);   % Random joint accelerations
targets            = zeros(numSamples, 3);   % Random Cartesian target positions
actions            = zeros(numSamples, 6);   % Corresponding joint torques

disp('Generating training data using Inverse Dynamics...');

for i = 1:numSamples
    % Generate random joint positions, velocities, and accelerations
    jointStates(i, :)        = rand(1, 6)*2*pi - pi;  % in [-pi, pi]
    jointVelocities(i, :)    = rand(1, 6)*2 - 1;      % in [-1, 1]
    jointAccelerations(i, :) = rand(1, 6)*2 - 1;      % in [-1, 1]
    
    % Generate random target positions in [0,1]
    targets(i, :) = rand(1, 3);
    
    % Compute torques using Inverse Dynamics
    actions(i, :) = inverseDynamics(ur5, ...
                                    jointStates(i,:), ...
                                    jointVelocities(i,:), ...
                                    jointAccelerations(i,:));
end

% Combine data into a MATLAB table (for reference)
dataTable = table( (1:numSamples)', ...
                   jointStates, ...
                   jointVelocities, ...
                   jointAccelerations, ...
                   targets, ...
                   actions, ...
    'VariableNames', {'SampleIndex','JointStates','JointVelocities','JointAccelerations','TargetPositions','Torques'});

disp('Data generation complete. Displaying first few rows of table:');
disp(dataTable(1:5,:));

%% ---------------- Model Training ----------------
% Include targets in the input so the network can condition on desired end-effector position.
inputs  = [jointStates, jointVelocities, jointAccelerations, targets]; 
outputs = actions;  % The torques

% For consistent normalization, use mapminmax so we can apply the same scaling later
[inputsNorm, inputPs] = mapminmax(inputs', -1, 1);
inputsNorm = inputsNorm';  % revert to row orientation

[outputsNorm, outputPs] = mapminmax(outputs', -1, 1);
outputsNorm = outputsNorm'; 

% Split into training and testing sets
trainRatio = 0.7;  
idx = floor(trainRatio * numSamples);

trainX = inputsNorm(1:idx, :);
trainY = outputsNorm(1:idx, :);
testX  = inputsNorm(idx+1:end, :);
testY  = outputsNorm(idx+1:end, :);

% Define neural network architecture
layers = [
    featureInputLayer(size(trainX,2), 'Name','input')     % e.g., 6+6+6+3 = 21 features
    fullyConnectedLayer(256, 'Name','fc1')                % Increased neurons
    batchNormalizationLayer('Name','bn1')                 
    reluLayer('Name','relu1')                            
    fullyConnectedLayer(128, 'Name','fc2')               
    dropoutLayer(0.3, 'Name','dropout1')                 
    reluLayer('Name','relu2')                            
    fullyConnectedLayer(64, 'Name','fc3')                
    reluLayer('Name','relu3')                            
    fullyConnectedLayer(size(trainY,2), 'Name','fc_out')  % Output: 6 torques
    regressionLayer('Name','reg_out')
];

% Define training options
options = trainingOptions('adam', ...
    'MaxEpochs', 100, ...
    'MiniBatchSize', 32, ...
    'InitialLearnRate', 1e-4, ...
    'Shuffle','every-epoch', ...
    'ValidationData', {testX, testY}, ...
    'ValidationFrequency', 10, ...
    'ValidationPatience', 10, ...
    'Plots','training-progress', ...
    'Verbose',true, ...
    ... % Requires MATLAB R2022b or later; remove if older.
    'OutputNetwork','best-validation-loss' ...
);

disp('Training the neural network...');
net = trainNetwork(trainX, trainY, layers, options);
disp('Training complete.');

% Test the model on the test set
predY_norm = predict(net, testX);
% Convert predicted torques back to original scale
predY = mapminmax('reverse', predY_norm', outputPs);
predY = predY';

testY_original = mapminmax('reverse', testY', outputPs);
testY_original = testY_original';

mse_val = mean((testY_original - predY).^2, 'all');
disp(['Test MSE: ', num2str(mse_val)]);

%% ---------------- Deployment ----------------
% Initialize the starting configuration and target position
currentConfig = initialConfig;   
targetPos     = [0.5, 0.4, 0.5];  % Example target in Cartesian space

% To store the joint configurations over time
stateHistory  = [];               
iteration     = 0;               
maxIterations = 2000;            
timeStep      = 0.01;            % smaller time step
configThreshold = 1e-3;          % threshold for minimal change

disp('Starting model deployment...');

while norm(tform2trvec(getTransform(ur5, currentConfig, 'tool0','base')) - targetPos) > 0.1 ...
      && iteration < maxIterations

    iteration = iteration + 1;
    
    % Assume zero velocity/acceleration at each iteration in this simplistic example:
    jointVel = zeros(1,6);
    jointAcc = zeros(1,6);
    
    % IMPORTANT: Include targetPos in the input
    inputVec = [currentConfig, jointVel, jointAcc, targetPos];
    
    % Apply the SAME mapminmax transformation from training
    inputVecNorm = mapminmax('apply', inputVec', inputPs);
    inputVecNorm = inputVecNorm'; 
    
    % Predict normalized torques
    torquesNorm = double(predict(net, inputVecNorm));
    
    % Map predicted torques back to original scale
    torques = mapminmax('reverse', torquesNorm', outputPs);
    torques = torques';
    
    % (Optionally) clamp torques to [-1, 1] if desired:
    torques = max(min(torques, 1), -1);
    
    % Update joint configuration by simple Euler integration
    newConfig = currentConfig + torques * timeStep;
    
    % Check if configuration change is too small
    if norm(newConfig - currentConfig) < configThreshold
        disp('Configuration update is too small, stopping to prevent stalling.');
        break;
    end
    
    % Update current configuration
    currentConfig = newConfig;
    stateHistory  = [stateHistory; currentConfig];
    
    % Visualize the robot's movement every 10 iterations
    if mod(iteration, 10) == 0
        show(ur5, double(currentConfig), 'PreservePlot', false, 'Visuals','on','Frames','off');
        drawnow;
    end
    
    % Display debug info every 50 iterations
    if mod(iteration, 50) == 0
        eePos = tform2trvec(getTransform(ur5, currentConfig, 'tool0','base'));
        disp(['Iteration ', num2str(iteration), ...
              ': End-Effector Position: ', mat2str(eePos)]);
        disp(['Target Position: ', mat2str(targetPos)]);
    end
end

% Check if the target was reached or loop ended
finalDist = norm(tform2trvec(getTransform(ur5, currentConfig, 'tool0','base')) - targetPos);
if iteration == maxIterations
    disp('Maximum iterations reached. Target may be unreachable.');
elseif finalDist <= 0.1
    disp('Target reached successfully!');
else
    disp('Deployment terminated due to small configuration updates.');
end

%% ---------------- Visualize Trajectory ----------------
% Convert each row of stateHistory to an end-effector position
endEffPositions = arrayfun(@(i) tform2trvec(getTransform(ur5, ...
                                    stateHistory(i,:), ...
                                    'tool0','base')), ...
                                    1:size(stateHistory,1), 'UniformOutput', false);
endEffPositions = cell2mat(endEffPositions');

figure('Name','End-Effector Trajectory');
plot3(endEffPositions(:,1), endEffPositions(:,2), endEffPositions(:,3), '-o');
hold on;
plot3(targetPos(1), targetPos(2), targetPos(3), 'rx','MarkerSize',10,'LineWidth',2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Manipulator End-Effector Trajectory');
legend('Trajectory','Target');
grid on;

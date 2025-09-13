%% 01_generate_data_and_train.m
% Generate data via inverse dynamics, train NN, and save myTrainedModel.mat

%% Load UR5 & visualize
ur5 = loadrobot('universalUR5','DataFormat','row','Gravity',[0,0,-9.81]); 
initialConfig = homeConfiguration(ur5);

figure('Name','UR5 Manipulator'); clf
show(ur5, initialConfig, 'Visuals','on','Frames','on');
title('UR5 Manipulator');
axis([-1.5 1.5 -1.5 1.5 0 1.5]); grid on

figure('Name','UR5 Workspace'); clf
[x,y,z] = sphere(20);
z(z<0) = NaN;
surf(x,y,z,'FaceAlpha',0.1,'EdgeColor','none');
title('UR5 Workspace (Upper Hemisphere)'); xlabel X; ylabel Y; zlabel Z; grid on; axis equal

%% Data generation
numSamples = 5000;
jointStates        = zeros(numSamples, 6);
jointVelocities    = zeros(numSamples, 6);
jointAccelerations = zeros(numSamples, 6);
targets            = zeros(numSamples, 3);
actions            = zeros(numSamples, 6);

disp('Generating training data using Inverse Dynamics...');
for i = 1:numSamples
    jointStates(i,:)        = rand(1,6)*2*pi - pi;
    jointVelocities(i,:)    = rand(1,6)*2 - 1;
    jointAccelerations(i,:) = rand(1,6)*2 - 1;
    targets(i,:)            = rand(1,3);         % in [0,1]^3
    
    actions(i,:) = inverseDynamics(ur5, ...
        jointStates(i,:), jointVelocities(i,:), jointAccelerations(i,:));
end

dataTable = table( (1:numSamples)', jointStates, jointVelocities, ...
    jointAccelerations, targets, actions, ...
    'VariableNames', {'SampleIndex','JointStates','JointVelocities','JointAccelerations','TargetPositions','Torques'});
disp(head(dataTable));

%% Prepare training data
inputs  = [jointStates, jointVelocities, jointAccelerations, targets];
outputs = actions;

[inputsNorm, inputPs]   = mapminmax(inputs',  -1, 1); inputsNorm  = inputsNorm';
[outputsNorm, outputPs] = mapminmax(outputs', -1, 1); outputsNorm = outputsNorm';

trainRatio = 0.7; idx = floor(trainRatio * numSamples);
trainX = inputsNorm(1:idx,:);   trainY = outputsNorm(1:idx,:);
testX  = inputsNorm(idx+1:end,:); testY  = outputsNorm(idx+1:end,:);

%% Define model
layers = [
    featureInputLayer(size(trainX,2), 'Name','input')
    fullyConnectedLayer(256, 'Name','fc1')
    batchNormalizationLayer('Name','bn1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(128, 'Name','fc2')
    dropoutLayer(0.3, 'Name','drop1')
    reluLayer('Name','relu2')
    fullyConnectedLayer(64, 'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(size(trainY,2), 'Name','fc_out')
    regressionLayer('Name','reg_out')
];

options = trainingOptions('adam', ...
    'MaxEpochs', 100, ...
    'MiniBatchSize', 32, ...
    'InitialLearnRate', 1e-4, ...
    'Shuffle','every-epoch', ...
    'ValidationData',{testX,testY}, ...
    'ValidationFrequency',10, ...
    'ValidationPatience',10, ...
    'Plots','training-progress', ...
    'Verbose',true, ...
    'OutputNetwork','best-validation-loss' ...  % comment this line if MATLAB < R2022b
);

disp('Training the neural network...');
net = trainNetwork(trainX, trainY, layers, options);
disp('Training complete.');

%% Evaluate
predY_norm = predict(net, testX);
predY      = mapminmax('reverse', predY_norm', outputPs)'; 
testY_orig = mapminmax('reverse', testY',     outputPs)'; 

mse_val = mean((testY_orig - predY).^2, 'all');
disp(['Test MSE: ', num2str(mse_val)]);

%% Save artifacts for deployment
save('../myTrainedModel.mat','net','inputPs','outputPs','-v7.3');
disp('Saved model to ../myTrainedModel.mat');

% Radar Point Cloud Generator using Driving Scenario
% Generates exportable point cloud data from simulated radar returns

clear all %#ok<CLALL>
which pointCloud -all

%% Create Driving Scenario
scenario = drivingScenario('SampleTime', 0.1, 'StopTime', 10);

road(scenario, [0 0; 100 0], 'Lanes', lanespec(2));

egoVehicle = vehicle(scenario, 'ClassID', 1, 'Position', [0 0 0]);
waypoints = [0 0 0; 50 0 0];
speed = 15; % m/s
trajectory(egoVehicle, waypoints, speed);

targetVeh1 = vehicle(scenario, 'ClassID', 1, 'Position', [30 0 0]);
targetVeh2 = vehicle(scenario, 'ClassID', 1, 'Position', [45 -3.5 0]);
waypoints2 = [45 -3.5 0; 70 -3.5 0];
trajectory(targetVeh2, waypoints2, 10);

%% Visualize Driving Scenario
figScenario = figure('Name', 'Driving Scenario Visualization', 'NumberTitle', 'off');
axScenario = axes(figScenario);
plot(scenario, 'Parent', axScenario);
title(axScenario, 'Driving Scenario Visualization');
xlabel(axScenario, 'X (m)');
ylabel(axScenario, 'Y (m)');
grid(axScenario, 'on');
hold(axScenario, 'on');

%% Configure Radar Sensor
radarSensor = drivingRadarDataGenerator('SensorIndex', 1, ...
    'MountingLocation', [egoVehicle.Wheelbase + egoVehicle.FrontOverhang, 0, 0.2], ...
    'RangeLimits', [0 100], ...
    'RangeResolution', 0.5, ...
    'AzimuthResolution', 4, ...
    'HasElevation', true, ...
    'FieldOfView', [90, 20], ...
    'TargetReportFormat', 'Detections');

%% Simulation Loop - Collect Point Cloud Data
allPointClouds = {}; % Store all point clouds
timeStamps = [];
frameIdx = 1;

while advance(scenario)
    % Get current time
    currentTime = scenario.SimulationTime;

    % Get target poses relative to ego vehicle
    tgtPoses = targetPoses(egoVehicle);

    % Generate radar detections
    [dets, ~, isValidTime] = radarSensor(tgtPoses, currentTime);

    % Update the driving scenario visualization
    updatePlots(scenario);
    pause(0.05);  % small delay to allow animation playback

    if isValidTime && ~isempty(dets)
        % Extract point cloud data from detections
        numDets = numel(dets);
        points = zeros(numDets, 3); % [x, y, z]
        intensity = zeros(numDets, 1);
        velocity = zeros(numDets, 3);

        for i = 1:numDets
            % Get measurement (range, azimuth, elevation, range-rate)
            meas = dets{i}.Measurement;

            % Convert spherical to Cartesian coordinates
            range = meas(1);
            azimuth = meas(2);
            elevation = meas(3);
            rangeRate = meas(4);

            % Spherical to Cartesian
            x = range * cosd(elevation) * cosd(azimuth);
            y = range * cosd(elevation) * sind(azimuth);
            z = range * sind(elevation);

            points(i, :) = [x, y, z];

            % Store SNR as intensity
            intensity(i) = dets{i}.ObjectAttributes{1}.SNR;

            % Velocity (radial velocity in x-direction for simplicity)
            velocity(i, :) = [rangeRate, 0, 0];
        end

        % Create point cloud structure
        pcStruct = struct();
        pcStruct.Location = points;
        pcStruct.Intensity = intensity;
        pcStruct.Velocity = velocity;
        pcStruct.Time = currentTime;

        % Store
        allPointClouds{frameIdx} = pcStruct; %#ok<SAGROW>
        timeStamps(frameIdx) = currentTime; %#ok<SAGROW>
        frameIdx = frameIdx + 1;

        % Display progress
        fprintf('Frame %d: %d detections at t=%.2f s\n', frameIdx-1, numDets, currentTime);
    end
end

%% Export Point Cloud Data

% % Option 1: Save as MAT file
save('radar_pointcloud_data.mat', 'allPointClouds', 'timeStamps');

% Make sure that output folder exists
% outputFolder = 'sample_data';
% if ~exist(outputFolder, 'dir')
%     mkdir(outputFolder);
% end

% % Option 2: Export to CSV (for each frame)
% for i = 1:length(allPointClouds)
%     pc = allPointClouds{i};

%     % Create table
%     T = array2table([pc.Location, pc.Intensity, pc.Velocity], ...
%         'VariableNames', {'X', 'Y', 'Z', 'Intensity', 'Vx', 'Vy', 'Vz'});

%     % Write to CSV
%     filename = sprintf('sample_data/pointcloud_frame_%03d.csv', i);
%     writetable(T, filename);
% end

% % Option 3: Export to PCD format (Point Cloud Data)
% % Requires Computer Vision Toolbox
% if license('test', 'video_and_image_blockset')
%     for i = 1:length(allPointClouds)
%         pc = allPointClouds{i};

%         % Create pointCloud object
%         % ptCloud = pointCloud(pc.Location, 'Intensity', pc.Intensity(:));

%         % Detailed diagnostics
%         disp('Checking for invalid values:');
%         disp(['Any NaN in Location: ', num2str(any(isnan(pc.Location(:))))]);
%         disp(['Any Inf in Location: ', num2str(any(isinf(pc.Location(:))))]);
%         disp(['Any NaN in Intensity: ', num2str(any(isnan(pc.Intensity(:))))]);
%         disp(['Any Inf in Intensity: ', num2str(any(isinf(pc.Intensity(:))))]);

%         % Check actual values
%         disp('First few Location values:');
%         disp(pc.Location(1:min(3,end),:));
%         disp('First few Intensity values:');
%         disp(pc.Intensity(1:min(3,end)));

%         % Try creating without Intensity first
%         try
%             ptCloud = pointCloud(pc.Location);
%             disp('SUCCESS: pointCloud created without Intensity');
%         catch ME
%             disp(['FAILED without Intensity: ', ME.message]);
%         end

%         % Try with Intensity
%         try
%             ptCloud = pointCloud(pc.Location, 'Intensity', pc.Intensity(:));
%             disp('SUCCESS: pointCloud created with Intensity');
%         catch ME
%             disp(['FAILED with Intensity: ', ME.message]);
%         end

%         % Write PCD file
%         filename = sprintf('pointcloud_frame_%03d.pcd', i);
%         pcwrite(ptCloud, filename);
%     end
% end

%% Visualize Sample Point Cloud
% figure;
% pc = allPointClouds{end}; % Last frame
% scatter3(pc.Location(:,1), pc.Location(:,2), pc.Location(:,3), ...
%     50, pc.Intensity, 'filled');
% colorbar;
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z (m)');
% title(sprintf('Radar Point Cloud at t=%.2f s', pc.Time));
% grid on;
% axis equal;
% view(0, 90); % Top-down view

% fprintf('\nExport complete: %d frames saved\n', length(allPointClouds));


%% === Load Radar Data ===
filename = 'radar_pointcloud_data.mat';  % your .mat file name
data = load(filename);

% The variable inside the .mat file
allPointClouds = data.allPointClouds;  % adjust if different

% Pick a frame to visualize (e.g., last frame)
pc = allPointClouds{end};

% Compute approximate range and radial velocity
ranges = sqrt(sum(pc.Location.^2, 2));  % Euclidean distance from radar
velocities = pc.Velocity(:,1);          % x-component of velocity (forward)
snr = pc.Intensity;                     % SNR for color

%% === Plot Approximate Range–Velocity Map ===
figure;
scatter(velocities, ranges, 50, snr, 'filled');  % point size 50
colormap(jet);
colorbar;
xlabel('Velocity (m/s)');
ylabel('Range (m)');
title(sprintf('Approx. Range–Velocity Map at t=%.2f s', pc.Time));
grid on;
set(gca, 'YDir', 'reverse');  % optional: radar convention

%% === Clear and Setup ===
clear all %#ok<CLALL>

% --- Radar Hardware Specs ---
f0 = 60e9;                   % Center frequency (Hz)
noRx = 4;                    % Number of receive antennas
noChirps = 32;               % Number of pulses (chirps) in a frame
fs = 12.5e6;                 % ADC sampling rate (Hz)
noADC = 256;                 % Number of ADC samples per chirp
slp = 144.858e12;            % Chirp rate (Hz/s)
PRF = 1 / 200e-6;              % Pulse Repetition Frequency (Hz)

% --- Simulation Control ---
noFrames = 800;              % Number of frames to simulate
frameRate = 155;             % Rate at which frames are collected (Hz)
allFramesData = cell(1, noFrames);  % Cell array

% --- Derived Parameters ---
c = physconst('LightSpeed');
tm = noADC / fs;             % Sweep time (s)
bw = slp * tm;               % Sweep bandwidth (Hz)
rangeRes = c / (2 * bw);       % Actual range resolution (m)
pri = 1 / PRF;               % Pulse Repetition Interval (s)
wavelength = c / f0;

fprintf('System Range Resolution: %.3f meters\n', rangeRes);

%% === Driving Scenario ===
timePerFrame = 1 / frameRate; % This is your new SampleTime
requiredSimTime = noFrames * timePerFrame; % This is your new StopTime
epsilon = timePerFrame/2; % For floating point precision

% Reference: https://www.mathworks.com/help/driving/ref/drivingscenario.trajectory.html

scenario = drivingScenario('StopTime',requiredSimTime + epsilon,'SampleTime',timePerFrame);
road1 = [50 1 0; 2 0.9 0];
road2 = [27 24 0; 27 -21 0];
laneSpecification = lanespec(2,'Width',4);
road(scenario, road1, 'Lanes', laneSpecification);
road(scenario, road2, 'Lanes', laneSpecification);

egoVehicle = vehicle(scenario,'ClassID',1,'Position',[5 -1 0]);
waypoints = [5 -1 0; 16 -1 0; 40 -1 0];
speed = [30; 0; 30];
waittime = [0; 0.3; 0];
trajectory(egoVehicle,waypoints,speed,waittime);

% car = vehicle(scenario,'ClassID',1,'Position',[48 4 0],'PlotColor',[0.494 0.184 0.556], 'Name','Car');
% waypoints = [47 3 0; 38 3 0; 10 3 0];
% speed = [30; 0; 30];
% waittime = [0; 0.3; 0];
% trajectory(car,waypoints,speed,waittime);

ambulance = vehicle(scenario,'ClassID',6,'Position',[25 22 0],'PlotColor',[0.466 0.674 0.188],'Name','Ambulance');
waypoints = [25 22 0; 25 13 0; 25 6 0; 26 2 0; 33 -1 0; 45 -1 0];
speed = 25;
trajectory(ambulance,waypoints,speed);

targetVehicle = ambulance;

%% === Preprocessing ===
waveform = phased.FMCWWaveform('SweepTime', tm, 'SweepBandwidth', bw, 'SampleRate', fs);

% Single antenna sending signal in all directions (kind of like a point light)
txAntenna = phased.IsotropicAntennaElement('FrequencyRange', [58e9 62e9]); % f0 +- bw/2

% Straight line of receivers
rxArray = phased.ULA('NumElements', noRx, 'ElementSpacing', wavelength/2);

% Amplifies outgoing signal
tx = phased.Transmitter('PeakPower', 0.1, 'Gain', 30);

% Amplifies incoming signal
rx = phased.ReceiverPreamp('Gain', 20, 'NoiseFigure', 5, 'SampleRate', fs);

% Narrowband signal radiator
radiator = phased.Radiator('Sensor', txAntenna, 'OperatingFrequency', f0);

% Collects incoming signal and turn into data
collector = phased.Collector('Sensor', rxArray, 'OperatingFrequency', f0);

% Simulates the air in between object and radar (e.g., air drag)
channel = phased.FreeSpace('PropagationSpeed', c, 'OperatingFrequency', f0, ...
    'SampleRate', fs, 'TwoWayPropagation', false);

% Simulates target reflection (simulated RCS)
target = phased.RadarTarget('MeanRCS', 10, 'Model', 'Nonfluctuating', 'OperatingFrequency', f0);

% Range-Doppler object
rdresp = phased.RangeDopplerResponse(...
    'RangeMethod', 'FFT', 'DopplerOutput', 'Speed', 'SweepSlope', slp, ...
    'SampleRate', fs, 'OperatingFrequency', f0);

%% === Plotting ===

% Driving environment
figScenario = figure('Name', 'Driving Scenario', 'Position', [100, 300, 700, 600]);
axScenario = axes(figScenario);
plot(scenario, 'Parent', axScenario);
title(axScenario, 'Live Driving Scenario');

% Range-Doppler Map
figRD = figure('Name', 'Range-Doppler Map', 'Position', [850, 300, 700, 600]);
axRD = axes(figRD);

% zeros = empty array of zeros (m by n) where m = # range pulses, n = # chirps
[~, range_grid, speed_grid] = rdresp(zeros(noADC, noChirps));
hRD = imagesc(axRD, speed_grid, range_grid, zeros(length(range_grid), length(speed_grid)));
hTitleRD = title(axRD, 'Range-Doppler Map (Initializing...)');
xlabel(axRD, 'Velocity (m/s)');
ylabel(axRD, 'Range (m)');
colorbar(axRD);
clim(axRD, [-20, 40]);

numSamples = noADC;
fprintf('Starting real-time simulation...\n');

for frameIdx = 1:noFrames

    % Advance the scenario and check if it's still running
    isRunning = advance(scenario);
    if ~isRunning
        fprintf('\nSimulation stopped after %d frames.\n', frameIdx - 1);
        break;
    end

    frameEchoes = zeros(numSamples, noChirps, noRx);

    egoPos = egoVehicle.Position.';
    egoVel = egoVehicle.Velocity.';
    posTgt = targetVehicle.Position.';
    velTgt = targetVehicle.Velocity.';
    [~, angTgt] = rangeangle(posTgt, egoPos);

    for k = 1:noChirps
        sig = waveform();
        sig_tx = tx(sig);
        sig_rad = radiator(sig_tx, angTgt);
        sig_prop = channel(sig_rad, egoPos, posTgt, egoVel, velTgt);
        sig_refl = target(sig_prop);
        sig_prop_back = channel(sig_refl, posTgt, egoPos, velTgt, egoVel);
        sig_collected = collector(sig_prop_back, angTgt);
        frameEchoes(:, k, :) = rx(sig_collected);
    end

    % Get the first antenna channel
    firstChannelData = frameEchoes(:, :, 1);
    [resp, ~, ~] = rdresp(firstChannelData);

    % Redraw the driving env with new vehicle positions
    plot(scenario, 'Parent', axScenario);

    % Update Range-Doppler map
    set(hRD, 'CData', mag2db(abs(resp)));
    set(hTitleRD, 'String', sprintf('Range-Doppler Map (Frame %d, Time %.2f s)', ...
        frameIdx, scenario.SimulationTime));

    drawnow;
    allFramesData{frameIdx} = frameEchoes; % Store frame data
end

fprintf('Simulation finished.\n');

%% === Save Raw Radar Data ===
save('raw_multichannel_radar_data.mat', 'allFramesData', 'waveform', 'f0', 'tm', 'slp', '-v7.3');
fprintf('Raw radar data for %d frames saved to raw_multichannel_radar_data.mat\n', noFrames);

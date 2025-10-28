%% === Clear and Setup ===
clear all %#ok<CLALL>
clc;

%% === Driving Scenario ===
scenario = drivingScenario('SampleTime',0.1,'StopTime',10);

road(scenario,[0 0; 100 0],'Lanes',lanespec(2));

egoVehicle = vehicle(scenario,'ClassID',1,'Position',[0 0 0]);
waypoints = [0 0 0; 50 0 0];
speed = 15;
trajectory(egoVehicle,waypoints,speed);

targetVeh1 = vehicle(scenario,'ClassID',1,'Position',[30 0 0]);
targetVeh2 = vehicle(scenario,'ClassID',1,'Position',[45 -3.5 0]);
waypoints2 = [45 -3.5 0; 70 -3.5 0];
trajectory(targetVeh2,waypoints2,10);

%% === Radar Parameters ===
fc = 77e9;             % Carrier frequency
c = physconst('LightSpeed');

rangeRes = 0.5;        % desired range resolution (meters)
tm = 5e-3;             % sweep time
bw = c/(2*rangeRes);   % bandwidth
sweepSlope = bw/tm;

waveform = phased.FMCWWaveform('SweepTime',tm,'SweepBandwidth',bw,'SampleRate',2e6);

tx = phased.Transmitter('PeakPower',0.1,'Gain',30);
rx = phased.ReceiverPreamp('Gain',20,'NoiseFigure',5);

% Free space propagation channel
channel = phased.FreeSpace('PropagationSpeed',c,'OperatingFrequency',fc);

% Radar targets
target1 = phased.RadarTarget('MeanRCS',10,'Model','Nonfluctuating');
target2 = phased.RadarTarget('MeanRCS',10,'Model','Nonfluctuating');

%% === Simulation Loop ===
numPulses = 64;
numSamples = waveform.SampleRate*tm;

radarEchoes = zeros(numSamples,numPulses);

for k = 1:numPulses
    advance(scenario);   % step the scenario

    % --- Get target positions relative to ego vehicle ---
    pos1 = targetVeh1.Position.';  % transpose to 3x1
    pos2 = targetVeh2.Position.';

    % Velocities (assuming constant for now)
    vel1 = [0;0;0];
    vel2 = [0;0;0];
    radarVel = [0;0;0];

    % --- Transmit waveform ---
    sig = waveform();
    sig = tx(sig);

    % --- Propagate to targets and back ---
    echo1 = channel(sig,[0;0;0], pos1, radarVel, vel1);
    echo1 = target1(echo1);

    echo2 = channel(sig,[0;0;0], pos2, radarVel, vel2);
    echo2 = target2(echo2);

    % --- Receive at radar ---
    radarEchoes(:,k) = rx(echo1 + echo2);
end

%% === Save Raw Radar Data ===
save('raw_radar_data.mat','radarEchoes','waveform','fc','tm','sweepSlope');

fprintf('Raw radar data saved to raw_radar_data.mat\n');

%% === Range-Doppler Processing ===
rdresp = phased.RangeDopplerResponse(...
    'RangeMethod','FFT', ...
    'DopplerOutput','Speed', ...
    'SweepSlope',sweepSlope, ...
    'SampleRate',waveform.SampleRate, ...
    'OperatingFrequency',fc);

figure;
plotResponse(rdresp, radarEchoes);
title('Range–Doppler Map (Forward Looking Radar)');
xlabel('Velocity (m/s)');
ylabel('Range (m)');
colorbar;

frameRate=155;             % rate at which radar collects CPI frames (hz)
noFrames=800;              % number of frames in the data file
prdcty=1/frameRate;        % time duration of CPI frame (s)
noADC=256;                 % number of range puses collected by the radar
noRx=4;                    % number of receive antennas
noChirps=32;               % number of puses (chirps) in a frame
PRF=1/200e-6;              % rate at which pulses are transmitted (hz)
f0=60e9;                   % center frequency of the radar (hz)
slp=144.858e12;            % chirt rate (Hz/S)
fs=12.5e6;                 % ADC sampling rate (Hz)
FFTRNGSIZE=2^ceil(log2(noADC));       % size for range dimension FFT
FFTDOPSIZE=2^ceil(log2(noChirps));    % size of doppler velocity FFT
c=physconst('lightspeed');            % speed of light

Radar_Filename0='\\g105-rtop\murmur\Measurements\20251016_chamber_tests\22sphere_0_Raw_0.bin';    % 

[radarFrame0,rAx]=getRadarReturns(Radar_Filename0,noADC,noChirps,noRx,noFrames,slp,fs,FFTRNGSIZE); % returns radar data cube and range axis

RD0=fftshift(fft(squeeze(radarFrame0(:,1,:,1)).*repmat(hamming(noChirps)',[noADC 1]),FFTDOPSIZE,2),2); % conver range profiles to range/doppler map
fAx=(-FFTDOPSIZE/2:FFTDOPSIZE/2-1)/FFTDOPSIZE*PRF; % doppler frequency axis
dopVAx=fAx*c/(2*f0);  % convert doppler frequency to radial velocity in m/s

subplot(2,1,1)        % plot data
plot(rAx,db20(squeeze(radarFrame0(:,1,:,1))),'b')
xlabel('range (m)')
title('power/range plot')

subplot(2,1,2)
imagesc(dopVAx,rAx,db20(RD0))
maxIm=max(db20(RD0(:)));
clim(maxIm+[-60 0]);
colormap('jet')
colorbar
xlabel('radial velocity (m/s)')
ylabel('range (m)')
title('Range/Doppler Velocity Plot')
axis('xy');

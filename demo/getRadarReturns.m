% [RD,fAx,rAx,dopVAx]=getRadarReturns(fName,noADC,noChirps,noRx,PRF,f0,slp,fs,FFTRNGSIZE,FFTDOPSIZE)
% INPUTS
% fName   - name of radar data file
% noADC     - number of ADC samples per chirp
% noChirps  - number of chirps per frame
% noRx      - number of antennas receive channels being recorded
% PRF       - pulse repeat frequency (hz)
% f0        - center frequency (hz)
% slp       - chirp frequency slope in Hz/Sec
% fs        - sampling rate (hz)
% FFTRNGSIZE - range fft size
% FFTDOPSIZE - dopler fft size
% OUTPUTS
% RD(MxN)   - range-doppler image cube
% rAx(M,1)  - range axis (m)
function [RD,rAx]=getRadarReturns(fName,noADC,noChirps,noRx,noFrames,slp,fs,FFTRNGSIZE)
c=physconst('lightspeed');
% FFTRNGSIZE=2^ceil(log2(noADC));
% FFTDOPSIZE=2^ceil(log2(noChirps));
% compute range and doppler axes
rAx=(0:FFTRNGSIZE-1)/FFTRNGSIZE*fs*(c/2)/slp;

frametot=noADC*noChirps*noRx;

fp=fopen(fName,'r');
aa=fread(fp,2*frametot*noFrames,'uint16=>single');
fclose(fp);
inds=find(aa>=2^15);
aa(inds)=aa(inds)-2^16;
cc=reshape(aa,2,[]);
ccs=1j*cc(1,:)+cc(2,:);
data=reshape(ccs,[noADC noRx noChirps noFrames]);
wt=hamming(noADC);
wts=repmat(wt,[1 noRx noChirps noFrames]); % not noChirps
RD=fft(data.*wts,FFTRNGSIZE,1);


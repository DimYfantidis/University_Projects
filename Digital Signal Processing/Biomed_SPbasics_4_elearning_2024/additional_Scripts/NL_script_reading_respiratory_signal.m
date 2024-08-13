clear all, close all;
load resp1;Fs=125;
subplot(2,1,1),plot([1:numel(resp)]*(1/Fs),resp),xlabel('sec'),title('respiration-signal')

[PKS,LOCS] = findpeaks(resp,Fs,'MinPeakHeight',0.5*range(resp))
subplot(2,1,2),plot([1:numel(resp)]*(1/Fs),resp,'k',LOCS,PKS,'ro'),xlabel('sec')

Breathing_Interval=mean(diff(LOCS));
display(strcat('breathing Interval:',num2str(Breathing_Interval),'sec')) 

display('alternatively, import the resp signal to sptool')

% launch sptool, import--> resp, sampling frequency--> Fs
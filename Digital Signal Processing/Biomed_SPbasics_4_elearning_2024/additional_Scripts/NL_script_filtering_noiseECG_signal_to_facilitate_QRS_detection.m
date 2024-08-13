% script demonstrating the use of band-pass filtering to suppress noise and
% before QRS-detection and formation of RR-interval timeseries   
% relates to problem 4.15

close all; clear all;
load ECG_noise, Fs=250

subplot(4,1,1),plot([1:numel(ecg)]*(1/Fs),ecg),xlabel('Time(s)'),grid,title('original ECG trace')

% design an a band-pass FIR 65-order filter with bandpass [4-25]Hz 
b=fir1(65,[4 25]/(Fs/2),'bandpass');
filtered_ecg=filtfilt(b,1,ecg); % apply the filter in causal (forward-direction) mode 

subplot(4,1,2),plot([1:numel(ecg)]*(1/Fs),filtered_ecg),grid,title('band-bass filtered signal')

[PKS,LOCs]=findpeaks(filtered_ecg,Fs,'MinPeakHeight',0.3*range(filtered_ecg))
subplot(4,1,3),plot([1:numel(ecg)]*(1/Fs),filtered_ecg,LOCs,PKS,'ro'),grid,%title(GRR
RR_intervals=diff(LOCs);
subplot(4,1,4),stem(LOCs(1:end-1),RR_intervals),grid,title('RR-intervals sequence'),ylabel('sec')
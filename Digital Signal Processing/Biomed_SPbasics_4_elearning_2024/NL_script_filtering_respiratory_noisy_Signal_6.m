% A simple script demonstrating the use of smooth-filtering with different options 
%   in order to perfrom denoising of a respiratory signal. Both the noisy and the original version are given.
%   We 'quantify' the performance of different options based on RelError:  ||reconstructed-original||/||original||  
% -it loosely relates to problem 4.24
%  you may experiment by altering the window (or span) parameter in the smooth command (currently set at 11) 

close all; clear all;load Resp_noise1, 
time=[1:numel(resp)]*(1/fs);

% original & noisy respiratory signal
subplot(3,2,1),plot(time,resp_noise1,'k',time,resp,'b'),xlabel('time(s)'),legend('noisy','original')
RelErrror=norm(resp_noise1-resp)/norm(resp); text(0.2,1.2,num2str(RelErrror))

filtered=smooth(resp_noise1,11);  %standard moving-average filter  
subplot(3,2,2),plot(time,filtered,'r',time,resp,'b'),xlabel('time(s)'),legend('restored','original'),title('moving average')
RelErrror=norm(filtered'-resp)/norm(resp); text(0.2,1.2,num2str(RelErrror))

filtered_loess=smooth(resp_noise1,11,'loess');% loess (quadratic fit with span of 11 samples) 
subplot(3,2,3),plot(time,filtered_loess,'r',time,resp,'b'),xlabel('time(s)'),legend('restored','original')
title('loess (quadratic fit)'),RelErrror=norm(filtered_loess'-resp)/norm(resp); text(0.2,1.2,num2str(RelErrror))

filtered_lowess=smooth(resp_noise1,11,'lowess'); % linear fit with span of 11 samples 
subplot(3,2,4),plot(time,filtered_lowess,'r',time,resp,'b'),xlabel('time(s)'),legend('restored','original')
title('lowess (linear fit)'),RelErrror=norm(filtered_lowess'-resp)/norm(resp); text(0.2,1.2,num2str(RelErrror))

filtered_sgolay=smooth(resp_noise1,11,'sgolay',1);% Savitzky-Golay of order1 and span of 11 samples
subplot(3,2,5),plot(time,filtered_sgolay,'r',time,resp,'b'),xlabel('time(s)'),legend('restored','original')
title('Savitzky-Golay:order1'),RelErrror=norm(filtered_sgolay'-resp)/norm(resp); text(0.2,1.2,num2str(RelErrror))

filtered_sgolay2=smooth(resp_noise1,11,'sgolay',3); % Savitzky-Golay of order3 and span of 11 samples
subplot(3,2,6),plot(time,filtered_sgolay2,'r',time,resp,'b'),xlabel('time(s)'),legend('restored','original')
title('Savitzky-Golay:order3'),RelErrror=norm(filtered_sgolay2'-resp)/norm(resp); text(0.2,1.2,num2str(RelErrror))


%% Mine
metrics = [
    mean(filtered), std(filtered), skewness(filtered);
    mean(filtered_loess), std(filtered_loess), skewness(filtered_loess);
    mean(filtered_lowess), std(filtered_lowess), skewness(filtered_lowess);
    mean(filtered_sgolay), std(filtered_sgolay), skewness(filtered_sgolay);
    mean(filtered_sgolay2), std(filtered_sgolay2), skewness(filtered_sgolay2);
]';

org_metrics = ones(3, 5);
org_metrics(:,1) = [mean(resp_noise1), std(resp_noise1), skewness(resp_noise1)]';
for i=2:5
    org_metrics(:,i) = org_metrics(:,1);
end
diff = abs(org_metrics - metrics)
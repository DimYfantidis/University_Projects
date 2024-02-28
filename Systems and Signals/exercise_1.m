%% Implementation Segment
% Read in the audio file
[x, Fs] = audioread('input.mp3');

% The audio file has two signals: the left and right channel
x_left = x(:,1)';
x_right = x(:,2)';

% Redundantly allocated memory is returned to the system
clear x;

% Create 300-point lowpass filter with 0.15Hz cutoff freequency
h_n = fir1(300, 0.15);

% Get the filtered signals
y_left = overlap_add(x_left, h_n);
y_right = overlap_add(x_right, h_n);

% Combine the two signals to create the final audio signal
y = [y_left', y_right'];

% Save the output data to a WAV file
audiowrite('output.wav', y, Fs);

% Redundantly allocated memory is returned to the system
clear y;


%% Plotting Segment
%  This code segment is used to plot the fourier transforms
%  of the initial and the filtered signal to indicate the 
%  frequences that have been removed.

% Magnitude of F{x}
X_left = abs(fft(x_left));
% Removes the mirrored part
X_left = X_left(1:(numel(X_left)/2));

% Magnitude of F{y}
Y_left = abs(fft(y_left));
% Removes the mirrored part
Y_left = Y_left(1:(numel(Y_left)/2));

% Filter response
fvtool(h_n);

signalAnalyzer;

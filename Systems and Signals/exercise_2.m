% Read in the audio file
[x, Fs] = audioread('input.mp3');

% The audio file has two signals: the left and right channel
x_left = x(:,1);
x_right = x(:,2);

% Create 300-point lowpass filter with 0.15Hz cutoff freequency
h_n = fir1(300, 0.15/(Fs/2));

% Get the filtered signals
y_left = overlap_save(x_left, h_n);
y_right = overlap_save(x_right, h_n);

% Combine the two signals to create the audio signal
y = [y_left', y_right'];

clear x x_right x_left y_left y_right;

figure(1);
plot(abs(fft(y)), 'b');

% Save the output data to a WAV file
audiowrite('output2.wav', y, Fs);

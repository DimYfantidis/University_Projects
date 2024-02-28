function y=overlap_add(x, h)

% Filter's samples
M = numel(h);

% The length of each component y_k
N = 8 * 2^ceil(log2(M));

% Selection of N corresponds to a power of 2
% Multiplied by 8 so that the zeros at the end are proportionally
% less in number than the L first elements

% Block length
L = N - (M-1);

% DFT of the filter's response, calculated once
H = fft(h, N);

% Used for iterating through the components y_k
offset = 0;

% Initializes the the length of the resulting signal and sets scalars to 0
y = zeros(1, length(x) + M-1);

% Iterates through the components y_k
while offset + L <= length(x)
    % The components are not treated as different matrices.
    % Instead, all calculations and additions happen on the 
    % resulting matrix using proper MATLAB expressions
    y(offset+(1:N)) = ... 
        y(offset+(1:N)) + ifft(fft(x(offset+(1:L)), N) .* H);

    % Proceeds to the next component
    offset = offset + L;
end

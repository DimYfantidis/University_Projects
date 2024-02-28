function y=overlap_save(x, h)

len_x = numel(x) / 2;

M = numel(h);
overlap = M - 1;
N = 8 * overlap;
L = N - overlap;

H = fft(h, N);
pos = 0;

while pos + N <= len_x
    yt = ifft(transpose(fft(x(pos+(1:N)))) .* H);
    y(pos+(1:L)) = yt(M : N);
    pos = pos + L;
end

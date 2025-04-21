load short_modem_rx.mat
load sync_noise.mat  
figure;
plot(y_r);
title('Full Received Signal y\_r');
xlabel('Sample Index');
ylabel('Amplitude');
grid on;
start_idx = find_start_of_signal(y_r, x_sync);
data_start = 40000;  
y_t = y_r(data_start:end);
%% Parameters modem_tx.m
fc = 1000;        
Fs = 8192;        
SymbolPeriod = 100;  
%% Step 1: Coherent demodulation (multiply by cosine)
t = (0:length(y_t)-1)' / Fs;
y_demod = y_t .* cos(2*pi*fc*t);
%% Step 2: FIR lowpass filter
lpFilt = designfilt('lowpassfir', ...
    'PassbandFrequency', 200, ...
    'StopbandFrequency', 500, ...
    'SampleRate', Fs);
m_baseband = filter(lpFilt, y_demod);
%% Step 3: Normalize
m_baseband = m_baseband / max(abs(m_baseband));
%% Step 4: Plot to inspect signal
figure;
plot(data_start + (0:length(m_baseband)-1), m_baseband);
title('Demodulated Baseband Signal m(t)');
xlabel('Sample Index (relative to y\_r)');
ylabel('Amplitude');
%% Bit extraction
SymbolPeriod = 100;
bits_expected = 5 * 8;  % "Hello" = 5 characters

x_d = zeros(1, bits_expected);
offset = -10;  

for k = 1:bits_expected
    idx = round((k - 0.5) * SymbolPeriod) + offset;
    if idx > 0 && idx <= length(m_baseband)
        x_d(k) = m_baseband(idx) > 0;  % 1 if sample > 0, else 0
    end
end

%% Decode bits to string
decoded = BitsToString(x_d);
disp("Decoded Message: " + decoded);

function start_idx = find_start_of_signal(x, sync)
% Uses a cross correlation-based approach to find
% the start of the signal. It assumes that the
% signal in x has the sync signal at its very beginning.

signal_rms = rms(x);
coarse_idx_list = find(abs(x) > signal_rms, 1);

if isempty(coarse_idx_list)
    error('No point in x exceeds the RMS threshold. Check signal.');
end

coarse_idx = coarse_idx_list - length(sync);

% Safety: clip to valid range
coarse_idx = max(coarse_idx, 1);
upper_bound = min(coarse_idx + length(sync)*3, length(x));

% Run correlation in safe range
[Ryx, lags] = xcorr(x(1:upper_bound), sync);
[~, ii] = max(abs(Ryx));
start_idx = lags(ii) + 1;

% Final check
start_idx = max(start_idx, 1);
end
clear; clc;
load short_modem_rx.mat
load sync_noise.mat  
figure;
plot(y_r);
title('Full Received Signal y\_r');
xlabel('Sample Index');
ylabel('Amplitude');
grid on;
start_idx = find_start_of_signal(y_r, x_sync);
data_start = start_idx + length(x_sync) ;  % skip sync and add a small buffer
y_t = y_r(data_start:end);
%% Parameters modem_tx.m
fc = 1000;        
Fs = 8192;        
SymbolPeriod = 100;  
%% Step 1: Coherent demodulation (multiply by cosine)
t = (0:length(y_t)-1)' / Fs;
y_demod = y_t .* cos(2*pi*fc*t);
%% Step 2: Apply truncated ideal LPF (sinc-based FIR)
k = -20:20;                 % 51-point filter
Omegac = pi/3;              % cutoff
h = (Omegac/pi) * sinc((Omegac/pi)*k);  % impulse response

m_baseband = conv(y_demod, h, 'same');  % apply filter
m_baseband = m_baseband / max(abs(m_baseband));  % normalize
%% Step 3: Plot to inspect signal
figure;
plot(data_start + (0:length(m_baseband)-1), m_baseband);
title('Demodulated Baseband Signal m(t)');
xlabel('Sample Index (relative to y\_r)');
ylabel('Amplitude');
%% Bit extraction
bits_expected = 5 * 8;  % "Hello" = 5 characters
ref = StringToBits('Hello');
best_msg = '';
best_score = 0;

for offset = -15:15 % try 31 different timing offsets
    x_d = zeros(1, bits_expected); % at each offset, samples 40 bits from the signal
    for k = 1:bits_expected
        idx = round((k - 0.5) * SymbolPeriod) + offset;
        if idx > 0 && idx <= length(m_baseband)
            x_d(k) = m_baseband(idx) > 0;
        end
    end
    score = sum(x_d == ref); % compares those bits to the binary form of "Hello"
    if score > best_score
        best_score = score;
        best_msg = BitsToString(x_d);
        best_offset = offset; % keeps the one with the most matching bits
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

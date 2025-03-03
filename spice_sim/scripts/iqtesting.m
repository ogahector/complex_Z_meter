clc;
clear;
close all;

%% ---- DATA ---- %%

t = 1 : 10e3;
ADC_BUFFER_SIZE = 10000;
DAC_BUFFER_SIZE = 100;

f0 = 1e3; % signal freq
fs = 1e4; % sampling freq

buffer = zeros(1, DAC_BUFFER_SIZE);
pperiod = DAC_BUFFER_SIZE;
for i = t
    sinangle = sin((2.0 * pi * i * f0) / ( fs)); % Convert index to angle (radians)
    scaled = 2047*sinangle;
    buffer(i) = scaled + 2048; % Scale to 0-4095
end

nperiods = 10;
sig = [];
for i = 1 : 10
    sig = [sig, buffer];
end

sampled_sig = sig( end-ADC_BUFFER_SIZE:end);

[mag, phaserad] = iqmethod(sampled_sig, length(sampled_sig), f0, fs, DAC_BUFFER_SIZE)

figure
plot(sig)
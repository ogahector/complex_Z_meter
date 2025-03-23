clc;
clear;

%% Resistor

Zstd = 3260;

dataOM




function Zdut = fully_calibrated(Zstd, Zstdm, Zm, Zsm, Zom)
    Zdut = Zstd * (Zom - Zstdm) * (Zm - Zsm) / ( (Zstdm - Zsm) * (Zom - Zm) );
end

function [freqs, Z] = data2usable(data)
    freqs = []
    Z = []
    for i = 1 : length(data)
        if mod(i, 3) == 0
            freqs = [freqs data(i)]
        end
end
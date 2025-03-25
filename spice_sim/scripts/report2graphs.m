clc;
clear;
close all;

fmin = 100;
fmax = 100e3;
nfreqs = 60;

freqs = logspace(log10(fmin), log10(fmax), nfreqs);

%% FOR RREF = 1K
Rref = 1e3;


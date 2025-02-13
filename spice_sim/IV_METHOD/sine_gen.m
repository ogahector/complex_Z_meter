%% SINE WAVE GENERATOR
close all

npoints = 100;
freq = 1e3;
file = fopen('sine_wave_points.txt', 'w');

t = linspace(0, 1/freq, 100);
theta = 2*pi*t*freq;
vals = sin(theta);

data = [t; vals];

figure
plot(theta, vals)

fprintf(file, "%.6f %.6f\n", data);
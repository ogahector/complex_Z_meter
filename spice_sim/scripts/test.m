clc;
clear;
close all;

file = fopen('data/logspace_test.txt');

data = fscanf(file, "%f")

figure
semilogy(data)
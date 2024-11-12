function [ output_args ] = dummy_fft( input_args )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
close all;
    X1 = randn(1000,1);
    X2 = randn(1000,1);

    h(1:16)  = 5*rand(16,1);
    temp = histogram(X1, 16);
    h(17:32) = temp.Values/3;
    h(33:48) = 5*rand(16,1);
    temp = histogram(X2, 16);
    h(49:64) = temp.Values/5;
    bar(1:64, h);
    xlabel('bin number');
    ylabel('db');
end


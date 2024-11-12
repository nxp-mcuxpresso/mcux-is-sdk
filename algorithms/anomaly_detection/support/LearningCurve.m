function [ output_args ] = LearningCurve( input_args )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    close all;
    x = 1:200;
    for a=0.15:0.15
    for i=x
        y(i)=f(i, a);
    end
        plot(x,y); hold on;
    end
    xlabel('PP = Sample Size');
    ylabel('% of False Negatives = FN/PP');
    title('Learning vs Data Set Size')
    %legend('parameter set 1', 'parameter set 2', ...
    %       'parameter set 3', 'parameter set 4');
end

function [y] = sigmoid(z)
    y = 1 / (1+exp(-z));
end
function [y] = f(x, a)
    y = .7 - 0.6*sigmoid(a*x-10);
    y=y*100;
end
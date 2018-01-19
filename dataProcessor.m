% This function is used to pull the data obtained on the microcomputer and process this and put into appropriate assignments
% Zachary Lippay

clear
clc
close all

data = '/Users/zlipp3032/Documents/MastersThesisUAS/SOLO/Code/ComTests/Control/OldVersions/LogDataScheme/junk/2018_01_19__00_15_03_log.csv';
[Data,relTime,scaleTime] = dataProcessor_v3(data);

xlb = 'Time (s)';

figure(1)
subplot(4,1,1)
plot(relTime,Data(:,1))
xlabel(xlb,'Interpreter','Latex')
ylabel('Chanel 1','Interpreter','Latex')
grid('on')
subplot(4,1,2)
plot(relTime,Data(:,2))
xlabel(xlb,'Interpreter','Latex')
ylabel('Chanel 2','Interpreter','Latex')
grid('on')
subplot(4,1,3)
plot(relTime,Data(:,3))
xlabel(xlb,'Interpreter','Latex')
ylabel('Chanel 3','Interpreter','Latex')
grid('on')
subplot(4,1,4)
plot(relTime,Data(:,4))
xlabel(xlb,'Interpreter','Latex')
ylabel('Chanel 4','Interpreter','Latex')
grid('on')



function [Data,relTime,scaleTime] = dataProcessor_v3(data)
% data = '/Users/zlipp3032/Documents/MastersThesisUAS/Code/ComTests/Control/junk/2018_01_12__23_10_12_log.csv';

% Import data from specified path
A = importdata(data);

% Define the points
relTime = A.data(:,1); %(s) Time stamp since the code was initialized
% ID = A.data(:,1);
Data(:,1) = A.data(:,3);
Data(:,2) = A.data(:,4);
Data(:,3) = A.data(:,5);
Data(:,4) = A.data(:,6);
%vxData = A.data(:,6);
%vyData = A.data(:,7);
%vzData = A.data(:,8);
%uData(:,1) = -A.data(:,12);
%uData(:,2) = -A.data(:,13);
%uData(:,3) = -A.data(:,14);
scaleTime = max(relTime) - min(relTime);
% TIMESCALER = scaleTime/max(t);
% tscaler = t*TIMESCALER;
% tscale = zeros(length(t));
% tscale(1) = min(relTime);
% for i = 1:length(t)
%     tscale(i) = min(relTime)+t(i);
% end

end

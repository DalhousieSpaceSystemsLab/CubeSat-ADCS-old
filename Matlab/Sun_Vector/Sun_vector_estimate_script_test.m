close all 
clear all
%Test Sun_vector_estimate.m function in a short Matlab script
%(instead of as a function in Simulink)
%using 11 different sample data in 'Sun_Test_Data.mat' which contains
% H = 18 row concatenation of 1x3 normal vectors of Sun sensors expressed
%     in BF creating an 18x3 matrix
% y = 18 Sun column raw sensor intensity measurements (values between 0 and 1)
%     each column is a different set of sample data (with 11 samples this creates
%     an 18x11 matrix)
% s_BF_est = estimated Sun vector in BF (with 11 samples this creates
%     a 11x3 matrix) which can be used to check the results

% This software is for Dal CubeSat project internal use only.
% Dr. Robert Bauer shall not be liable for any direct, indirect, 
% consequential, or other damages suffered by anyone resulting from this 
% work or the use of the research results/data of this work.

load Sun_Test_Data
Data = load('Sun_Test_Data.mat');
DataField = fieldnames(Data);
dlmwrite('H.txt', Data.(DataField{1}), 'delimiter', ' ', 'precision', 15);
dlmwrite('s_BF_est.txt', Data.(DataField{2}), 'delimiter', ' ', 'precision', 15);
dlmwrite('y.txt', Data.(DataField{3}), 'delimiter', ' ', 'precision', 15);

s_hat_BF_calculated=[];
for i=1:11
    s_hat_BF_calculated(i,:) = Sun_vector_estimate(H,y(:,i));
end
figure(1)
subplot(3,1,1)
plot(s_BF_est(:,1),'k-','LineWidth',1)
hold on
plot(s_hat_BF_calculated(:,1),'r--','LineWidth',1)
ylabel('First component','FontSize',12)
title('Sun Vector Component Check','FontSize',12)
subplot(3,1,2)
plot(s_BF_est(:,2),'k-','LineWidth',1)
hold on
plot(s_hat_BF_calculated(:,2),'r--','LineWidth',1)
ylabel('Second component','FontSize',12)
subplot(3,1,3)
plot(s_BF_est(:,3),'k-','LineWidth',1)
hold on
plot(s_hat_BF_calculated(:,3),'r--','LineWidth',1)
ylabel('Third component','FontSize',12)
xlabel('Sample Number','FontSize',12)
dlmwrite('s_hat_BF_matlab.csv', s_hat_BF_calculated, 'delimiter', ',', 'precision', 15);


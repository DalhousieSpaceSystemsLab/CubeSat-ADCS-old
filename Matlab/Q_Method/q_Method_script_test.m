close all 
clear all
%Test q_Method.m function in a short Matlab script
%(instead of as a function in Simulink)
%using 11 different sample data in 'q_Method_Test_Data.mat' which contains
% b1 = components of Sun column vectors expressed in Body-Fixed frame
%      3x11 matrix
% b2 = components of Magnetic Field column vectors expressed in Body-Fixed frame
%      3x11 matrix (from magnetometer A)
% b3 = components of Magnetic Field column vectors expressed in Body-Fixed frame
%      3x11 matrix (from magnetometer B)
% r1 = components of Sun column vectors expressed in ECI frame
%      3x11 matrix (from theory)
% r2 = components of Magnetic Field column vectors expressed in ECI frame
%      3x11 matrix (from IGRF)
% q_est_check = quaternion estimate 4x11 matrix which can be used to check the results
load q_Method_Test_Data

q_est=[];
for i=1:11
    q_est(:,i) = q_Method(b1(:,i),b2(:,i),b3(:,i),r1(:,i),r2(:,i));
    %note: multiplying q_est by -1 gives the SAME rotation
    %so to visually compare q_est against q_est_check, need to 
    %ensure signs are the same
    if sign(q_est(1,i)) ~= sign(q_est_check(1,i))
        %signs don't agree so multiply q_est by -1
        q_est(:,i) = q_est(:,i)*-1;
    end
end

figure(1)
subplot(4,1,1)
plot(q_est_check(1,:),'k-','LineWidth',1)
hold on
plot(q_est(1,:),'r--','LineWidth',1)
ylabel('First component','FontSize',12)
title('Quaternion Estimate Check','FontSize',12)
axis([1,11,-1,1])

subplot(4,1,2)
plot(q_est_check(2,:),'k-','LineWidth',1)
hold on
plot(q_est(2,:),'r--','LineWidth',1)
ylabel('Second component','FontSize',12)
axis([1,11,-1,1])

subplot(4,1,3)
plot(q_est_check(3,:),'k-','LineWidth',1)
hold on
plot(q_est(3,:),'r--','LineWidth',1)
ylabel('Third component','FontSize',12)
axis([1,11,-1,1])

subplot(4,1,4)
plot(q_est_check(4,:),'k-','LineWidth',1)
hold on
plot(q_est(4,:),'r--','LineWidth',1)
ylabel('Fourth component','FontSize',12)
xlabel('Sample Number','FontSize',12)
axis([1,11,-1,1])

% Print Results
Data = load('q_Method_Test_Data.mat');
DataField = fieldnames(Data);
dlmwrite('q_est_matlab.txt', q_est, 'delimiter', ' ', 'precision', 15);
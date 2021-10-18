% close all,clear all;
%% Format CSV file to mat

yolo = readmatrix('Yolo_raw.csv');
kf = readmatrix('KF.csv');
aruco = readmatrix('Aruco&Depth_raw.csv');
groundtruth = readmatrix('Groundtruth.csv');

figure('Name','Position Estimation','Position',[0 0 850 850]);
tiledlayout(3,1);

nexttile
plot(yolo(:,1),yolo(:,2),'Color','k','LineWidth',1);
title('Position Estimation','FontSize',14);
ylabel('X Position (m)','FontSize',14);
hold on
plot(groundtruth(:,1),groundtruth(:,2),'Color','r','LineWidth',1);
plot(kf(:,1),kf(:,2),'Color',[0 0.4470 0.7410],'LineWidth',1);
plot(aruco(:,1),aruco(:,2),'Color','g','LineWidth',1);
set(gca,'FontSize',14);

nexttile
plot(yolo(:,1),yolo(:,3),'Color','k','LineWidth',1);
ylabel('Y Position (m)','FontSize',14);
hold on
plot(groundtruth(:,1),groundtruth(:,3),'Color','r','LineWidth',1);
plot(kf(:,1),kf(:,3),'Color',[0 0.4470 0.7410],'LineWidth',1);
plot(aruco(:,1),aruco(:,3),'Color','g','LineWidth',1);
set(gca,'FontSize',14);

nexttile
plot(yolo(:,1),yolo(:,4),'Color','k','LineWidth',1);
ylabel('Z Position (m)','FontSize',14);
hold on
plot(groundtruth(:,1),groundtruth(:,4),'Color','r','LineWidth',1);
plot(kf(:,1),kf(:,4),'Color',[0 0.4470 0.7410],'LineWidth',1);
plot(aruco(:,1),aruco(:,4),'Color','g','LineWidth',1);
set(gca,'FontSize',14);

legend('Yolo','GT','KF','Aruco','FontSize',10);
close all,clear all;
%% Format CSV file to mat

FSM = readmatrix('FSM.csv');
PlotParam.Color = [1 0 0;
                   0 0 1;
                   0 1 0;
                   1 0.5 0;
                   0 0.5 1];
FigureParam.Count = 1;
FigureParam.StartPos = [25 25];
fontsize = 15;

figure ('Name','Position Estimation','Position',[0 0 850 850]);
    hold on;ax=gca;ax.FontSize=fontsize;
    set(gcf, 'position', [ FigureParam.StartPos(1)+(FigureParam.Count-1)*25 FigureParam.StartPos(2)+(FigureParam.Count-1)*25 1600 900]);
    subplot(311);
    plot(FSM(:,1),FSM(:,2),...
            'color',PlotParam.Color(1,:),'LineWidth',1.1,'MarkerIndices',1:75:1501,'MarkerSize',10);grid on;;hold on;ax=gca;ax.FontSize=fontsize;
    plot(FSM(:,1),FSM(:,5),...
            'color',PlotParam.Color(2,:),'LineWidth',1.1,'MarkerIndices',1:75:1501,'MarkerSize',10);grid on;;...
            xlabel('time(s)','FontSize',fontsize);
    plot(FSM(:,1),FSM(:,11),...
            'k--','LineWidth',1.1,'MarkerIndices',1:75:1501,'MarkerSize',10);grid on;...
            xlabel('time(s)','FontSize',fontsize);ylabel('X (m)');legend('Ground Truth','Pose Estimation','Setpoint','FontSize',fontsize);hold on;ax=gca;ax.FontSize=fontsize;
    subplot(312);
    plot(FSM(:,1),FSM(:,3),...
            'color',PlotParam.Color(1,:),'LineWidth',1.1,'MarkerIndices',1:75:1501,'MarkerSize',10);grid on;;hold on;ax=gca;ax.FontSize=fontsize;
    plot(FSM(:,1),FSM(:,6),...
            'color',PlotParam.Color(2,:),'LineWidth',1.1,'MarkerIndices',1:75:1501,'MarkerSize',10);grid on;;...
            xlabel('time(s)','FontSize',fontsize);
    plot(FSM(:,1),FSM(:,12),...
            'k--','LineWidth',1.1,'MarkerIndices',1:75:1501,'MarkerSize',10);grid on;...
            xlabel('time(s)','FontSize',fontsize);ylabel('Y (m)');legend('Ground Truth','Pose Estimation','Setpoint','FontSize',fontsize);hold on;ax=gca;ax.FontSize=fontsize;
    subplot(313);
    plot(FSM(:,1),FSM(:,4),...
            'color',PlotParam.Color(1,:),'LineWidth',1.1,'MarkerIndices',1:75:1501,'MarkerSize',10);grid on;;hold on;ax=gca;ax.FontSize=fontsize;
    plot(FSM(:,1),FSM(:,7),...
            'color',PlotParam.Color(2,:),'LineWidth',1.1,'MarkerIndices',1:75:1501,'MarkerSize',10);grid on;;...
            xlabel('time(s)','FontSize',fontsize);
    plot(FSM(:,1),FSM(:,13),...
            'k--','LineWidth',1.1,'MarkerIndices',1:75:1501,'MarkerSize',10);grid on;...
            xlabel('time(s)','FontSize',fontsize);ylabel('Z (m)');legend('Ground Truth','Pose Estimation','Setpoint','FontSize',fontsize);hold on;ax=gca;ax.FontSize=fontsize;


% -----------------------------------------------------------------------------------------------    
    
% figure('Name','Position Estimation','Position',[0 0 850 850]);
% tiledlayout(3,1);
% 
% nexttile
% ylabel('X Position (m)','FontSize',14);
% plot(FSM(:,1),FSM(:,2),'--','Color','k','LineWidth',1);
% title('Position Estimation','FontSize',14);
% hold on
% plot(groundtruth(:,1),groundtruth(:,2),'Color','r','LineWidth',1);
% plot(kf(:,1),kf(:,2),'Color',[0 0.4470 0.7410],'LineWidth',1);
% plot(aruco(:,1),aruco(:,2),'Color','g','LineWidth',1);
% set(gca,'FontSize',14);
% 
% nexttile
% plot(yolo(:,1),yolo(:,3),'Color','k','LineWidth',1);
% ylabel('Y Position (m)','FontSize',14);
% hold on
% plot(groundtruth(:,1),groundtruth(:,3),'Color','r','LineWidth',1);
% plot(kf(:,1),kf(:,3),'Color',[0 0.4470 0.7410],'LineWidth',1);
% plot(aruco(:,1),aruco(:,3),'Color','g','LineWidth',1);
% set(gca,'FontSize',14);
% 
% nexttile
% plot(yolo(:,1),yolo(:,4),'Color','k','LineWidth',1);
% ylabel('Z Position (m)','FontSize',14);
% hold on
% plot(groundtruth(:,1),groundtruth(:,4),'Color','r','LineWidth',1);
% plot(kf(:,1),kf(:,4),'Color',[0 0.4470 0.7410],'LineWidth',1);
% plot(aruco(:,1),aruco(:,4),'Color','g','LineWidth',1);
% set(gca,'FontSize',14);
% 
% legend('Yolo','GT','KF','Aruco','FontSize',10);
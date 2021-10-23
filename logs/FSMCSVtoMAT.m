close all,clear all;
%% Format CSV file to mat

FSM = readmatrix('FSM1.csv');
Time = FSM(:,1);
Time = Time- Time(1);
UAV_groundtruth = [FSM(:,2),FSM(:,3),FSM(:,4)];
Estimation = [FSM(:,5),FSM(:,6),FSM(:,7)];
UGV_pose = [FSM(:,8),FSM(:,9),FSM(:,10)];
Setpoint = [FSM(:,11),FSM(:,12),FSM(:,13)];
Velocity = [FSM(:,14),FSM(:,15),FSM(:,16)];
X_start = find(Setpoint(:,1),1);
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
        plot(Time((X_start:end),1),UAV_groundtruth((X_start:end),1),...
                'color',PlotParam.Color(1,:),'LineWidth',1.1,'MarkerSize',10);grid on;hold on;ax=gca;ax.FontSize=fontsize;
        plot(Time((X_start:end),1),Estimation((X_start:end),1),...
                'color',PlotParam.Color(2,:),'LineWidth',1.1,'MarkerSize',10);grid on;...
                xlabel('time(s)','FontSize',fontsize);
        plot(Time((X_start:end),1),UGV_pose((X_start:end),1),...
                'r--','LineWidth',1.1,'MarkerSize',10);grid on;...
                xlabel('time(s)','FontSize',fontsize);ylabel('X (m)');
        plot(Time((X_start:end),1),Setpoint((X_start:end),1),...
                'k--','LineWidth',1.1,'MarkerSize',10);grid on;...
                xlabel('time(s)','FontSize',fontsize);ylabel('X (m)');legend('Ground Truth','Pose Estimation','UGV','Setpoint','FontSize',fontsize);hold on;ax=gca;ax.FontSize=fontsize;
        subplot(312);
        plot(Time((X_start:end),1),UAV_groundtruth((X_start:end),2),...
                'color',PlotParam.Color(1,:),'LineWidth',1.1,'MarkerSize',10);grid on;hold on;ax=gca;ax.FontSize=fontsize;
        plot(Time((X_start:end),1),Estimation((X_start:end),2),...
                'color',PlotParam.Color(2,:),'LineWidth',1.1,'MarkerSize',10);grid on;...
                xlabel('time(s)','FontSize',fontsize);
        plot(Time((X_start:end),1),UGV_pose((X_start:end),2),...
                'r--','LineWidth',1.1,'MarkerSize',10);grid on;...
                xlabel('time(s)','FontSize',fontsize);ylabel('X (m)');
        plot(Time((X_start:end),1),Setpoint((X_start:end),2),...
                'k--','LineWidth',1.1,'MarkerSize',10);grid on;...
                xlabel('time(s)','FontSize',fontsize);ylabel('Y (m)');legend('Ground Truth','Pose Estimation','UGV','Setpoint','FontSize',fontsize);hold on;ax=gca;ax.FontSize=fontsize;
        subplot(313);
        plot(Time((X_start:end),1),UAV_groundtruth((X_start:end),3),...
                'color',PlotParam.Color(1,:),'LineWidth',1.1,'MarkerSize',10);grid on;hold on;ax=gca;ax.FontSize=fontsize;
        plot(Time((X_start:end),1),Estimation((X_start:end),3),...
                'color',PlotParam.Color(2,:),'LineWidth',1.1,'MarkerSize',10);grid on;...
                xlabel('time(s)','FontSize',fontsize);
        plot(Time((X_start:end),1),UGV_pose((X_start:end),3),...
                'r--','LineWidth',1.1,'MarkerSize',10);grid on;...
                xlabel('time(s)','FontSize',fontsize);ylabel('X (m)');
        plot(Time((X_start:end),1),Setpoint((X_start:end),3),...
                'k--','LineWidth',1.1,'MarkerSize',10);grid on;...
                xlabel('time(s)','FontSize',fontsize);ylabel('Z (m)');legend('Ground Truth','Pose Estimation','UGV','Setpoint','FontSize',fontsize);hold on;ax=gca;ax.FontSize=fontsize;


figure ('Name','Velocity','Position',[10 10 860 860]);
        hold on;ax=gca;ax.FontSize=fontsize;
        set(gcf, 'position', [ FigureParam.StartPos(1)+(FigureParam.Count-1)*25 FigureParam.StartPos(2)+(FigureParam.Count-1)*25 1600 900]);
        subplot(311);
        plot(Time((X_start:end),1),Velocity((X_start:end),1),...
                'k--','LineWidth',1.1,'MarkerSize',10);grid on;...
                xlabel('time(s)','FontSize',fontsize);ylabel('X (m/s)');legend('Velocity','FontSize',fontsize);hold on;ax=gca;ax.FontSize=fontsize;
        subplot(312);
        plot(Time((X_start:end),1),Velocity((X_start:end),2),...
                    'k--','LineWidth',1.1,'MarkerSize',10);grid on;...
                    xlabel('time(s)','FontSize',fontsize);ylabel('Y (m/s)');legend('Velocity','FontSize',fontsize);hold on;ax=gca;ax.FontSize=fontsize;
        subplot(313);
        plot(Time((X_start:end),1),Velocity((X_start:end),3),...
                    'k--','LineWidth',1.1,'MarkerSize',10);grid on;...
                    xlabel('time(s)','FontSize',fontsize);ylabel('Z (m/s)');legend('Velocity','FontSize',fontsize);hold on;ax=gca;ax.FontSize=fontsize;
        
        
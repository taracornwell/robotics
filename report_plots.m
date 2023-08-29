%% Make plots for final report

%% Task 1
% Positions and vels
figure;
subplot(1,2,1)
hold on;
plot(out.tout,out.x(:,1),'LineWidth',1.5)
plot(out.tout,out.x(:,2),'LineWidth',1.5)
plot(out.tout,out.x(:,3),'LineWidth',1.5)
xlabel('time (s)')
ylabel('position (m)')
legend({'x','y','z'},'Location','northwest')
set(gca,'FontName','Arial','FontSize',14)

subplot(1,2,2)
hold on;
plot(out.tout,out.x(:,7),'LineWidth',1.5)
plot(out.tout,out.x(:,8),'LineWidth',1.5)
plot(out.tout,out.x(:,9),'LineWidth',1.5)
plot([0 10],[-0.5 -0.5],'--k')
xlabel('time (s)')
ylabel('velocity (m/s)')
set(gca,'FontName','Arial','FontSize',14)

sgtitle('Running','FontName','Arial','FontSize',16,'Fontweight','bold')

%% Rotating
figure;
subplot(1,2,1)
hold on;
plot(out.tout,out.x(:,4),'LineWidth',1.5)
plot(out.tout,out.x(:,5),'LineWidth',1.5)
plot(out.tout,out.x(:,6),'LineWidth',1.5)
xlabel('time (s)')
ylabel('orientation (rad)')
legend({'roll','pitch','yaw'},'Location','northwest')
set(gca,'FontName','Arial','FontSize',14)

subplot(1,2,2)
hold on;
plot(out.tout,out.x(:,10),'LineWidth',1.5)
plot(out.tout,out.x(:,11),'LineWidth',1.5)
plot(out.tout,out.x(:,12),'LineWidth',1.5)
plot([0 10],[0.5 0.5],'--k')
xlabel('time (s)')
ylabel('angular velocity (rad/s)')
set(gca,'FontName','Arial','FontSize',14)

sgtitle('Rotating left','FontName','Arial','FontSize',16,'Fontweight','bold')

%% Joint angles, velocities, torques
figure;
subplot(3,3,1)
hold on;
title('Hip')
plot(out.tout,out.q_FL(:,1),'LineWidth',1)
plot(out.tout,out.q_FR(:,1),'LineWidth',1)
plot(out.tout,out.q_RL(:,1),'LineWidth',1)
plot(out.tout,out.q_RR(:,1),'LineWidth',1)
ylim([-3 2.5])
ylabel('joint angle (rad)')
set(gca,'FontName','Arial','FontSize',14)

subplot(3,3,2)
hold on;
title('Thigh')
plot(out.tout,out.q_FL(:,2),'LineWidth',1)
plot(out.tout,out.q_FR(:,2),'LineWidth',1)
plot(out.tout,out.q_RL(:,2),'LineWidth',1)
plot(out.tout,out.q_RR(:,2),'LineWidth',1)
ylim([-3 2.5])
set(gca,'FontName','Arial','FontSize',14)

subplot(3,3,3)
hold on;
plot(out.tout,out.q_FL(:,3),'LineWidth',1)
plot(out.tout,out.q_FR(:,3),'LineWidth',1)
plot(out.tout,out.q_RL(:,3),'LineWidth',1)
plot(out.tout,out.q_RR(:,3),'LineWidth',1)
ylim([-3 2.5])
legend({'FL','FR','RL','RR'},'Location','northwest')
title('Calf')
set(gca,'FontName','Arial','FontSize',14)

subplot(3,3,4)
hold on;
plot(out.tout,out.dq_FL(:,1),'LineWidth',1)
plot(out.tout,out.dq_FR(:,1),'LineWidth',1)
plot(out.tout,out.dq_RL(:,1),'LineWidth',1)
plot(out.tout,out.dq_RR(:,1),'LineWidth',1)
ylim([-40 60])
ylabel('joint velocity (rad/s)')
set(gca,'FontName','Arial','FontSize',14)

subplot(3,3,5)
hold on;
plot(out.tout,out.dq_FL(:,2),'LineWidth',1)
plot(out.tout,out.dq_FR(:,2),'LineWidth',1)
plot(out.tout,out.dq_RL(:,2),'LineWidth',1)
plot(out.tout,out.dq_RR(:,2),'LineWidth',1)
ylim([-40 60])
set(gca,'FontName','Arial','FontSize',14)

subplot(3,3,6)
hold on;
plot(out.tout,out.dq_FL(:,3),'LineWidth',1)
plot(out.tout,out.dq_FR(:,3),'LineWidth',1)
plot(out.tout,out.dq_RL(:,3),'LineWidth',1)
plot(out.tout,out.dq_RR(:,3),'LineWidth',1)
ylim([-40 60])
set(gca,'FontName','Arial','FontSize',14)

subplot(3,3,7)
hold on;
plot(out.tout,out.sensed_torques(:,1),'LineWidth',1)
plot(out.tout,out.sensed_torques(:,4),'LineWidth',1)
plot(out.tout,out.sensed_torques(:,7),'LineWidth',1)
plot(out.tout,out.sensed_torques(:,10),'LineWidth',1)
ylim([-35 35])
yticks([-35 0 35])
ylabel('joint torque (Nm)')
xlabel('time (s)')
set(gca,'FontName','Arial','FontSize',14)

subplot(3,3,8)
hold on;
plot(out.tout,out.sensed_torques(:,2),'LineWidth',1)
plot(out.tout,out.sensed_torques(:,5),'LineWidth',1)
plot(out.tout,out.sensed_torques(:,8),'LineWidth',1)
plot(out.tout,out.sensed_torques(:,11),'LineWidth',1)
ylim([-35 35])
yticks([-35 0 35])
xlabel('time (s)')
set(gca,'FontName','Arial','FontSize',14)

subplot(3,3,9)
hold on;
plot(out.tout,out.sensed_torques(:,3),'LineWidth',1)
plot(out.tout,out.sensed_torques(:,6),'LineWidth',1)
plot(out.tout,out.sensed_torques(:,9),'LineWidth',1)
plot(out.tout,out.sensed_torques(:,12),'LineWidth',1)
ylim([-35 35])
yticks([-35 0 35])
xlabel('time (s)')
set(gca,'FontName','Arial','FontSize',14)

sgtitle('Stair-climbing','FontName','Arial','FontSize',16,'Fontweight','bold')

%% Task 2

% Calculate distance and time
final_dist = 10+out.x(30626,1);
ind = find(out.x(:,1) > final_dist,1,"first");
time = out.tout(ind)-6;
score = round(200/time,2);

% Plot
figure;
subplot(1,2,1)
hold on;
plot(out.tout,out.x(:,1),'LineWidth',1.5)
plot([6 6],[-2 16],'--k','LineWidth',1)
plot(out.tout(ind),out.x(ind,1),'.r','MarkerSize',15)
ylim([-2 16])
xlim([0 18])
xlabel('time (s)')
ylabel('x-position (m)')
set(gca,'FontName','Arial','FontSize',14)

subplot(1,2,2)
hold on;
plot(out.tout,out.x(:,7),'LineWidth',1.5)
plot([6 6],[-0.2 1.2],'--k','LineWidth',1)
plot(out.tout(ind),out.x(ind,7),'.r','MarkerSize',15)
xlim([0 18])
xlabel('time (s)')
ylabel('forward speed (m/s)')
legend({'','walk-to-run transition','10m finish'})
set(gca,'FontName','Arial','FontSize',14)

title = sprintf('Task 2 Score = %.2f',score);
sgtitle(title,'FontName','Arial','FontSize',16,'Fontweight','bold')

% Flight phase example
figure; hold on;
plot(out.tout,out.pf(:,3),'LineWidth',1.5)
plot(out.tout,out.pf(:,6),'LineWidth',1.5)
plot(out.tout,out.pf(:,9),'LineWidth',1.5)
plot(out.tout,out.pf(:,12),'LineWidth',1.5)
xlabel('time (s)')
ylabel('vertical foot position (m)')
legend({'FL','FR','RL','RR'})
set(gca,'FontName','Arial','FontSize',14)

%% Task 3

figure;
hold on;
plot(out.pf(:,1),out.pf(:,3),'LineWidth',1.5)
plot(out.pf(:,4),out.pf(:,6),'LineWidth',1.5)
plot(out.pf(:,7),out.pf(:,9),'LineWidth',1.5)
plot(out.pf(:,10),out.pf(:,12),'LineWidth',1.5)
loc1 = 0.86;
x = [0 loc1 loc1 loc1+0.2 loc1+0.2 loc1+0.4 loc1+0.4 loc1+0.6 loc1+0.6 loc1+0.8 loc1+0.8 loc1+1.5];
z = [0 0 0.1 0.1 0.2 0.2 0.3 0.3 0.4 0.4 0.5 0.5];
plot(x,z,'LineWidth',1.5)
xlabel('x-position (m)')
ylabel('z-position (m)')
xlim([0.5 2.5])
legend({'FL','FR','RL','RR','Stairs'})
title('Feet did not collide with stairs')
set(gca,'FontName','Arial','FontSize',14)

%% Task 4

% Calculate distance and time
final_dist = 8;
ind = find(out.x(:,1) >= final_dist,1,"first");
time = out.tout(ind);
score = round(100/time,2);

% Plot
figure;
hold on;
plot(out.tout,out.x(:,1),'LineWidth',1.5)
plot(out.tout,out.x(:,2),'LineWidth',1.5)
plot(out.tout,out.x(:,3),'LineWidth',1.5)
plot(out.tout(ind),out.x(ind,1),'.r','MarkerSize',15)
xlabel('time (s)')
ylabel('position (m)')
set(gca,'FontName','Arial','FontSize',14)
legend({'x','y','z','Finish line'})

title = sprintf('Task 4 Score = %.2f',score);
sgtitle(title,'FontName','Arial','FontSize',16,'Fontweight','bold')

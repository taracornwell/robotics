%% AME 556 HW 3
% Tara Cornwell
% due Friday, March 10

%% Q1a. Single bar - Minimal coordinates
clear;

% Minimal coordinates
global params
syms m l g p q dq ddq px dx ddx py dy ddy theta dtheta ddtheta
assume([theta m l dtheta ddtheta g],'real')

% Given moment of inertia
I = (1/12)*m*l^2;

q = theta; % Minimal coordinates will just be angle
params.q = q;
dq = dtheta;
params.dq = dq;
tau = 0;

% Find position and velocity of COM
px = (l/2)*cos(theta);  % x position
py = (l/2)*sin(theta);  % y position
p = [px;py];        
v = jacobian(p,q)*dq;   % x_dot = J(q)*q_dot

% Kinetic and potential energies:
% dtheta is only rotation and completely perpindicular to bar
K = (1/2)*I*dtheta^2 + (1/2)*m*transpose(v)*v; % rotation + translation
P = m*g*py; % mgh

% Find equation of motion
[D,N] = eom(K,P);

% D*q_double_dot + N = tau
ddq = simplify(D\(tau-N)); % Use to compare to part B

%% Q1b. Single bar - Excess coordinates
clearvars px py p q dq v ddq tau
syms px py pc Jc p q dq ddq v tau

tau = [0;0;0];
params.tau = tau;
q = [px;py;theta]; % Excess coordinates will be COM position and angle
params.q = q;
dq = [dx;dy;dtheta];
params.dq = dq;
ddq = [ddx;ddy;ddtheta];
params.ddq = ddq;

% Contact position of bottom bar
pc(1,1) = px-(l/2)*cos(theta); % contact x-position
pc(2,1) = py-(l/2)*sin(theta); % contact y-position
Jc = jacobian(pc,q);

% Derivative of Jc by hand (will use later for Fc)
dJc = [0, 0, (l/2)*(cos(theta))*dtheta;
       0, 0, (l/2)*(sin(theta))*dtheta];

% Find position and velocity of COM
p = [px;py];        
v = jacobian(p,q)*dq;   % x_dot = J(q)*q_dot

% Kinetic and potential energies
% dtheta is only rotation and completely perpindicular to bar
K = (1/2)*I*dtheta^2 + (1/2)*m*transpose(v)*v; % rotation + translation
P = m*g*py; % mgh

% Find equations of motion
[D,N] = eom(K,P);

% Contact inertia (lambda)
lambda = inv(Jc/D*transpose(Jc));

% Contact force (Fc)
Fc = -lambda*(dJc*dq+Jc/D*(tau-N));
ddq = simplify(D\(tau+transpose(Jc)*Fc-N)); % Use to compare to part A

%% Q1d. Simulate the system dynamics of single bar

% Parameters
m = 0.5;
params.m = m;
l = 0.2;
params.l = l;
g = 9.81;

% Simulation settings
tspan = [0; 2];

% Initial conditions
q0 = [0; 0.3; pi/4]; % px, py, theta
dq0 = [0; 0; 0];     % velocities
x0 = [q0; dq0];

% Before impact
options = odeset('Events',@contact_event);
[t1,x1] = ode45(@robot_dynamics1,tspan,x0,options);

% During impact
x1e = x1(end,:);
x02 = impact(x1e); % impact map

% After impact
tspan2 = [t1(end); tspan(end)]; % from after impact until end
[t2,x2] = ode45(@robot_dynamics2,tspan2,x02);

% Plots of x, y, and theta over time
x = [x1;x2];
t = [t1;t2];
figure;
subplot(3,1,1)
plot(t,x(:,1),'LineWidth',1.5);
ylabel('x (m)','FontSize',16,'FontName','Arial')

subplot(3,1,2)
plot(t,x(:,2),'LineWidth',1.5);
ylabel('y (m)','FontSize',16,'FontName','Arial')

subplot(3,1,3)
plot(t,x(:,3),'LineWidth',1.5);
ylabel('\theta (rad)','FontSize',16,'FontName','Arial')
xlabel('time(s)','FontSize',16,'FontName','Arial')

sgtitle('Q1d. COM position and orientation over time','FontSize',16,'FontName','Arial')

%% Q1d. Animation
filename = 'HW3_Q1d';
anim(t,x(:,1:3),1/24,filename); % animate the system and save the simulation video

%% Q1e. Soft contact simulation
clearvars x1 x2 x t1 t2 t

% Soft contact parameters
kp = 10^5;  % spring
kd = 10^3;  % damper
params.kp = kp;
params.kd = kd;

% Before impact
options = odeset('Events',@contact_event);
[t1,x1] = ode45(@robot_dynamics1,tspan,x0,options);

% Initial contact
xc = x1(end,1)-(l/2)*cos(x1(end,3));
yc = x1(end,2)-(l/2)*sin(x1(end,3));
c0 = [xc;yc];
params.c0 = c0;

% After impact
tspan2 = [t1(end); tspan(end)]; % from after impact until end
[t2,x2] = ode45(@robot_dynamics3,tspan2,x1(end,:)');

% Plots of x, y, and theta over time
x = [x1;x2];
t = [t1;t2];
figure;
subplot(3,1,1)
plot(t,x(:,1),'LineWidth',1.5);
ylabel('x (m)','FontSize',16,'FontName','Arial')

subplot(3,1,2)
plot(t,x(:,2),'LineWidth',1.5);
ylabel('y (m)','FontSize',16,'FontName','Arial')

subplot(3,1,3)
plot(t,x(:,3),'LineWidth',1.5);
ylabel('\theta (rad)','FontSize',16,'FontName','Arial')
xlabel('time(s)','FontSize',16,'FontName','Arial')

sgtitle('Q1e. COM position and orientation over time (soft contact)','FontSize',16,'FontName','Arial')

%% Q1e. Animation
filename = 'HW3_Q1e';
anim(t,x(:,1:3),1/24,filename); % animate the system and save the simulation video

%% Q1f. Simulink

% Run Simulink simulation
m = 0.5;
l = 0.2;
I = (1/12)*m*l^2;
kp = 10^5;
kd = 10^3;
I1 = double(subs(I));
out = sim('AME556_HW3_Q1.slx',2);

% Plots of x, y, and theta over time
clearvars x y t theta
x(:,1) = out.px.signals.values(1,1,:);
y(:,1) = out.py.signals.values(1,1,:);
theta(:,1) = abs((out.theta.signals.values(1,1,:)-pi/2));
time = out.px.time;

figure;
subplot(3,1,1)
plot(time,x,'LineWidth',1.5);
ylim([-1e-5 1e-5])
ylabel('x (m)','FontSize',16,'FontName','Arial')
xticks([0 0.4 0.8 1.2 1.6 2])

subplot(3,1,2)
plot(time,y,'LineWidth',1.5);
ylabel('y (m)','FontSize',16,'FontName','Arial')
xticks([0 0.4 0.8 1.2 1.6 2])

subplot(3,1,3)
plot(time,theta,'LineWidth',1.5);
ylabel('\theta (rad)','FontSize',16,'FontName','Arial')
xlabel('time(s)','FontSize',16,'FontName','Arial')
ylim([0 0.8])
yticks([0 0.4 0.8])
xticks([0 0.4 0.8 1.2 1.6 2])

sgtitle('Q1f. Simscape simulation','FontSize',16,'FontName','Arial')

%% Q2a. 2-link w/ minimal coordinates
clear;
global params
syms theta1 theta2 theta3 q dtheta1 dtheta2 dtheta3 dq
assume([theta1 theta2 theta3 dtheta1 dtheta2 dtheta3],'real')

% Parameters
m1 = 0.5; % kg
m2 = 0.7;
l1 = 0.2; % m
l2 = 0.3; 
g = 9.81; % m/s^2
I1 = (1/12)*m1*l1^2;
I2 = (1/12)*m2*l2^2;

% Minimal coordinates
q = [theta1; theta2; theta3];
params.q = q;
dq = [dtheta1; dtheta2; dtheta3];
params.dq = dq;
tau = [0; 0; 0];
params.tau = tau;

% Link 1 (top bar)
px1 = 0.5*l1*cos(theta1)-l2*cos(pi-theta1-theta2); % x COM position
py1 = l2*sin(pi-theta1-theta2) + 0.5*l1*sin(theta1); % y COM position
pc1 = [px1; py1];
Jc1 = jacobian(pc1,q);
v1 = Jc1*dq; % linear velocity of COM = Jacobian of position*dq
wn1 = dtheta1; % angular velocity

K1 = 0.5*m1*transpose(v1)*v1 + 0.5*I1*wn1^2; % Kinetic
P1 = m1*g*py1; % Potential

% Link 2
px2 = -0.5*l2*cos(pi-theta1-theta2);
py2 = 0.5*l2*sin(pi-theta1-theta2);
pc2 = [px2; py2];
Jc2 = jacobian(pc2,q);
v2 = Jc2*dq;
wn2 = dtheta3;

K2 = 0.5*m2*transpose(v2)*v2 + 0.5*I2*wn2^2; % Kinetic
P2 = m2*g*py2; % Potential

% Sum kinetic and potential energies
K = simplify(K1 + K2);
P = simplify(P1 + P2);

% Dynamics: Find equations of motion
[D,N] = eom(K,P);

% D*q_double_dot + N = tau
ddq = simplify(D\(tau-N));

% Initial conditions
theta0 = [pi/4; pi/3];
theta0(3,1) = theta0(1)+theta0(2); % dummy variable
dtheta0 = [0; 0; 0];
x0 = [theta0; dtheta0];

% ODE solver
tspan = [0;2];
[t,x]=ode45(@dynamics_2link_inelastic,tspan,x0);

% Plot x1, y1, theta1, theta2 over time
figure;
for ii = 1:length(t)
    x1(ii,1) = 0.5*l1*cos(x(ii,1))-l2*cos(pi-x(ii,1)-x(ii,2));
    y1(ii,1) = l2*sin(pi-x(ii,1)-x(ii,2)) + 0.5*l1*sin(x(ii,1));
end

subplot(2,2,1)
plot(t,x1,'LineWidth',1.5);
ylabel('x_1 (m)','FontSize',16,'FontName','Arial')

subplot(2,2,2)
plot(t,y1,'LineWidth',1.5);
ylabel('y_1 (m)','FontSize',16,'FontName','Arial')

subplot(2,2,3)
plot(t,x(:,1),'LineWidth',1.5);
ylabel('\theta_1 (rad)','FontSize',16,'FontName','Arial')
xlabel('time(s)','FontSize',16,'FontName','Arial')

subplot(2,2,4)
plot(t,x(:,2),'LineWidth',1.5);
ylabel('\theta_2 (rad)','FontSize',16,'FontName','Arial')
xlabel('time(s)','FontSize',16,'FontName','Arial')

sgtitle('Q2a. Minimal Coordinate Simulation','FontSize',16,'FontName','Arial')

%% Q2a. Animation
filename = 'HW3_Q2a';
anim(t,x(:,1:2),1/24,filename); % animate the system and save the simulation video

%% Q2b. Excess coordinates
clear;
global params
syms px py dx dy theta1 theta2 q dtheta1 dtheta2 dq
assume([px py dx dy theta1 theta2 dtheta1 dtheta2],'real')

% Parameters
m1 = 0.5; % kg
m2 = 0.7;
l1 = 0.2; % m
l2 = 0.3; 
params.l1 = l1;
params.l2 = l2;
g = 9.81; % m/s^2
I1 = (1/12)*m1*l1^2;
I2 = (1/12)*m2*l2^2;

% Excess coordinates
q = [px; py; theta1; theta2];
params.q = q;
dq = [dx; dy; dtheta1; dtheta2];
params.dq = dq;
tau = [0; 0; 0; 0];
params.tau = tau;

% Contact point position
pc = [px-0.5*l1*cos(theta1)+l2*cos(pi-theta1-theta2); 
      py-0.5*l1*sin(theta1)-l2*sin(pi-theta1-theta2)];
Jc = jacobian(pc,q); % Jacobian of contact point position

% Take derivitive by hand
dJc = [0, 0, 0.5*l1*cos(theta1)*dtheta1+l2*cos(theta1+theta2)*(dtheta1+dtheta2), l2*cos(theta1+theta2)*(dtheta1+dtheta2);
       0, 0, l2*sin(theta1+theta2)*(dtheta1+dtheta2)+0.5*l1*sin(theta1)*dtheta1, l2*sin(theta1+theta2)*(dtheta1+dtheta2)];

% Link 1
pc1 = [px; py];
Jc1 = jacobian(pc1,q);
v1 = Jc1*dq; % Linear velocity
wn1 = dtheta1; % Angular velocity

K1 = 0.5*m1*transpose(v1)*v1 + 0.5*I1*wn1^2; % Kinetic
P1 = m1*g*py; % Potential

% Link 2
px2 = px-0.5*l1*cos(theta1)+0.5*l2*cos(pi-theta1-theta2);
py2 = py-0.5*l1*sin(theta1)-0.5*l2*sin(pi-theta1-theta2);
pc2 = [px2; py2];
Jc2 = jacobian(pc2,q);
v2 = Jc2*dq; % Linear velocity
wn2 = dtheta1+dtheta2; % Angular velocity

K2 = 0.5*m2*transpose(v2)*v2 + 0.5*I2*wn2^2; % Kinetic
P2 = m2*g*py2; % Potential

% Sum kinetic and potential energies
K = simplify(K1 + K2);
P = simplify(P1 + P2);

% Dynamics: find equations of motion
[D,N] = eom(K,P);

% Contact inertia (lambda)
lambda = simplify(inv(Jc/D*transpose(Jc)));

% Contact force (Fc)
Fc = -lambda*(dJc*dq+Jc/D*(tau-N));

% ddq now also depends on contact force
ddq = simplify(D\(tau+transpose(Jc)*Fc-N));

% Initial conditions
theta1_0 = pi/4;
theta2_0 = pi/3;
px0 = -l2*cos(pi-theta1_0-theta2_0)+0.5*l1*cos(theta1_0);
py0 = l2*sin(pi-theta1_0-theta2_0)+0.5*l1*sin(theta1_0);
q0 = [px0; py0; theta1_0; theta2_0];
dq0 = [0; 0; 0; 0];
x0 = [q0; dq0];

% ODE solver
tspan = [0;2];
[t,x]=ode45(@dynamics_2link_inelastic,tspan,x0);

% Plot x1, y1, theta1, theta2 over time
figure;

subplot(2,2,1)
plot(t,x(:,1),'LineWidth',1.5);
ylabel('x_1 (m)','FontSize',16,'FontName','Arial')

subplot(2,2,2)
plot(t,x(:,2),'LineWidth',1.5);
ylabel('y_1 (m)','FontSize',16,'FontName','Arial')

subplot(2,2,3)
plot(t,x(:,3),'LineWidth',1.5);
ylabel('\theta_1 (rad)','FontSize',16,'FontName','Arial')
xlabel('time(s)','FontSize',16,'FontName','Arial')

subplot(2,2,4)
plot(t,x(:,4),'LineWidth',1.5);
ylabel('\theta_2 (rad)','FontSize',16,'FontName','Arial')
xlabel('time(s)','FontSize',16,'FontName','Arial')

sgtitle('Q2b. Excess Coordinate Simulation','FontSize',16,'FontName','Arial')

%% Q2b. ANIMATION
filename = 'HW3_Q2b';
anim(t,x(:,3:4),1/24,filename); % animate the system and save the simulation video

%% Q2c. Excess coordinates + inelastic impact
clearvars x1 x2 x t1 t2 t

% Initial conditions
q0 = [0;0.6;pi/4;pi/3];
dq0 = [0;0;0;0];
x0 = [q0;dq0];

% Before impact
options = odeset('Events',@contact_event2);
[t1,x1] = ode45(@dynamics_2link_flight,tspan,x0,options);

% During impact
x1e = x1(end,:);
x02 = impact2(x1e); % impact map

% After impact
tspan2 = [t1(end); tspan(end)]; % from impact until end
[t2,x2] = ode45(@dynamics_2link_inelastic,tspan2,x02);

% Plots of x1, y1, and thetas over time
x = [x1;x2];
t = [t1;t2];

figure;
subplot(2,2,1)
plot(t,x(:,1),'LineWidth',1.5);
ylabel('x_1 (m)','FontSize',16,'FontName','Arial')

subplot(2,2,2)
plot(t,x(:,2),'LineWidth',1.5);
ylabel('y_1 (m)','FontSize',16,'FontName','Arial')

subplot(2,2,3)
plot(t,x(:,3),'LineWidth',1.5);
ylabel('\theta_1 (rad)','FontSize',16,'FontName','Arial')
xlabel('time(s)','FontSize',16,'FontName','Arial')

subplot(2,2,4)
plot(t,x(:,4),'LineWidth',1.5);
ylabel('\theta_2 (rad)','FontSize',16,'FontName','Arial')
xlabel('time(s)','FontSize',16,'FontName','Arial')

sgtitle('Q2c. Inelastic simulation','FontSize',16,'FontName','Arial')

%% Q2c. ANIMATION
filename = 'HW3_Q2c';
anim(t,x(:,1:4),1/24,filename); % animate the system and save the simulation video

%% Q2d. Soft contact
clearvars x1 x2 x t1 t2 t

% Soft contact parameters
kp = 10^5; % spring
kd = 10^3; % damper
params.kp = kp;
params.kd = kd;

% Before impact
options = odeset('Events',@contact_event2);
[t1,x1] = ode45(@dynamics_2link_flight,tspan,x0,options);

% Initial contact (relative to link 1's COM)
xc = x1(end,1)-(l1/2)*cos(x1(end,3))-l2*cos(x1(end,3)+x1(end,4));
yc = x1(end,2)-(l1/2)*sin(x1(end,3))-l2*sin(x1(end,3)+x1(end,4));
c0 = [xc;yc];
params.c0 = c0;

% After impact
tspan2 = [t1(end); tspan(end)]; % from after impact until end
[t2,x2] = ode45(@dynamics_2link_soft,tspan2,x1(end,:)');

% Plots of x1, y1, and thetas over time
x = [x1;x2];
t = [t1;t2];

figure;
subplot(2,2,1)
plot(t,x(:,1),'LineWidth',1.5);
ylabel('x_1 (m)','FontSize',16,'FontName','Arial')

subplot(2,2,2)
plot(t,x(:,2),'LineWidth',1.5);
ylabel('y_1 (m)','FontSize',16,'FontName','Arial')

subplot(2,2,3)
plot(t,x(:,3),'LineWidth',1.5);
ylabel('\theta_1 (rad)','FontSize',16,'FontName','Arial')
xlabel('time(s)','FontSize',16,'FontName','Arial')

subplot(2,2,4)
plot(t,x(:,4),'LineWidth',1.5);
ylabel('\theta_2 (rad)','FontSize',16,'FontName','Arial')
xlabel('time(s)','FontSize',16,'FontName','Arial')

sgtitle('Q2d. Soft contact simulation','FontSize',16,'FontName','Arial')

%% Q2d. ANIMATION
filename = 'HW3_Q2d';
anim(t,x(:,1:4),1/24,filename); % animate the system and save the simulation video

%% Q2e. Simulink

% Parameters
m1 = 0.5;
m2 = 0.7;
l1 = 0.2;
l2 = 0.3;
I1 = (1/12)*m1*l1^2;
I2 = (1/12)*m2*l2^2;
kp = 10^5;
kd = 10^3;
I1 = double(subs(I1));

out = sim('AME556_HW3_Q2.slx',2);

% Plots of x, y, and theta over time
clearvars x y t theta1 theta_raw1 theta2
x(:,1) = out.px.signals.values(1,1,:)-out.px.signals.values(1,1,1);
y(:,1) = out.py.signals.values(1,1,:);
% Convert angles to the convention on assignment
theta_raw1(:,1) = out.theta1.signals.values(1,1,:);
for ii = 1:length(theta_raw1)
    if theta_raw1(ii) >= pi/2 % > or = 90 deg --> 0 or negative
        theta1(ii,1) = -(theta_raw1(ii)-pi/2);
    else
        theta1(ii,1) = pi/2-theta_raw1(ii);
    end
end

theta2(:,1) = -out.theta2.signals.values(1,1,:)-pi/2;

time = out.px.time;

figure;
subplot(2,2,1)
plot(time,x,'LineWidth',1.5);
ylabel('x_1 (m)','FontSize',16,'FontName','Arial')
xticks([0 0.4 0.8 1.2 1.6 2])

subplot(2,2,2)
plot(time,y,'LineWidth',1.5);
ylabel('y_1 (m)','FontSize',16,'FontName','Arial')
xticks([0 0.4 0.8 1.2 1.6 2])

subplot(2,2,3)
plot(time,theta1,'LineWidth',1.5);
ylabel('\theta_1 (rad)','FontSize',16,'FontName','Arial')
xlabel('time(s)','FontSize',16,'FontName','Arial')
xticks([0 0.4 0.8 1.2 1.6 2])

subplot(2,2,4)
plot(time,theta2,'LineWidth',1.5);
ylabel('\theta_2 (rad)','FontSize',16,'FontName','Arial')
xlabel('time(s)','FontSize',16,'FontName','Arial')
xticks([0 0.4 0.8 1.2 1.6 2])

sgtitle('Q2e. Simscape simulation','FontSize',16,'FontName','Arial')

%% Q3. Unitree A1 robot

% Import model
% smimport('a1.urdf')

% Set initial joint angles
hip0 = 0;
thigh0 = pi/3;
calf0 = -4*pi/5;

% Set desired joint angles (desired velocities=0)
hipd = 0;
thighd = 0.95;
calfd = -1.6;

% PD gains for controller (hand-tune)
kp = 25;
kd = 4;

% Run simulation
out = sim('a1_HW3.slx',2);

% Plots of joint torques, angles, and velocities for each leg
clearvars t

figure;

t = out.tout;

subplot(3,3,1)
hold on;
plot(t,out.FR_hip(:,1),'linewidth',1)
plot(t,out.FL_hip(:,1),'linewidth',1)
plot(t,out.RR_hip(:,1),'linewidth',1)
plot(t,out.RL_hip(:,1),'linewidth',1)
title('Hip joint angles')
ylabel('rad')
set(gca,'FontSize',14)
subplot(3,3,2)
hold on;
plot(t,out.FR_thigh(:,1),'linewidth',1)
plot(t,out.FL_thigh(:,1),'linewidth',1)
plot(t,out.RR_thigh(:,1),'linewidth',1)
plot(t,out.RL_thigh(:,1),'linewidth',1)
title('Thigh joint angles')
set(gca,'FontSize',14)
subplot(3,3,3)
hold on;
plot(t,out.FR_calf(:,1),'linewidth',1)
plot(t,out.FL_calf(:,1),'linewidth',1)
plot(t,out.RR_calf(:,1),'linewidth',1)
plot(t,out.RL_calf(:,1),'linewidth',1)
title('Calf joint angles')
legend({'Front Right','Front Left','Rear Right','Rear Left'})
set(gca,'FontSize',14)

subplot(3,3,4)
hold on;
plot(t,out.FR_hip(:,2),'linewidth',1)
plot(t,out.FL_hip(:,2),'linewidth',1)
plot(t,out.RR_hip(:,2),'linewidth',1)
plot(t,out.RL_hip(:,2),'linewidth',1)
title('Hip joint velocities')
ylabel('rad/s')
set(gca,'FontSize',14)
subplot(3,3,5)
hold on;
plot(t,out.FR_thigh(:,2),'linewidth',1)
plot(t,out.FL_thigh(:,2),'linewidth',1)
plot(t,out.RR_thigh(:,2),'linewidth',1)
plot(t,out.RL_thigh(:,2),'linewidth',1)
title('Thigh joint velocities')
set(gca,'FontSize',14)
subplot(3,3,6)
hold on;
plot(t,out.FR_calf(:,2),'linewidth',1)
plot(t,out.FL_calf(:,2),'linewidth',1)
plot(t,out.RR_calf(:,2),'linewidth',1)
plot(t,out.RL_calf(:,2),'linewidth',1)
title('Calf joint velocities')
set(gca,'FontSize',14)

subplot(3,3,7)
hold on;
plot(t,out.FR_hip(:,3),'linewidth',1)
plot(t,out.FL_hip(:,3),'linewidth',1)
plot(t,out.RR_hip(:,3),'linewidth',1)
plot(t,out.RL_hip(:,3),'linewidth',1)
title('Hip joint torques')
ylabel('Nm')
xlabel('Time (s)')
set(gca,'FontSize',14)
subplot(3,3,8)
hold on;
plot(t,out.FR_thigh(:,3),'linewidth',1)
plot(t,out.FL_thigh(:,3),'linewidth',1)
plot(t,out.RR_thigh(:,3),'linewidth',1)
plot(t,out.RL_thigh(:,3),'linewidth',1)
title('Thigh joint torques')
xlabel('Time (s)')
set(gca,'FontSize',14)
subplot(3,3,9)
hold on;
plot(t,out.FR_calf(:,3),'linewidth',1)
plot(t,out.FL_calf(:,3),'linewidth',1)
plot(t,out.RR_calf(:,3),'linewidth',1)
plot(t,out.RL_calf(:,3),'linewidth',1)
title('Calf joint torques')
xlabel('Time (s)')
set(gca,'FontSize',14)

sgtitle('Q3. A1 robot simulation','FontWeight','Bold','FontSize',16)

%% Functions

% Function to find D(q) and N(q,q_dot) from kinetic and potential energies
function [D,N] = eom(K,P)
    global params
    % Lagrange
    L = K-P;

    % df/dt = df/dq * q_dot + df/dq_dot * q_double_dot

    % f = dL/dq_dot
    f = jacobian(L,params.dq)'; % vector

    % D = df/dq_dot
    D = jacobian(f,params.dq); % square matrix

    % D_dot = df/dq
    dD = jacobian(f,params.q); % square matrix

    % g = dP/dq
    g = jacobian(P,params.q)'; % vector

    % C*q_dot = D_dot * q_dot - dK/dq
    Cqdot = dD*params.dq - jacobian(K,params.q)'; % vector

    % N = C*q_dot + g
    N = Cqdot + g; % 3x1 vector
end

% Q1c. Impact map (find function that relates x- to x+)
function x_after = impact(x_before)

    global params

    % Parameters
    m = params.m;
    l = params.l;
    theta = x_before(3);

    % Find number of q's and dq's
    num = length(x_before)/2;
    % Define q_after
    x_after(1:num,1) = x_before(1:num); % q_before = q_after
    % Define dq_before
    dq_before(1:num,1) = x_before(num+1:end);

    % D(q) from eom function (solved earlier)
    D = [m, 0,          0;
         0, m,          0;
         0, 0, (l^2*m)/12];

    % Jc and lambda (solved earlier)
    Jc = [1, 0,  (l*sin(theta))/2;
          0, 1, -(l*cos(theta))/2];

    lambda = [-(m*(3*sin(theta)^2 - 4))/4,         (3*m*sin(2*theta))/8;
                     (3*m*sin(2*theta))/8, m*((3*sin(theta)^2)/4 + 1/4)];

    deltaFc = -lambda*Jc*dq_before;

    % Define dq_after
    x_after(num+1:2*num,1) = dq_before+D\transpose(Jc)*deltaFc;
end

% myEventsFcn to detect when contact is made
function [position,isterminal,direction] = contact_event(t,x)
    global params
    l = params.l;
    position = x(2)-l/2*sin(x(3));  % y position
    isterminal = 1;                 % halt the ode solver
    direction = 0;
end

function dx=robot_dynamics1(t,x)
    global params

    % Parameters
    m = params.m;
    l = params.l;
    g = 9.81;

    % Control input
    u=params.tau;

    % Organize values from x input where x = [q; q_dot]
    vx = x(4);
    vy = x(5);
    dtheta = x(6);

    % First 3 = [x vel, y vel, dtheta];
    dx(1:3,1) = [vx; vy; dtheta];

    % D(q) and N(q,q_dot) from eom function (solved earlier)
    D = [m, 0,          0;
         0, m,          0;
         0, 0, (l^2*m)/12];

    N = [0;
         g*m;
         0];
    
    % D*q_double_dot + N = tau
    ddq = D\(u-N);

    % Last 3 = joint accels (q_double_dot)
    dx(4:6,1) = ddq;
end

function dx=robot_dynamics2(t,x)
    global params

    % Parameters
    m = params.m;
    l = params.l;
    g = 9.81;

    % Control input
    u=params.tau;

    % Organize values from x input where x = [q; q_dot]
    theta = x(3);
    vx = x(4);
    vy = x(5);
    dtheta = x(6);

    % First 3 = [x vel, y vel, dtheta];
    dx(1:3,1) = [vx; vy; dtheta];

    % D(q) and N(q,q_dot) from eom function (solved earlier)
    D = [m, 0,          0;
         0, m,          0;
         0, 0, (l^2*m)/12];

    N = [0;
         g*m;
         0];

    % Jc and Fc (solved earlier)
    Jc = [1, 0,  (l*sin(theta))/2;
          0, 1, -(l*cos(theta))/2];

    Fc = [(m*(- 4*l*cos(theta)*dtheta^2 + 3*g*sin(2*theta)))/8;
          (m*(g + 3*g*sin(theta)^2 - 2*dtheta^2*l*sin(theta)))/4];
    
    % D*q_double_dot + N = tau
    ddq = D\(u+transpose(Jc)*Fc-N);

    % Last 3 = joint accels (q_double_dot)
    dx(4:6,1) = ddq;
end

function dx=robot_dynamics3(t,x)
    global params

    % Parameters
    m = params.m;
    l = params.l;
    kp = params.kp;
    kd = params.kd;
    g = 9.81;

    % Initial contact
    c0 = params.c0;
    xc0 = c0(1);
    yc0 = c0(2);

    % Control input
    u=params.tau;

    % Organize values from x input where x = [q; q_dot]
    px = x(1);
    py = x(2);
    theta = x(3);
    vx = x(4);
    vy = x(5);
    dtheta = x(6);

    % First 3 = [x vel, y vel, dtheta];
    dx(1:3,1) = [vx; vy; dtheta];

    % D(q) and N(q,q_dot) from eom function (solved earlier)
    D = [m, 0,          0;
         0, m,          0;
         0, 0, (l^2*m)/12];

    N = [0;
         g*m;
         0];

    % Jc (solved earlier)
    Jc = [1, 0,  (l*sin(theta))/2;
          0, 1, -(l*cos(theta))/2];

    % Contact force
    xc = px-(l/2)*cos(theta); % new contact x-position
    yc = py-(l/2)*sin(theta); % new contact y-position

    dxc = vx+(l/2)*sin(theta)*dtheta; % velocities (deriv by hand)
    dyc = vy-(l/2)*cos(theta)*dtheta;

    Fcx = -kp*(xc-xc0)-kd*dxc;
    Fcy = -kp*(yc-yc0)-kd*dyc;
    Fc = [Fcx;Fcy];
    
    % D*q_double_dot + N = tau
    ddq = D\(u+transpose(Jc)*Fc-N);

    % Last 3 = joint accels (q_double_dot)
    dx(4:6,1) = ddq;
end

% Q2c. Impact map (find function that relates x- to x+)
function x_after = impact2(x_before)

    theta1 = x_before(3);
    theta2 = x_before(4);

    % Find number of q's and dq's
    num = length(x_before)/2;
    % Define q_after
    x_after(1:num,1) = x_before(1:num); % q_before = q_after

    % Define dq_before
    dq_before(1:num,1) = x_before(num+1:end);

    % D(q) from eom function (solved earlier)
    D = [                                                6/5,                                                     0,   (21*sin(theta1 + theta2))/200 + (7*sin(theta1))/100,   (21*sin(theta1 + theta2))/200;
                                                           0,                                                   6/5, - (21*cos(theta1 + theta2))/200 - (7*cos(theta1))/100,  -(21*cos(theta1 + theta2))/200;
         (21*sin(theta1 + theta2))/200 + (7*sin(theta1))/100, - (21*cos(theta1 + theta2))/200 - (7*cos(theta1))/100,                       (21*cos(theta2))/1000 + 89/3000, (21*cos(theta2))/2000 + 21/1000;
                               (21*sin(theta1 + theta2))/200,                        -(21*cos(theta1 + theta2))/200,                       (21*cos(theta2))/2000 + 21/1000,                         21/1000];

    % Jc and lambda (solved earlier)
    Jc = [1, 0,   (3*sin(theta1 + theta2))/10 + sin(theta1)/10,  (3*sin(theta1 + theta2))/10;
          0, 1, - (3*cos(theta1 + theta2))/10 - cos(theta1)/10, -(3*cos(theta1 + theta2))/10];

    lambda = [(3*(35*cos(2*theta1) + 105*cos(2*theta2) - 323*cos(2*theta1 + 2*theta2) - 505))/(5*(180*cos(2*theta2) - 524)),                            (3*(35*sin(2*theta1) - 323*sin(2*theta1 + 2*theta2)))/(10*(90*cos(2*theta2) - 262));
              (3*(35*sin(2*theta1) - 323*sin(2*theta1 + 2*theta2)))/(10*(90*cos(2*theta2) - 262)),                          -(3*(35*cos(2*theta1) - 105*cos(2*theta2) - 323*cos(2*theta1 + 2*theta2) + 505))/(5*(180*cos(2*theta2) - 524))];

    deltaFc = -lambda*Jc*dq_before;

    % Define dq_after
    x_after(num+1:2*num,1) = dq_before+D\transpose(Jc)*deltaFc;
end

% myEventsFcn to detect when contact is made
function [position,isterminal,direction] = contact_event2(t,x)
    global params
    l1 = params.l1;
    l2 = params.l2;
    position = x(2)-l1/2*sin(x(3))-l2*sin(x(3)+x(4));   % y position
    isterminal = 1;                                     % halt the ode solver
    direction = 0;
end

function dx=dynamics_2link_flight(t,x) % FLIGHT PHASE
    global params

    % Control input
    u=params.tau;
    
    % Organize values from x input where x = [q; q_dot]
    theta1 = x(3);
    theta2 = x(4);
    dtheta1 = x(7);
    dtheta2 = x(8);

    % First 4 = joint vels (q_dot)
    dx(1:4,1) = [x(5); x(6); dtheta1; dtheta2];

    % D(q) and N(q,q_dot) from eom function (solved earlier)
    D = [                                                6/5,                                                     0,   (21*sin(theta1 + theta2))/200 + (7*sin(theta1))/100,   (21*sin(theta1 + theta2))/200;
                                                           0,                                                   6/5, - (21*cos(theta1 + theta2))/200 - (7*cos(theta1))/100,  -(21*cos(theta1 + theta2))/200;
         (21*sin(theta1 + theta2))/200 + (7*sin(theta1))/100, - (21*cos(theta1 + theta2))/200 - (7*cos(theta1))/100,                       (21*cos(theta2))/1000 + 89/3000, (21*cos(theta2))/2000 + 21/1000;
                               (21*sin(theta1 + theta2))/200,                        -(21*cos(theta1 + theta2))/200,                       (21*cos(theta2))/2000 + 21/1000,                         21/1000];

    
    N = [(21*dtheta1^2*cos(theta1 + theta2))/200 + (21*dtheta2^2*cos(theta1 + theta2))/200 + (7*dtheta1^2*cos(theta1))/100 + (21*dtheta1*dtheta2*cos(theta1 + theta2))/100;
        (7*dtheta1^2*sin(theta1))/100 + (21*dtheta1^2*sin(theta1 + theta2))/200 + (21*dtheta2^2*sin(theta1 + theta2))/200 + (21*dtheta1*dtheta2*sin(theta1 + theta2))/100 + 2943/250;
        - (20601*cos(theta1 + theta2))/20000 - (6867*cos(theta1))/10000 - (21*dtheta2^2*sin(theta2))/2000 - (21*dtheta1*dtheta2*sin(theta2))/1000;
        (21*sin(theta2)*dtheta1^2)/2000 - (20601*cos(theta1 + theta2))/20000];
    
    % Last 4 = joint accels (q_double_dot)
    % D*q_double_dot + N = tau
    ddq = D\(u-N);
    dx(5:8,1) = ddq;
end

function dx=dynamics_2link_inelastic(t,x) % INELASTIC CONTACT
    global params

    % Control input
    u=params.tau;

    if length(x) == 6 % MINIMAL
        % Organize values from x input where x = [q; q_dot]
        theta1 = x(1);
        theta2 = x(2);
        theta3 = x(3);
        dtheta1 = x(4);
        dtheta2 = x(5);
        dtheta3 = x(6);
    
        % First 2 = joint vels (q_dot)
        dx(1:3,1) = [dtheta1; dtheta2; dtheta3];
    
        % D(q) and N(q,q_dot) from eom function (solved earlier)
        D = [(3*cos(theta2))/100 + 809/12000, (3*cos(theta2))/200 + 243/4000,       0;
             (3*cos(theta2))/200 + 243/4000,                       243/4000,       0;
                                          0,                              0, 21/4000];
   
        N = [(50031*cos(theta1 + theta2))/20000 + (981*cos(theta1))/2000 - dtheta2*((3*dtheta1*sin(theta2))/100 + (3*dtheta2*sin(theta2))/200);
                                                               (3*sin(theta2)*dtheta1^2)/200 + (50031*cos(theta1 + theta2))/20000;
                                                                                                                                0];
    elseif length(x) == 8 % EXCESS
        % Organize values from x input where x = [q; q_dot]
        theta1 = x(3);
        theta2 = x(4);
        dtheta1 = x(7);
        dtheta2 = x(8);
    
        % First 4 = joint vels (q_dot)
        dx(1:4,1) = [x(5); x(6); dtheta1; dtheta2];
    
        % D(q) and N(q,q_dot) from eom function (solved earlier)
        D = [                                                6/5,                                                     0,   (21*sin(theta1 + theta2))/200 + (7*sin(theta1))/100,   (21*sin(theta1 + theta2))/200;
                                                               0,                                                   6/5, - (21*cos(theta1 + theta2))/200 - (7*cos(theta1))/100,  -(21*cos(theta1 + theta2))/200;
             (21*sin(theta1 + theta2))/200 + (7*sin(theta1))/100, - (21*cos(theta1 + theta2))/200 - (7*cos(theta1))/100,                       (21*cos(theta2))/1000 + 89/3000, (21*cos(theta2))/2000 + 21/1000;
                                   (21*sin(theta1 + theta2))/200,                        -(21*cos(theta1 + theta2))/200,                       (21*cos(theta2))/2000 + 21/1000,                         21/1000];

        
        N = [(21*dtheta1^2*cos(theta1 + theta2))/200 + (21*dtheta2^2*cos(theta1 + theta2))/200 + (7*dtheta1^2*cos(theta1))/100 + (21*dtheta1*dtheta2*cos(theta1 + theta2))/100;
            (7*dtheta1^2*sin(theta1))/100 + (21*dtheta1^2*sin(theta1 + theta2))/200 + (21*dtheta2^2*sin(theta1 + theta2))/200 + (21*dtheta1*dtheta2*sin(theta1 + theta2))/100 + 2943/250;
            - (20601*cos(theta1 + theta2))/20000 - (6867*cos(theta1))/10000 - (21*dtheta2^2*sin(theta2))/2000 - (21*dtheta1*dtheta2*sin(theta2))/1000;
            (21*sin(theta2)*dtheta1^2)/2000 - (20601*cos(theta1 + theta2))/20000];
    end

    % Last half of dx = joint accels (ddq)
    if length(x) == 6
        % D*q_double_dot + N = tau
        ddq = D\(u-N);
        dx(4:6,1) = ddq;
    elseif length(x) == 8
        % D*q_double_dot + N = tau + Jc^T*Fc
        Jc = [1, 0,   (3*sin(theta1 + theta2))/10 + sin(theta1)/10,  (3*sin(theta1 + theta2))/10;
              0, 1, - (3*cos(theta1 + theta2))/10 - cos(theta1)/10, -(3*cos(theta1 + theta2))/10];
        Fc = [((103005*sin(2*theta1))/2 - (950589*sin(2*theta1 + 2*theta2))/2 - 1575*dtheta1^2*cos(theta1 - theta2) + 2850*dtheta1^2*cos(theta1 + 2*theta2) - 1575*dtheta2^2*cos(theta1 - theta2) + 23505*dtheta1^2*cos(theta1 + theta2) + 23505*dtheta2^2*cos(theta1 + theta2) + 1450*dtheta1^2*cos(theta1) - 3150*dtheta1*dtheta2*cos(theta1 - theta2) + 47010*dtheta1*dtheta2*cos(theta1 + theta2))/(45000*cos(2*theta2) - 131000);
              (309015*cos(2*theta2) - 103005*cos(2*theta1) + 950589*cos(2*theta1 + 2*theta2) + 2900*dtheta1^2*sin(theta1) - 3150*dtheta1^2*sin(theta1 - theta2) + 5700*dtheta1^2*sin(theta1 + 2*theta2) - 3150*dtheta2^2*sin(theta1 - theta2) + 47010*dtheta1^2*sin(theta1 + theta2) + 47010*dtheta2^2*sin(theta1 + theta2) - 6300*dtheta1*dtheta2*sin(theta1 - theta2) + 94020*dtheta1*dtheta2*sin(theta1 + theta2) - 1486215)/(2000*(45*cos(2*theta2) - 131))];
        ddq = D\(u+transpose(Jc)*Fc-N);
        dx(5:8,1) = ddq;
    end
end

function dx=dynamics_2link_soft(t,x) % SOFT CONTACT
    global params

    % Control input
    u = params.tau;

    % Parameters
    l1 = params.l1;
    l2 = params.l2;
    kp = params.kp;
    kd = params.kd;
    
    % Organize values from x input where x = [q; q_dot]
    px = x(1);
    py = x(2);
    theta1 = x(3);
    theta2 = x(4);
    vx = x(5);
    vy = x(6);
    dtheta1 = x(7);
    dtheta2 = x(8);

    % First 4 = joint vels (q_dot)
    dx(1:4,1) = [vx; vy; dtheta1; dtheta2];

    % D(q) and N(q,q_dot) from eom function (solved earlier)
    D = [                                                6/5,                                                     0,   (21*sin(theta1 + theta2))/200 + (7*sin(theta1))/100,   (21*sin(theta1 + theta2))/200;
                                                           0,                                                   6/5, - (21*cos(theta1 + theta2))/200 - (7*cos(theta1))/100,  -(21*cos(theta1 + theta2))/200;
         (21*sin(theta1 + theta2))/200 + (7*sin(theta1))/100, - (21*cos(theta1 + theta2))/200 - (7*cos(theta1))/100,                       (21*cos(theta2))/1000 + 89/3000, (21*cos(theta2))/2000 + 21/1000;
                               (21*sin(theta1 + theta2))/200,                        -(21*cos(theta1 + theta2))/200,                       (21*cos(theta2))/2000 + 21/1000,                         21/1000];

    
    N = [(21*dtheta1^2*cos(theta1 + theta2))/200 + (21*dtheta2^2*cos(theta1 + theta2))/200 + (7*dtheta1^2*cos(theta1))/100 + (21*dtheta1*dtheta2*cos(theta1 + theta2))/100;
        (7*dtheta1^2*sin(theta1))/100 + (21*dtheta1^2*sin(theta1 + theta2))/200 + (21*dtheta2^2*sin(theta1 + theta2))/200 + (21*dtheta1*dtheta2*sin(theta1 + theta2))/100 + 2943/250;
        - (20601*cos(theta1 + theta2))/20000 - (6867*cos(theta1))/10000 - (21*dtheta2^2*sin(theta2))/2000 - (21*dtheta1*dtheta2*sin(theta2))/1000;
        (21*sin(theta2)*dtheta1^2)/2000 - (20601*cos(theta1 + theta2))/20000];

    % Last 4 = joint accels (q_double_dot)
    % D*q_double_dot + N = tau + Jc^T*Fc
    Jc = [1, 0,   (3*sin(theta1 + theta2))/10 + sin(theta1)/10,  (3*sin(theta1 + theta2))/10;
          0, 1, - (3*cos(theta1 + theta2))/10 - cos(theta1)/10, -(3*cos(theta1 + theta2))/10];

    % Contact force
    c0 = params.c0;
    xc0 = c0(1);
    yc0 = c0(2);

    xc = px-(l1/2)*cos(theta1)-l2*cos(theta1+theta2); % new contact x-position
    yc = py-(l1/2)*sin(theta1)-l2*sin(theta1+theta2); % new contact y-position

    dxc = vx+(l1/2)*sin(theta1)*dtheta1+l2*sin(theta1+theta2)*(dtheta1+dtheta2); % velocities (deriv by hand)
    dyc = vy-(l1/2)*cos(theta1)*dtheta1-l2*cos(theta1+theta2)*(dtheta1+dtheta2);

    Fcx = -kp*(xc-xc0)-kd*dxc;
    Fcy = -kp*(yc-yc0)-kd*dyc;
    Fc = [Fcx;Fcy];
        
    ddq = D\(u+transpose(Jc)*Fc-N);
    dx(5:8,1) = ddq;
end

% Animate videos
function anim(t,x,ts,filename)
    % Some conditions will have errors with sampling bc of discontinuities
    if contains(filename,'HW3_Q1d') || contains(filename,'HW3_Q1e') ...
            || contains(filename,'HW3_Q2c') || contains(filename,'HW3_Q2d')
        [te1,xe1] = even_sample(t(1:29),x(1:29,:),1/ts);
        [te2,xe2] = even_sample(t(30:end),x(30:end,:),1/ts);
        te = [te1;te2];
        xe = [xe1;xe2];
    else
        [te,xe]=even_sample(t,x,1/ts);  % evenly sample for video frames
    end

    figure(1);
    axes1 = axes;
    
    % Save as a video
    spwriter = VideoWriter(filename,'MPEG-4');
    set(spwriter, 'FrameRate', 1/ts,'Quality',100);
    open(spwriter);
    
    fig1 = figure(1);
    
    % Set limits
    figure_x_limits = [-0.75 0.75];
    figure_y_limits = [-0.75 0.75];
    
    axes1 = axes;
    set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits);
    set(axes1,'Position',[0 0 1 1]);
    set(axes1,'Color','w');
    
    % Loop through the time and plot each data point at a time
    for k = 1:length(te)
        drawone(axes1, xe(k,:)');
        set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits);
        drawnow;
        pause(ts);
        frame = getframe(gcf);
        writeVideo(spwriter, frame);
    end
end

function [Et, Ex] = even_sample(t, x, Fs)
    % CONVERTS A RANDOMLY SAMPLED SIGNAL SET INTO AN EVENLY SAMPLED
    % SIGNAL SET (by interpolation)
    % Obtain the process related parameters
    N = size(x, 2);    % number of signals to be interpolated
    M = size(t, 1);    % Number of samples provided
    t0 = t(1,1);       % Initial time
    tf = t(M,1);       % Final time
    EM = (tf-t0)*Fs;   % Number of samples in the evenly sampled case with
                       % the specified sampling frequency
    Et = linspace(t0, tf, EM)';
    
    % Using linear interpolation (used to be cubic spline interpolation)
    % and re-sample each signal to obtain the evenly sampled forms
    for s = 1:N
        Ex(:,s) = interp1(t(:,1), x(:,s), Et(:,1)); 
    end
end

function drawone(parent, x)
    % Delete existing plot
    tem = get(parent,'Children');
    delete(tem);

    hold on;

    if length(x) == 3 % 1 link
        % X, Y, and theta from input
        px = x(1);
        py = x(2);
        theta = x(3);

        % Parameter(s)
        l = 0.2;

        pl = [px-0.5*l*cos(theta); py-0.5*l*sin(theta)];
        pr = [px+0.5*l*cos(theta); py+0.5*l*sin(theta)];

        plot([pl(1) pr(1)],[pl(2) pr(2)],'-ob','LineWidth',1.5)
        plot(-1:1:1,[0 0 0],'-k')

    elseif length(x) == 2 % 2 links with fixed contact base
        % Get joint positions
        theta1 = x(1);
        theta2 = x(2);
    
        l1 = 0.2;
        l2 = 0.3;
    
        % Endpoint positions
        O0 = [0;0];                                                                                 % Fixed base O0
        O1 = [-l2*cos(pi-theta1-theta2); l2*sin(pi-theta1-theta2)];                                 % Position of 01
        O2 = [l1*cos(theta1)-l2*cos(pi-theta1-theta2); l2*sin(pi-theta1-theta2)+l1*sin(theta1)];    % Position of O2
    
        % Plot each link based on endpoint positions (and origin O0)
        plot([O0(1) O1(1)],[O0(2) O1(2)],'-or','LineWidth',1.5)
        plot([O1(1) O2(1)],[O1(2) O2(2)],'-ob','LineWidth',1.5)
        plot(-1:1:1,[0 0 0],'-k')

    elseif length(x) == 4 % 2 links with flight phase
        % Get joint positions
        px = x(1);
        py = x(2);
        theta1 = x(3);
        theta2 = x(4);
    
        l1 = 0.2;
        l2 = 0.3;
    
        % Endpoint positions
        O2 = [px+0.5*l1*cos(theta1); py+0.5*l1*sin(theta1)];                    % End of top link
        O1 = [px-0.5*l1*cos(theta1); py-0.5*l1*sin(theta1)];                    % Connects 2 links
        O0 = [px-0.5*l1*cos(theta1)+l2*cos(pi-theta1-theta2);py-0.5*l1*sin(theta1)-l2*sin(pi-theta1-theta2)];   % Contact point

        % Plot each link based on endpoint positions (and origin O0)
        plot([O0(1) O1(1)],[O0(2) O1(2)],'-or','LineWidth',1.5)
        plot([O1(1) O2(1)],[O1(2) O2(2)],'-ob','LineWidth',1.5)
        plot(-1:1:1,[0 0 0],'-k')
    end

end
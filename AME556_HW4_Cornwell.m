%% AME 556 HW4
% Tara Cornwell
% due Friday, March 24, 2023

%% Q1. Cart-pole system

clear; clc;
global params

% Given parameters
M = 1;
m = 0.2;
L = 0.3;
g = 9.81;
params.M = M;
params.m = m;
params.L = L;
params.g = g;

% Initial state
x0 = [0; pi/6; 0; 0];

% Desired state
xf = [0; 0; 0; 0];

% Simulation time
tspan = [0; 2];

%% Q1a. LQR control

syms x theta dx dtheta ddx ddtheta state dynamics u

ddx = (u-m*L*sin(theta)*dtheta^2+m*g*cos(theta)*sin(theta))./(M+m-m*cos(theta)^2);
ddtheta = (cos(theta)*ddx+g*sin(theta))./L;

state = [x; theta; dx; dtheta];
dynamics = [dx; dtheta; ddx; ddtheta];

% Linearize to get A and B matrices
A = jacobian(dynamics,state);
B = jacobian(dynamics,u);

% Plug in desired values
theta = 0;
dtheta = 0;
u = 0;
A = double(subs(A));
B = double(subs(B));
params.A = A;
params.B = B;

clearvars theta dtheta
syms theta dtheta

% Given QR values
Q1 = diag([10,10,10,10]);
Q2 = diag([100,0.01,100,0.01]);

R_values = [1; 10; 1];

figure;
for ii = 1:3 % iterate through Q and R values
    clearvars x t Q R

    % Given QR values
    if ii == 1
        Q = Q1;
        dispname = 'Q1, R=1';
    elseif ii == 2
        Q = Q2;
        dispname = 'Q2, R=10';
    elseif ii == 3
        Q = Q2;
        dispname = 'Q2, R=1';
    end
    R = R_values(ii);

    % Define controller
    k = lqr(A,B,Q,R);
    if ii == 1
        k1 = k;
    end
    params.k = k;

    % ODE solver for simulation
    params.controller = 'LQR';
    [t,x]=ode45(@sys_dynamics,tspan,x0);

    % Plot x, theta, torque
    subplot(3,1,1)
    hold on;
    if ii == 1
        plot([0 2],[xf(1),xf(1)],'--k') % Show desired value
    end
    plot(t,x(:,1),'LineWidth',2)
    ylabel('x-position (m)')
    set(gca,'FontSize',14,'FontName','Arial')

    subplot(3,1,2)
    hold on;
    if ii == 1
        plot([0 2],[xf(2),xf(2)],'--k') % Show desired value
    end
    plot(t,x(:,2),'LineWidth',2)
    ylabel('\theta (rad)')
    set(gca,'FontSize',14,'FontName','Arial')
    
    subplot(3,1,3)
    hold on;
    if ii == 1
        plot([0 2],[0 0],'--k','DisplayName','Desired') % Show desired value
    end

    % Recreate control inputs
    for jj=1:length(t)
        u(:,jj)=LQR_controller(t(jj),x(jj,:)');
    end

    plot(t,u,'LineWidth',2,'DisplayName',dispname)
    ylabel('\tau (Nm)')
    set(gca,'FontSize',14,'FontName','Arial')
    xlabel('time (s)')
    legend
end

sgtitle('Q1a. LQR control','FontSize',16,'FontName','Arial')

%% Q1b. QP control

clearvars u x t
params.k = k1;

% ODE solver for simulation
params.controller = 'QP';
[t,x]=ode45(@sys_dynamics,tspan,x0);

%% Q1b. Plot x, theta, torque
figure;
subplot(3,1,1)
hold on;
plot([0 2],[xf(1),xf(1)],'--k') % Show desired value
plot(t,x(:,1),'LineWidth',2,'DisplayName',dispname)
ylabel('x-position (m)')
set(gca,'FontSize',14,'FontName','Arial')

subplot(3,1,2)
hold on;
plot([0 2],[xf(2),xf(2)],'--k') % Show desired value
plot(t,x(:,2),'LineWidth',2)
ylabel('\theta (rad)')
set(gca,'FontSize',14,'FontName','Arial')

subplot(3,1,3)
hold on;
plot([0 2],[0 0],'--k','DisplayName','Desired') % Show desired value
% Recreate control inputs
for jj=1:length(t)
    u(:,jj)=QP_controller(t(jj),x(jj,:)');
end
plot(t,u,'LineWidth',2)
ylabel('\tau (Nm)')
set(gca,'FontSize',14,'FontName','Arial')
xlabel('time (s)')

sgtitle('Q1b. QP control','FontSize',16,'FontName','Arial')

%% Q1c. MPC

clearvars u x t tspan

N = 20;      % number of horizons
dt = 50/1000;% sampling time in seconds
params.N = N;
params.dt = dt;  

% ODE solver for simulation
tspan = 0:dt:2;
params.controller = 'MPC';
[t,x]=ode45(@sys_dynamics,tspan,x0);

%% Q1c. Plot x, theta, torque

figure;
subplot(3,1,1)
hold on;
plot([0 2],[xf(1),xf(1)],'--k') % Show desired value
plot(t,x(:,1),'LineWidth',2,'DisplayName',dispname)
ylabel('x-position (m)')
set(gca,'FontSize',14,'FontName','Arial')

subplot(3,1,2)
hold on;
plot([0 2],[xf(2),xf(2)],'--k') % Show desired value
plot(t,x(:,2),'LineWidth',2)
ylabel('\theta (rad)')
set(gca,'FontSize',14,'FontName','Arial')

subplot(3,1,3)
hold on;
plot([0 2],[0 0],'--k','DisplayName','Desired') % Show desired value
% Recreate control inputs
for jj=1:length(t)
    u(:,jj)=MPC_controller(t(jj),x(jj,:)');
end
plot(t,u,'_','MarkerSize',10,'LineWidth',1.5)
ylabel('\tau (Nm)')
set(gca,'FontSize',14,'FontName','Arial')
xlabel('time (s)')

sgtitle('Q1c. MPC control','FontSize',16,'FontName','Arial')

%% Q2a. 3D quadruped robot - balancing QP controller
clear; clc;

global params

% Desired state
pd = [0; 0; 0.3];       % x,y,z
dpd = [0;0;0];          % dx,dy,dz
Rd = eul2rotm([0 0 0]); % roll,pitch,yaw
wd = [0;0;0];

params.pd = pd;
params.dpd = dpd;
params.Rd = Rd;
params.wd = wd;

% Constraints
mu = 0.5;
params.mu = mu;

% Robot parameters
a = 0.15;       % m
b = 0.5;
c = 0.3;
mass = 10;      % kg
g = [0;0;-9.81];
params.a = a;
params.b = b;
params.c = c;
params.mass = mass;
params.g = g;

% Moment of inertia equations
Ixb = (1/12)*mass*(a^2+c^2);
Iyb = (1/12)*mass*(a^2+b^2);
Izb = (1/12)*mass*(b^2+c^2);
Ib = [Ixb 0 0; 0 Iyb 0; 0 0 Izb];
params.Ib = Ib;

% Foot positions
pf1 = [0.25; 0.15; 0];
pf2 = [0.25; -0.15; 0];
pf3 = [-0.25; 0.15; 0];
pf4 = [-0.25; -0.15; 0];
params.pf1 = pf1;
params.pf2 = pf2;
params.pf3 = pf3;
params.pf4 = pf4;

% Initial conditions
p0 = [0.05; 0.05; 0.2];
R0 = eul2rotm([pi/4, pi/8, pi/6]);
v0 = [0; 0; 0];
w0 = [0; 0; 0];

% Get R into 9x1 vector
R0_v(1:3,1) = R0(1,:)';
R0_v(4:6,1) = R0(2,:)';
R0_v(7:9,1) = R0(3,:)';
x0 = [p0; R0_v; v0; w0];

% Hand tune parameters
kp_p = diag([30,30,400]);
kd_p = diag([10,10,50]);
kp_o = diag([10,10,10]);
kd_o = diag([5,5,5]);
alpha = 0.01;
S = diag([2,2,10,1,2,1]);
params.kp_p = kp_p;
params.kp_o = kp_o;
params.kd_p = kd_p;
params.kd_o = kd_o;
params.alpha = alpha;
params.S = S;

% Run simulation
tspan=[0; 2]; % simulation time (s)
params.controller = 'QP';
[t,x]=ode45(@sys_dynamics_quad,tspan,x0);

% Plots of quadruped COM and body orientation
quad_plots(x, t, 'Q2a. QP Balancing')

%% Q2a. Animate robot motion
anim_3D(t,x,1/24,'HW4_Q2a');

%% Q2b. 3D quadruped robot - balancing MPC controller 
clearvars N dt tspan t x

N = 10;
dt = 40/1000;
params.N = N;
params.dt = dt;

% Desired state
theta_zyx = rotm2eul(Rd);
thetad = [theta_zyx(3); theta_zyx(2); theta_zyx(1)];
Xd = [pd; thetad; dpd; wd; -9.81];
params.Xd = Xd;

% Run simulation
tspan = 0:dt:2; % simulation time (s)
params.controller = 'MPC';
[t,x]=ode45(@sys_dynamics_quad,tspan,x0);

% Plots of quadruped COM and body orientation
quad_plots(x, t, 'Q2b. MPC Balancing')

%% Q2b. Animate robot motion
anim_3D(t,x,1/24,'HW4_Q2b');

%% Q2c. 3D quadruped robot - trotting MPC controller 
clearvars N dt tspan t x

N = 10;
dt = 40/1000;
params.N = N;
params.dt = dt;

% Foot positions
pf1 = [0.25; 0.15; 0];
pf2 = [0.25; -0.15; 0];
pf3 = [-0.25; 0.15; 0];
pf4 = [-0.25; -0.15; 0];
params.pf1 = pf1;
params.pf2 = pf2;
params.pf3 = pf3;
params.pf4 = pf4;

% Desired state
theta_zyx = rotm2eul(Rd);
thetad = [theta_zyx(3); theta_zyx(2); theta_zyx(1)];
Xd = [pd; thetad; dpd; wd; -9.81];
params.Xd = Xd;

% Run simulation
tspan = 0:dt:2; % simulation time (s)
params.controller = 'MPC_gait';
params.gait = 'trot';
[t,x]=ode45(@sys_dynamics_quad,tspan,x0);

% Plots of quadruped COM and body orientation
quad_plots(x, t, 'Q2c. MPC Trotting')

% Plot controller output (check timings)
figure;
hold on;
clearvars u

% Reset foot positions
pf1 = [0.25; 0.15; 0];
pf2 = [0.25; -0.15; 0];
pf3 = [-0.25; 0.15; 0];
pf4 = [-0.25; -0.15; 0];
params.pf1 = pf1;
params.pf2 = pf2;
params.pf3 = pf3;
params.pf4 = pf4;

for ii = 1:length(t)
    u(ii,1:12) = MPC_gait(t(ii),x(ii,:)');
end

plot(t,u(:,3))
plot(t,u(:,6))
plot(t,u(:,9))
plot(t,u(:,12))
ylabel('Force (N)')
xlabel('time (s)')
legend({'F1z','F2z','F3z','F4z'})

%% Q2c. Animate robot motion

% Reset foot positions
pf1 = [0.25; 0.15; 0];
pf2 = [0.25; -0.15; 0];
pf3 = [-0.25; 0.15; 0];
pf4 = [-0.25; -0.15; 0];
params.pf1 = pf1;
params.pf2 = pf2;
params.pf3 = pf3;
params.pf4 = pf4;

anim_3D(t,x,1/24,'HW4_Q2c');

%% Q2d. 3D quadruped robot - bounding MPC controller 
clearvars N dt tspan t x

N = 10;
dt = 40/1000;
params.N = N;
params.dt = dt;

% Foot positions
pf1 = [0.25; 0.15; 0];
pf2 = [0.25; -0.15; 0];
pf3 = [-0.25; 0.15; 0];
pf4 = [-0.25; -0.15; 0];
params.pf1 = pf1;
params.pf2 = pf2;
params.pf3 = pf3;
params.pf4 = pf4;

% Desired state
theta_zyx = rotm2eul(Rd);
thetad = [theta_zyx(3); theta_zyx(2); theta_zyx(1)];
Xd = [pd; thetad; dpd; wd; -9.81];
params.Xd = Xd;

% Run simulation
tspan = 0:dt:2; % simulation time (s)
params.controller = 'MPC_gait';
params.gait = 'bound';
[t,x]=ode45(@sys_dynamics_quad,tspan,x0);

% Plots of quadruped COM and body orientation
quad_plots(x, t, 'Q2d. MPC Bounding')

% Plot controller output (check timings)
figure;
hold on;
clearvars u

% Reset foot positions
pf1 = [0.25; 0.15; 0];
pf2 = [0.25; -0.15; 0];
pf3 = [-0.25; 0.15; 0];
pf4 = [-0.25; -0.15; 0];
params.pf1 = pf1;
params.pf2 = pf2;
params.pf3 = pf3;
params.pf4 = pf4;

for ii = 1:length(t)
    u(ii,1:12) = MPC_gait(t(ii),x(ii,:)');
end

plot(t,u(:,3))
plot(t,u(:,6))
plot(t,u(:,9))
plot(t,u(:,12))
ylabel('Force (N)')
xlabel('time (s)')
legend({'F1z','F2z','F3z','F4z'})

%% Q2d. Animate robot motion

% Reset foot positions
pf1 = [0.25; 0.15; 0];
pf2 = [0.25; -0.15; 0];
pf3 = [-0.25; 0.15; 0];
pf4 = [-0.25; -0.15; 0];
params.pf1 = pf1;
params.pf2 = pf2;
params.pf3 = pf3;
params.pf4 = pf4;

anim_3D(t,x,1/24,'HW4_Q2d');

%% Functions

% System dynamics for cart-pole
function dynamics=sys_dynamics(t,state)
    global params

    % Parameters
    M = params.M;
    m = params.m;
    L = params.L;
    g = params.g;

    % Control input
    if contains(params.controller,'LQR') == 1
        u=LQR_controller(t,state);
    elseif contains(params.controller,'QP') == 1
        u=QP_controller(t,state);
    elseif contains(params.controller,'MPC') == 1
        u=MPC_controller(t,state);
    end

    % Organize values from x input where x = [x; theta; dx; dtheta]
    theta = state(2);
    dx = state(3);
    dtheta = state(4);

    % First 2 = [x vel, y vel, dtheta];
    dynamics(1:2,1) = [dx; dtheta];

    % Set up equations for ddx and ddtheta
    ddx = (u-m*L*sin(theta)*dtheta^2+m*g*cos(theta)*sin(theta))./(M+m-m*cos(theta)^2);
    ddtheta = (cos(theta)*ddx+g*sin(theta))./L;

    % Last 2 = [ddx; ddtheta];
    dynamics(3:4,1) = [ddx; ddtheta];
end

% LQR controller
function u = LQR_controller(t,x)
    global params
    k = params.k;
    u = -k*x;
end

% QP controller
function u = QP_controller(t,x)
    H = 1;
    f = -LQR_controller(t,x);
    A = [1; -1];
    b = [10; 10];

    u = quadprog(H,f,A,b);
end

% MPC controller
function u = MPC_controller(t,x) % for loop based on num of horizons
    global params
    Acont = params.A;
    Bcont = params.B;

    % Controller settings
    N = params.N;           % number of horizons
    dt = params.dt;         % sampling time in seconds
    Q = diag([100 0.01 100 0.01]);
    R = 1;
    H = [];
    for ii = 1:N
        H = blkdiag(H,Q);
    end
    for ii = 1:N
        H = blkdiag(H,R);
    end
    f = zeros(100,1);

    % Make continuous --> discrete dynamics
    A = Acont*dt+eye(4);
    B = Bcont*dt;

    % State-dependent constraints
    a1 = [eye(N); -eye(N)];                % torque constraints
    a2 = [zeros(2*N,4*N) a1];
    b1 = 10*ones(N,1);
    b2 = b1;

    a3 = zeros(N,5*N);
    a4 = a3;
    a5 = a3;            
    a6 = a3;
    for row = 1:N
        col_x = 4*row-3;
        col_theta = 4*row-2;
        a3(row,col_x) = 1;      % x constraints
        a4(row,col_x) = -1;
        a5(row,col_theta) = 1;  % theta constraints
        a6(row,col_theta) = -1;
    end
    b3 = 0.8*ones(N,1);         % x constraints
    b4 = b3;
    b5 = (pi/4)*ones(N,1);      % theta constraints
    b6 = b5;

    Astate = [a2; a3; a4; a5; a6];
    bstate = [b1; b2; b3; b4; b5; b6];

    % Equality constraints
    Aeq = zeros(4*N,5*N);               % (4N,5N)
    for ii = 1:N
        Aeq(ii*4-3:ii*4,ii*4-3:ii*4) = eye(4,4);
        if ii > 1
            Aeq(ii*4-3:ii*4,(ii-1)*4-3:(ii-1)*4) = -A;
        end
        Aeq(ii*4-3:ii*4,4*N+ii) = -B;
    end

    beq = [A*x(1:4); zeros(4*(N-1),1)]; % (4N,1)
    
    % Solve
    X = quadprog(H,f,Astate,bstate,Aeq,beq);
    u = X(4*N+1);
end

% System dynamics for 3D quadruped
function dx=sys_dynamics_quad(t,x) % Adapted from HW2
    global params

    % Control input
    if contains(params.controller,'QP') == 1
        u = QP_quad(t,x);
    elseif contains(params.controller,'MPC_gait') == 1
        u = MPC_gait(t,x);
    elseif contains(params.controller,'MPC') == 1
        u = MPC_quad(t,x);
    end

    F1 = u(1:3); % wrt world frame
    F2 = u(4:6);
    F3 = u(7:9);
    F4 = u(10:12);

    % Organize values from x input where x = [p; R; dp; wb]
    p = x(1:3);
    R(1,1:3) = x(4:6);
    R(2,1:3) = x(7:9);
    R(3,1:3) = x(10:12);
    dp = x(13:15);
    wb = x(16:18);

    % Get forces into body frame
    F1b = transpose(R)*F1;
    F2b = transpose(R)*F2;
    F3b = transpose(R)*F3;
    F4b = transpose(R)*F4;
    
    % First 3 = linear velocities (dp)
    dx(1:3,1) = dp;

    % Derivative of Rotation matrix dR = R*S(wb)
    S = [0 -wb(3) wb(2);
         wb(3) 0 -wb(1);
         -wb(2) wb(1) 0];
    dR = R*S;

    % Next 9 = dR in 9x1 vector
    dx(4:12,1) = [dR(1,:)'; dR(2,:)'; dR(3,:)'];

    % Linear accelerations (ddp)
    F = F1+F2+F3+F4;
    ddp = F./params.mass + params.g; % F = ma

    % Next 3 = ddp
    dx(13:15,1) = ddp;
    
    % Angular accelerations (dwb)
    r1b = transpose(R)*(params.pf1 - p); % ri = R*rb where ri = pf-p
    r2b = transpose(R)*(params.pf2 - p);
    r3b = transpose(R)*(params.pf3 - p);
    r4b = transpose(R)*(params.pf4 - p);

    M1b = cross(r1b,F1b);   % M = rXF
    M2b = cross(r2b,F2b);
    M3b = cross(r3b,F3b);
    M4b = cross(r4b,F4b);
    Mb = M1b + M2b + M3b + M4b; % sum of all moments

    dwb = params.Ib\(Mb - cross(wb,params.Ib*wb)); % I*dwb + wb X (I*wb) = Mb

    % Last 3 = dwb
    dx(16:18,1) = dwb;
end

% QP controller (3D quadruped)
function F = QP_quad(t,x)
    global params
    pd = params.pd;
    dpd = params.dpd;
    wd = params.wd;
    Rd = params.Rd;
    m = params.mass;
    Ib = params.Ib;
    kp_p = params.kp_p;
    kd_p = params.kd_p;
    kp_o = params.kp_o;
    kd_o = params.kd_o;
    mu = params.mu;
    S = params.S;
    alpha = params.alpha;
    g = [0; 0; -9.81];

    % x = [x;y;z;R;dx;dy;dz;w]
    p(1:3,1) = [x(1); x(2); x(3)];
    dp(1:3,1) = [x(13); x(14); x(15)];
    ddp = kp_p*(pd-p) + kd_p*(dpd-dp);

    % Rotation matrix
    R(1,1:3) = x(4:6);
    R(2,1:3) = x(7:9);
    R(3,1:3) = x(10:12);

    e = rotmat2vec3d(Rd*transpose(R));
    wb(1:3,1) = [x(16); x(17); x(18)];
    w = R*wb;
    dwd = kp_o*e' + kd_o*(wd-w);

    p = x(1:3);
    r1 = params.pf1 - p; % ri = pf-p
    r2 = params.pf2 - p;
    r3 = params.pf3 - p;
    r4 = params.pf4 - p;
 
    I = R*Ib*transpose(R);
    bd = [m.*ddp+m.*g; I*dwd];
    A = [eye(3) eye(3) eye(3) eye(3);
        0 -r1(3) r1(2) 0 -r2(3) r2(2) 0 -r3(3) r3(2) 0 -r4(3) r4(2);
        r1(3) 0 -r1(1) r2(3) 0 -r2(1) r3(3) 0 -r3(1) r4(3) 0 -r4(1);
        -r1(2) r1(1) 0 -r2(2) r2(1) 0 -r3(2) r3(1) 0 -r4(2) r4(1) 0];
    
    H = transpose(A)*S*A+alpha;
    f = transpose(-transpose(bd)*S*A);

    % Inequality constraints
    A1_Fz = [0 0 1];
    A1 = blkdiag(A1_Fz,A1_Fz,A1_Fz,A1_Fz);
    A2 = -A1;
    A3_Fx = [1 0 -mu];
    A3 = blkdiag(A3_Fx,A3_Fx,A3_Fx,A3_Fx);
    A4_Fx = [-1 0 -mu];
    A4 = blkdiag(A4_Fx,A4_Fx,A4_Fx,A4_Fx);
    A5_Fx = [0 1 -mu];
    A5 = blkdiag(A5_Fx,A5_Fx,A5_Fx,A5_Fx);
    A6_Fx = [0 -1 -mu];
    A6 = blkdiag(A6_Fx,A6_Fx,A6_Fx,A6_Fx);
    Aqp = [A1; A2; A3; A4; A5; A6];
    bqp = [500*ones(4,1); -10*ones(4,1); zeros(16,1)];

    F = quadprog(H,f,Aqp,bqp);
end

% MPC controller
function u = MPC_quad(t,x)
    global params

    % Rearrange to get x_bar
    p = x(1:3);
    R(1,1:3) = x(4:6);
    R(2,1:3) = x(7:9);
    R(3,1:3) = x(10:12);
    theta_zyx = rotm2eul(R);
    theta = [theta_zyx(3); theta_zyx(2); theta_zyx(1)];
    dp = x(13:15);
    wb = x(16:18);
    w = R*wb;

    % Control input
    x_bar = [p; theta; dp; w; -9.81];

    % Controller settings
    N = params.N;           % number of horizons
    dt = params.dt;         % sampling time in seconds
    Q = diag([40 50 60 10 10 10 4 4 4 1 1 1 0]);
    Rmpc = 0.001*eye(12);
    H = [];
    for ii = 1:N
        H = blkdiag(H,Q);
    end
    for ii = 1:N
        H = blkdiag(H,Rmpc);
    end
    f1 = transpose(params.Xd)*Q;
    f2 = zeros(1,12);
    f = [];
    for ii = 1:N
        f = [f f1];
    end
    for ii = 1:N
        f = [f f2];
    end
    f = -transpose(f);

    % dtheta
    yaw = theta(3);
    T = [cos(yaw)*cos(0), -sin(yaw), 0;
         sin(yaw)*cos(0), cos(yaw), 0;
         -sin(0), 0, 1];

    % r vectors in world frame
    r1 = params.pf1 - p; % ri = pf-p
    r2 = params.pf2 - p;
    r3 = params.pf3 - p;
    r4 = params.pf4 - p;

    % Inertia in world frame
    I = R*params.Ib*transpose(R);

    % Make continuous --> discrete dynamics: dX = AX + Bu
    Acont = [zeros(3,6) eye(3) zeros(3,4);
             zeros(3,9) inv(T) zeros(3,1);
             zeros(2,13);
             zeros(1,12) 1;
             zeros(4,13)]; % (13x13)

    r1x = [0, -r1(3), r1(2);
           r1(3), 0, -r1(1);
           -r1(2), r1(1), 0];
    r2x = [0, -r2(3), r2(2);
           r2(3), 0, -r2(1);
           -r2(2), r2(1), 0];
    r3x = [0, -r3(3), r3(2);
           r3(3), 0, -r3(1);
           -r3(2), r3(1), 0];
    r4x = [0, -r4(3), r4(2);
           r4(3), 0, -r4(1);
           -r4(2), r4(1), 0];

    Bcont = [zeros(6,12);
             eye(3)./params.mass eye(3)./params.mass eye(3)./params.mass eye(3)./params.mass;
             I\r1x, I\r2x, I\r3x, I\r4x;
             zeros(1,12)]; % (13x12)

    A = Acont.*dt+eye(13);
    B = Bcont.*dt;

    % State-dependent constraints: AX <= b
    a = [0 0 1];                % Fz constraints
    a1 = blkdiag(a,a,a,a);
    b = [500; 500; 500; 500];
    A1 = []; b1 = [];
    for ii = 1:N
        A1 = blkdiag(A1,a1);
        b1 = [b1; b];
    end
    A1 = [zeros(4*N,13*N), A1]; % no x constraints
    A2 = -A1;
    b2 = -10*ones(length(b1),1);

    clearvars a b
    a = [1 0 -params.mu];       % Fx/Fz constraints
    aa = [-1 0 -params.mu];
    a3 = blkdiag(a,a,a,a);
    a4 = blkdiag(aa,aa,aa,aa);
    b = [0; 0; 0; 0];
    A3 = []; A4 = []; b3 = [];
    for ii = 1:N
        A3 = blkdiag(A3,a3);
        A4 = blkdiag(A4,a4);
        b3 = [b3; b];
    end
    A3 = [zeros(4*N,13*N), A3]; % no x constraints
    A4 = [zeros(4*N,13*N), A4];
    b4 = b3;

    clearvars a aa
    a = [0 1 -params.mu];       % Fy/Fz constraints
    aa = [0 -1 -params.mu];
    a5 = blkdiag(a,a,a,a);
    a6 = blkdiag(aa,aa,aa,aa);
    A5 = []; A6 = [];
    for ii = 1:N
        A5 = blkdiag(A5,a5);
        A6 = blkdiag(A6,a6);
    end
    A5 = [zeros(4*N,13*N), A5]; % no x constraints
    A6 = [zeros(4*N,13*N), A6];
    b5 = b3;
    b6 = b3;

    Astate = [A1; A2; A3; A4; A5; A6];
    bstate = [b1; b2; b3; b4; b5; b6];

    % Equality constraints
    Aeq = zeros(13*N,25*N);               % (13N,25N)
    for ii = 1:N
        Aeq(13*(ii-1)+1:13*ii,13*(ii-1)+1:13*ii) = eye(13,13);
        if ii > 1
            Aeq(13*(ii-1)+1:13*ii,13*(ii-2)+1:13*(ii-1)) = -A;
        end
        Aeq(13*(ii-1)+1:13*ii,13*N+12*(ii-1)+1:13*N+12*ii) = -B;
    end

    beq = [A*x_bar; zeros(13*(N-1),1)]; % (13N,1)
    
    % Solve
    X = quadprog(H,f,Astate,bstate,Aeq,beq);

    u = X(13*N+1:13*N+12);
end

% MPC gait controller
function u = MPC_gait(t,x)
    global params

    % Rearrange to get x_bar
    p = x(1:3);
    R(1,1:3) = x(4:6);
    R(2,1:3) = x(7:9);
    R(3,1:3) = x(10:12);
    theta_zyx = rotm2eul(R);
    theta = [theta_zyx(3); theta_zyx(2); theta_zyx(1)];
    dp = x(13:15);
    wb = x(16:18);
    w = R*wb;

    % Control input
    x_bar = [p; theta; dp; w; -9.81];

    % Controller settings
    N = params.N;           % number of horizons
    dt = params.dt;         % sampling time in seconds
    if contains(params.gait,'bound') == 1
        Q = diag([500 100 5000 35 30 60 10 10 4 1 1 1 0]);
        Rmpc = 0.00001*eye(12);
    else
        Q = diag([40 50 60 10 10 10 4 4 4 1 1 1 0]);
        Rmpc = 0.001*eye(12);
    end
    H = [];
    for ii = 1:N
        H = blkdiag(H,Q);
    end
    for ii = 1:N
        H = blkdiag(H,Rmpc);
    end
    f1 = transpose(params.Xd)*Q;
    f2 = zeros(1,12);
    f = [];
    for ii = 1:N
        f = [f f1];
    end
    for ii = 1:N
        f = [f f2];
    end
    f = -transpose(f);

    % dtheta
    yaw = theta(3);
    T = [cos(yaw)*cos(0), -sin(yaw), 0;
         sin(yaw)*cos(0), cos(yaw), 0;
         -sin(0), 0, 1];

    % Inertia in world frame
    I = R*params.Ib*transpose(R);

    % Make continuous --> discrete dynamics: dX = AX + Bu
    Acont = [zeros(3,6) eye(3) zeros(3,4);
             zeros(3,9) inv(T) zeros(3,1);
             zeros(2,13);
             zeros(1,12) 1;
             zeros(4,13)]; % (13x13)

    % Get current time to determine gait phase
    time = floor(t/dt)*dt;
    if mod(time/dt,N) == 0                      % divisible by N (or =0)
        contact = [ones(1,N/2) zeros(1,N/2)];   % start at stance
        if contains(params.gait,'trot') == 1
            params.pf1 = [p(1)+params.b/2; p(2)+params.c/2; 0];
            params.pf4 = [p(1)-params.b/2; p(2)-params.c/2; 0];
        elseif contains(params.gait,'bound') == 1
            params.pf1 = [p(1)+params.b/2; p(2)+params.c/2; 0];
            params.pf2 = [p(1)+params.b/2; p(2)-params.c/2; 0];
        end
    elseif mod(time/dt,N) == N/2
        contact = [zeros(1,N/2) ones(1,N/2)];
        if contains(params.gait,'trot') == 1
            params.pf2 = [p(1)+params.b/2; p(2)-params.c/2; 0];
            params.pf3 = [p(1)-params.b/2; p(2)+params.c/2; 0];
        elseif contains(params.gait,'bound') == 1
            params.pf3 = [p(1)-params.b/2; p(2)+params.c/2; 0];
            params.pf4 = [p(1)-params.b/2; p(2)-params.c/2; 0];
        end
    elseif mod(time/dt,N) < N/2
        offset = floor(mod(time/dt,N));
        contact = [ones(1,N/2-offset) zeros(1,N/2) ones(1,offset)];
    elseif mod(time/dt,N) > N/2
        offset = N/2 - floor(mod(time/dt,N));
        contact = [zeros(1,N/2-offset) ones(1,N/2) zeros(1,offset)];
    end

    % r vectors in world frame
    r1 = params.pf1 - p; % ri = pf-p
    r2 = params.pf2 - p;
    r3 = params.pf3 - p;
    r4 = params.pf4 - p;

    r1x = [0, -r1(3), r1(2);
           r1(3), 0, -r1(1);
           -r1(2), r1(1), 0];
    r2x = [0, -r2(3), r2(2);
           r2(3), 0, -r2(1);
           -r2(2), r2(1), 0];
    r3x = [0, -r3(3), r3(2);
           r3(3), 0, -r3(1);
           -r3(2), r3(1), 0];
    r4x = [0, -r4(3), r4(2);
           r4(3), 0, -r4(1);
           -r4(2), r4(1), 0];

    Bcont = [zeros(6,12);
             eye(3)./params.mass eye(3)./params.mass eye(3)./params.mass eye(3)./params.mass;
             I\r1x, I\r2x, I\r3x, I\r4x;
             zeros(1,12)]; % (13x12)

    A = Acont.*dt+eye(13);
    B = Bcont.*dt;

    % State-dependent constraints: AX <= b
    a = [0 0 1];                % Fz constraints
    a1 = blkdiag(a,a,a,a);
    A1 = []; b1 = []; b2 = [];
    for ii = 1:N
        clearvars b bb
        if contact(ii) == 0
            if contains(params.gait,'trot') == 1
                b(1,1) = 0;
                b(2,1) = 500;
                b(3,1) = 500;
                b(4,1) = 0;
            elseif contains(params.gait,'bound') == 1
                b(1,1) = 0;
                b(2,1) = 0;
                b(3,1) = 500;
                b(4,1) = 500;
            end
            bb = b./(-50);
        else
            if contains(params.gait,'trot') == 1
                b(1,1) = 500;
                b(2,1) = 0;
                b(3,1) = 0;
                b(4,1) = 500;
            elseif contains(params.gait,'bound') == 1
                b(1,1) = 500;
                b(2,1) = 500;
                b(3,1) = 0;
                b(4,1) = 0;
            end
            bb = b./(-50);
        end
        A1 = blkdiag(A1,a1);
        b1 = [b1; b];
        b2 = [b2; bb];
    end
    A1 = [zeros(4*N,13*N), A1]; % no x constraints
    A2 = -A1;

    clearvars a b
    a = [1 0 -params.mu];       % Fx/Fz constraints
    aa = [-1 0 -params.mu];
    a3 = blkdiag(a,a,a,a);
    a4 = blkdiag(aa,aa,aa,aa);
    b = [0; 0; 0; 0];
    A3 = []; A4 = []; b3 = [];
    for ii = 1:N
        A3 = blkdiag(A3,a3);
        A4 = blkdiag(A4,a4);
        b3 = [b3; b];
    end
    A3 = [zeros(4*N,13*N), A3]; % no x constraints
    A4 = [zeros(4*N,13*N), A4];
    b4 = b3;

    clearvars a aa
    a = [0 1 -params.mu];       % Fy/Fz constraints
    aa = [0 -1 -params.mu];
    a5 = blkdiag(a,a,a,a);
    a6 = blkdiag(aa,aa,aa,aa);
    A5 = []; A6 = [];
    for ii = 1:N
        A5 = blkdiag(A5,a5);
        A6 = blkdiag(A6,a6);
    end
    A5 = [zeros(4*N,13*N), A5]; % no x constraints
    A6 = [zeros(4*N,13*N), A6];
    b5 = b3;
    b6 = b3;

    Astate = [A1; A2; A3; A4; A5; A6];
    bstate = [b1; b2; b3; b4; b5; b6];

    % Equality constraints
    Aeq = zeros(13*N,25*N);               % (13N,25N)
    for ii = 1:N
        Aeq(13*(ii-1)+1:13*ii,13*(ii-1)+1:13*ii) = eye(13,13);
        if ii > 1
            Aeq(13*(ii-1)+1:13*ii,13*(ii-2)+1:13*(ii-1)) = -A;
        end
        Aeq(13*(ii-1)+1:13*ii,13*N+12*(ii-1)+1:13*N+12*ii) = -B;
    end

    beq = [A*x_bar; zeros(13*(N-1),1)]; % (13N,1)
    
    % Solve
    X = quadprog(H,f,Astate,bstate,Aeq,beq);

    u = X(13*N+1:13*N+12);

end

function anim_3D(t,x,ts,filename)
    [te,xe]=even_sample(t,x,1/ts); % evenly sample for video frames

    figure;
            
    % Save as a video
    spwriter = VideoWriter(filename,'MPEG-4');
    set(spwriter, 'FrameRate', 1/ts,'Quality',100);
    open(spwriter);
    
    % Set limits
    axes1 = axes;

    if contains(filename,'HW4_Q2d') == 1
        figure_x_limits = [-0.7 1];
        figure_y_limits = [-0.5 0.5];
        figure_z_limits = [-0.1 0.85];
    else
        figure_x_limits = [-0.7 0.7];
        figure_y_limits = [-0.5 0.5];
        figure_z_limits = [-0.1 0.5];
    end
    set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits,'ZLim',figure_z_limits)

    % Plot settings
    set(axes1,'Color','w');
    xlabel('x (m)','FontSize',16,'FontName','Arial')
    ylabel('y (m)','FontSize',16,'FontName','Arial')
    zlabel('z (m)','FontSize',16,'FontName','Arial')
    view(3)
    axis('equal')
    grid on
    set(gca,'FontSize',16,'FontName','Arial')
    
    % Loop through the time and plot each data point at a time
    for k = 1:length(te)
        drawone(axes1, xe(k,:)', filename, te(k));
        set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits,'ZLim',figure_z_limits);
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

function drawone(parent, x, filename, t)
    global params
    % Delete existing plot
    tem = get(parent,'Children');
    delete(tem);

    % Draw the robot at the current frame

    hold on;
    % COM position
    px = x(1);
    py = x(2);
    pz = x(3);
    p = [px;py;pz];

    % Recreate controller input
    if filename == 'HW4_Q2a'
        u = QP_quad([],x);
    elseif filename == 'HW4_Q2b'
        u = MPC_quad([],x);
    else
        u = MPC_gait(t,x);
    end
    F1 = u(1:3); % wrt world frame
    F2 = u(4:6);
    F3 = u(7:9);
    F4 = u(10:12);

    % Rotation matrix from input
    R(1,1:3) = x(4:6); % rotation matrix
    R(2,1:3) = x(7:9);
    R(3,1:3) = x(10:12);

    % Get forces in body frame
    F1b = transpose(R)*F1;
    F2b = transpose(R)*F2;
    F3b = transpose(R)*F3;
    F4b = transpose(R)*F4;

    % Four points representing feet
    plot3(params.pf1(1),params.pf1(2),params.pf1(3),'ok','MarkerFaceColor','k','MarkerSize',10)
    plot3(params.pf2(1),params.pf2(2),params.pf2(3),'ok','MarkerFaceColor','k','MarkerSize',10)
    plot3(params.pf3(1),params.pf3(2),params.pf3(3),'ok','MarkerFaceColor','k','MarkerSize',10)
    plot3(params.pf4(1),params.pf4(2),params.pf4(3),'ok','MarkerFaceColor','k','MarkerSize',10)
    
    % Plot normalized vectors
    plot3([params.pf1(1) params.pf1(1)+F1b(1)/100],[params.pf1(2) params.pf1(2)+F1b(2)/100],[params.pf1(3) params.pf1(3)+F1b(3)/100],'-r')
    plot3([params.pf2(1) params.pf2(1)+F2b(1)/100],[params.pf2(2) params.pf2(2)+F2b(2)/100],[params.pf2(3) params.pf2(3)+F2b(3)/100],'-r')
    plot3([params.pf3(1) params.pf3(1)+F3b(1)/100],[params.pf3(2) params.pf3(2)+F3b(2)/100],[params.pf3(3) params.pf3(3)+F3b(3)/100],'-r')
    plot3([params.pf4(1) params.pf4(1)+F4b(1)/100],[params.pf4(2) params.pf4(2)+F4b(2)/100],[params.pf4(3) params.pf4(3)+F4b(3)/100],'-r')

    % Define robot body vertices in body frame
    v1b = [-params.b/2; -params.c/2; params.a/2];
    v2b = v1b + [0;params.c;0];
    v3b = v1b + [0;0;-params.a];
    v4b = v1b + [0;params.c;-params.a];
    v5b = v1b + [params.b;0;0];
    v6b = v1b + [params.b;params.c;0];
    v7b = v1b + [params.b;0;-params.a];
    v8b = v1b + [params.b;params.c;-params.a];

    % Transform robot body vertices into world frame
    v1 = R*v1b + p;
    v2 = R*v2b + p;
    v3 = R*v3b + p;
    v4 = R*v4b + p;
    v5 = R*v5b + p;
    v6 = R*v6b + p;
    v7 = R*v7b + p;
    v8 = R*v8b + p;

    % Plot robot body box lines
    line1 = [v1 v2];
    plot3(line1(1,:),line1(2,:),line1(3,:),'k-')
    line2 = [v1 v3];
    plot3(line2(1,:),line2(2,:),line2(3,:),'k-')
    line3 = [v2 v4];
    plot3(line3(1,:),line3(2,:),line3(3,:),'k-')
    line4 = [v3 v4];
    plot3(line4(1,:),line4(2,:),line4(3,:),'k-')

    line1 = [v5 v6];
    plot3(line1(1,:),line1(2,:),line1(3,:),'k-')
    line2 = [v5 v7];
    plot3(line2(1,:),line2(2,:),line2(3,:),'k-')
    line3 = [v6 v8];
    plot3(line3(1,:),line3(2,:),line3(3,:),'k-')
    line4 = [v7 v8];
    plot3(line4(1,:),line4(2,:),line4(3,:),'k-')

    line1 = [v1 v5];
    plot3(line1(1,:),line1(2,:),line1(3,:),'k-')
    line2 = [v2 v6];
    plot3(line2(1,:),line2(2,:),line2(3,:),'k-')
    line3 = [v3 v7];
    plot3(line3(1,:),line3(2,:),line3(3,:),'k-')
    line4 = [v4 v8];
    plot3(line4(1,:),line4(2,:),line4(3,:),'k-')

    % Plot COM position
    plot3(p(1),p(2),p(3),'bo','MarkerSize',8,'MarkerFaceColor','b');  
end

% Plot figures
function quad_plots(x, t, Q_name)
    clearvars euler
    % Plot COM position
    figure; hold on;
    subplot(1,2,1)
    plot3(x(:,1),x(:,2),x(:,3),'ko')
    
    % Add vertices as text
    labels = {'Start','End'};
    text(x(1,1),x(1,2),x(1,3),labels{1},'VerticalAlignment','bottom','FontSize',16,'Color','b')
    text(x(end,1),x(end,2),x(end,3),labels{2},'VerticalAlignment','bottom','FontSize',16,'Color','b')
    
    % Plot settings
    xlabel('x')
    ylabel('y')
    zlabel('z')
    view(3)
    axis('equal')
    grid on
    
    title('COM position over time')
    set(gca,'FontName','Arial','FontSize',16)
    
    % Plot body orientation's Euler angles
    for ii = 1:size(x,1)
        clearvars R_plot
        R_plot(1,1:3) = x(ii,4:6);
        R_plot(2,1:3) = x(ii,7:9);
        R_plot(3,1:3) = x(ii,10:12);
        angles = rotm2eul(R_plot);
        euler(ii,1:3) = [angles(3) angles(2) angles(1)];
    end
    subplot(1,2,2); hold on;
    plot3(euler(:,1),euler(:,2),euler(:,3),'ko')
    
    % Add vertices as text
    labels = {'Start','End'};
    text(euler(1,1),euler(1,2),euler(1,3),labels{1},'VerticalAlignment','top','FontSize',16,'Color','b')
    text(euler(end,1),euler(end,2),euler(end,3),labels{2},'VerticalAlignment','top','FontSize',16,'Color','b')
    
    % Plot settings
    xlabel('Roll')
    ylabel('Pitch')
    zlabel('Yaw')
    view(3)
    axis('equal')
    grid on
    
    title('Body orientation over time')
    sgtitle(Q_name,'FontName','Arial','FontSize',20,'FontWeight','Bold')
    set(gca,'FontName','Arial','FontSize',16)
    
    % Plots in 2D
    figure;
    subplot(3,2,1)
    plot(t,x(:,1),'LineWidth',1.5)
    ylabel('X (m)','FontSize',16,'FontName','Arial')
    title('COM position','FontSize',16,'FontName','Arial','FontWeight','Bold')
    
    subplot(3,2,3)
    plot(t,x(:,2),'LineWidth',1.5)
    ylabel('Y (m)','FontSize',16,'FontName','Arial')
    
    subplot(3,2,5)
    plot(t,x(:,3),'LineWidth',1.5)
    ylabel('Z (m)','FontSize',16,'FontName','Arial')
    xlabel('time (s)','FontSize',16,'FontName','Arial')
    
    subplot(3,2,2)
    plot(t,euler(:,1),'LineWidth',1.5)
    ylabel('Roll (rad)','FontSize',16,'FontName','Arial')
    title('Body Orientation','FontSize',16,'FontName','Arial','FontWeight','Bold')
    
    subplot(3,2,4)
    plot(t,euler(:,2),'LineWidth',1.5)
    ylabel('Pitch (rad)','FontSize',16,'FontName','Arial')
    
    subplot(3,2,6)
    plot(t,euler(:,3),'LineWidth',1.5)
    ylabel('Yaw (rad)','FontSize',16,'FontName','Arial')
    xlabel('time (s)','FontSize',16,'FontName','Arial')
    
    sgtitle(Q_name,'FontName','Arial','FontSize',20,'FontWeight','Bold')
end
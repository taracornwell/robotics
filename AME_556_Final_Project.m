%% Final Project
% Unitree A1 robot

%% Tasks 1 and 2: Walking and Running
clear; clc;
% Task 1:
% standing (0 = QP,0.5 = MPC): a1
% walking (1 = forwards, 2 = back, 3 = left, 4 = right): a1, a1, a1_sideways
% rotating (5 = left, 6 = right): a1

% Task 2: Running (7): a1

gait_type = 7;
PD = 0;
% speed = 0.65; % forwards
% speed = 0.625; % backwards
% speed = 0.5; % rotating/sideways
speed = 0.75; % running

% Original robot height
z0 = 0.2855;
zd = 0.2855;
mu = 0.5;

% Set initial joint angles
hip0 = 0;
thigh0 = 0.95;
calf0 = -1.6;

% Set desired joint angles (desired velocities=0)
hipd = 0;
thighd = 0.95;
calfd = -1.6;
kp = 25;
kd = 4;

% Controller settings
if gait_type < 1
    Q = diag([100 500 2000 9000 6000 1000 100 100 300 500 500 500 0]); % standing
elseif gait_type == 1
    Q = diag([0 800 15000 49500 9000 7000 1500 700 100 900 300 200 0]); % forwards (Prof. said to focus on x vel and not x pos during walking)
elseif gait_type == 2
    Q = diag([0 800 12000 40000 12000 1900 1600 800 300 500 500 500 0]); % backwards
elseif gait_type == 3 || gait_type == 4
    Q = diag([100 0 50 11000 150 150 10 300 10 10 1 1 0]); % left/right
elseif gait_type == 5 || gait_type == 6
    Q = diag([1000 1000 1000 1000 1000 1000 500 500 500 500 500 15000 0]); % rotate left/right
elseif gait_type == 7
    Q = diag([5 800 15000 48000 9000 7000 1500 700 100 900 300 200 0]); % running
end
Rmpc = 0.000001*eye(12);
N = 10;
dt = 0.03;
Tstance = 0.15;

H = [];
for ii = 1:N
    H = blkdiag(H,Q);
end
for ii = 1:N
    H = blkdiag(H,Rmpc);
end

% State-dependent constraints: AX <= b
a = [0 0 1];                % Fz constraints
aa = blkdiag(a,a,a,a);
A1 = [];
for ii = 1:N
    A1 = blkdiag(A1,aa);
end
A1 = [zeros(4*N,13*N), A1]; % no x constraints
A2 = -A1;

clearvars a b
a = [1 0 -mu];       % Fx/Fz constraints
aa = [-1 0 -mu];
a3 = blkdiag(a,a,a,a);
a4 = blkdiag(aa,aa,aa,aa);
b = [0; 0; 0; 0];
A3 = []; A4 = []; b3 = [];
for ii = 1:N
    A3 = blkdiag(A3,a3);
    A4 = blkdiag(A4,a4);
end
A3 = [zeros(4*N,13*N), A3]; % no x constraints
A4 = [zeros(4*N,13*N), A4];

clearvars a aa
a = [0 1 -mu];       % Fy/Fz constraints
aa = [0 -1 -mu];
a5 = blkdiag(a,a,a,a);
a6 = blkdiag(aa,aa,aa,aa);
A5 = []; A6 = [];
for ii = 1:N
    A5 = blkdiag(A5,a5);
    A6 = blkdiag(A6,a6);
end
A5 = [zeros(4*N,13*N), A5]; % no x constraints
A6 = [zeros(4*N,13*N), A6];

Astate = [A1; A2; A3; A4; A5; A6];

% Initialize gait table
tbl0 = ones(N*4,1);

%% Task 3: Stairclimbing
gait_type = 1;
PD = 0;
speed = 0.55;

% Original robot height
z0 = 0.2855;
zd = 0.2855;
mu = 0.5;

% Set initial joint angles
hip0 = 0;
thigh0 = 0.95;
calf0 = -1.6;

% Set desired joint angles (desired velocities=0)
hipd = 0;
thighd = 0.95;
calfd = -1.6;
kp = 25;
kd = 4;

% Controller settings
Q = diag([0 800 11000 49500 7000 7000 1800 700 240 900 900 200 0]); % forwards
Rmpc = 0.000001*eye(12);
N = 10;
dt = 0.03;
Tstance = 0.15;

H = [];
for ii = 1:N
    H = blkdiag(H,Q);
end
for ii = 1:N
    H = blkdiag(H,Rmpc);
end

% State-dependent constraints: AX <= b
a = [0 0 1];                % Fz constraints
aa = blkdiag(a,a,a,a);
A1 = [];
for ii = 1:N
    A1 = blkdiag(A1,aa);
end
A1 = [zeros(4*N,13*N), A1]; % no x constraints
A2 = -A1;

clearvars a b
a = [1 0 -mu];       % Fx/Fz constraints
aa = [-1 0 -mu];
a3 = blkdiag(a,a,a,a);
a4 = blkdiag(aa,aa,aa,aa);
b = [0; 0; 0; 0];
A3 = []; A4 = []; b3 = [];
for ii = 1:N
    A3 = blkdiag(A3,a3);
    A4 = blkdiag(A4,a4);
end
A3 = [zeros(4*N,13*N), A3]; % no x constraints
A4 = [zeros(4*N,13*N), A4];

clearvars a aa
a = [0 1 -mu];       % Fy/Fz constraints
aa = [0 -1 -mu];
a5 = blkdiag(a,a,a,a);
a6 = blkdiag(aa,aa,aa,aa);
A5 = []; A6 = [];
for ii = 1:N
    A5 = blkdiag(A5,a5);
    A6 = blkdiag(A6,a6);
end
A5 = [zeros(4*N,13*N), A5]; % no x constraints
A6 = [zeros(4*N,13*N), A6];

Astate = [A1; A2; A3; A4; A5; A6];

% Initialize gait table
tbl0 = ones(N*4,1);

%% Task 4: Obstacle course
gait_type = 1;
PD = 0;
speed = 0.37;

% Original robot height
z0 = 0.2855;
zd = 0.31;
mu = 0.5;

% Set initial joint angles
hip0 = 0;
thigh0 = 0.95;
calf0 = -1.6;

% Set desired joint angles (desired velocities=0)
hipd = 0;
thighd = 0.95;
calfd = -1.6;
kp = 25;
kd = 4;

% Controller settings
Q = diag([0 800 15000 49500 14000 5000 1900 700 700 900 700 200 0]); % forwards
Rmpc = 0.000001*eye(12);
N = 10;
dt = 0.03;
Tstance = 0.15;

H = [];
for ii = 1:N
    H = blkdiag(H,Q);
end
for ii = 1:N
    H = blkdiag(H,Rmpc);
end

% State-dependent constraints: AX <= b
a = [0 0 1];                % Fz constraints
aa = blkdiag(a,a,a,a);
A1 = [];
for ii = 1:N
    A1 = blkdiag(A1,aa);
end
A1 = [zeros(4*N,13*N), A1]; % no x constraints
A2 = -A1;

clearvars a b
a = [1 0 -mu];       % Fx/Fz constraints
aa = [-1 0 -mu];
a3 = blkdiag(a,a,a,a);
a4 = blkdiag(aa,aa,aa,aa);
b = [0; 0; 0; 0];
A3 = []; A4 = []; b3 = [];
for ii = 1:N
    A3 = blkdiag(A3,a3);
    A4 = blkdiag(A4,a4);
end
A3 = [zeros(4*N,13*N), A3]; % no x constraints
A4 = [zeros(4*N,13*N), A4];

clearvars a aa
a = [0 1 -mu];       % Fy/Fz constraints
aa = [0 -1 -mu];
a5 = blkdiag(a,a,a,a);
a6 = blkdiag(aa,aa,aa,aa);
A5 = []; A6 = [];
for ii = 1:N
    A5 = blkdiag(A5,a5);
    A6 = blkdiag(A6,a6);
end
A5 = [zeros(4*N,13*N), A5]; % no x constraints
A6 = [zeros(4*N,13*N), A6];

Astate = [A1; A2; A3; A4; A5; A6];

% Initialize gait table
tbl0 = ones(N*4,1);

% Run simulation
out = sim('a1_obstacle.slx',23.1);
save('obstacle.mat','out','-v7.3');
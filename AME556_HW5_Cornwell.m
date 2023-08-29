%% AME 556 HW5
% Tara Cornwell

%% Q1a. I-O linearization
clear; clc;
global params

params.k = [1.25 1]; % Look at stability-related notes to see how to get k

% Initial conditions
x0 = [0;0];
z0 = [x0(1)-1; x0(2)];

tspan = [0;20];
[t,z]=ode45(@sys_dynamics,tspan,z0);

% Get control input
for ii = 1:length(t)
    u(ii,1) = IO(t(ii),z(ii,:));
end

% Transform back to x
y = z(:,1);
x = y+1;

% Plots
figure;
subplot(2,1,1)
plot(t,x,'LineWidth',1.5)
ylabel('x position')
subplot(2,1,2)
plot(t,u,'LineWidth',1.5)
ylabel('torque')
xlabel('time (s)')


%% Functions
function dz = sys_dynamics(t,z)
    global params
    k = params.k;

    mu = -k*z;

    % Create A and B matrices
    A = [0 1; 0 0];
    B = [0; 1];

    % Solve linear system dynamics
    dz = A*z + B*mu;

end

% IO linearizing controller
function u = IO(t,z)
    global params
    k = params.k;

    mu = -k*z';

    dx = z(2);
    u = dx\(mu + dx^3);
end
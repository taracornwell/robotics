%% AME 556 HW 1
% Tara Cornwell
% due Friday Jan 27, 2023

%% Q3: [Roll, Pitch, Yaw] = [pi/3, -pi/4, pi/2]

clear; clc;

% a) Determine rotation matrix using built-in MATLAB function

% Use built-in function to conver Euler angles to rotation matrix R
R = eul2rotm([pi/2, -pi/4, pi/3]); % default is ZYX
disp(R)

% b) Apply rotation to vector p = [1;2;3] to get new vector p1

% Set coordinates for p vector and define relative to origin 0
p = [1;2;3];
O = [0;0;0];
Op = [O,p];

% Create figure for plotting
figure;
hold on;
% Plot original p vector
plot3(Op(1,:),Op(2,:),Op(3,:),'-^','LineWidth',1.5)
% Add coordinates in text
text(p(1),p(2),p(3),'(1,2,3)','VerticalAlignment','top')

% Multiply p by R to get new vector p1
p1 = R*p;
Op1 = [O,p1];

% Plot new p1 vector after rotation R
plot3(Op1(1,:),Op1(2,:),Op1(3,:),'-^','LineWidth',1.5)
% Add coordinates in text
str = sprintf('(%0.3g,%0.3g,%0.3g)',p1(1),p1(2),p1(3));
text(p1(1),p1(2),p1(3),str,'VerticalAlignment','top')

% Plot settings
grid on
xlabel('x')
ylabel('y')
zlabel('z')
legend({'p','p1'})
axis equal
view(3)
title('Rotate p by Euler angles [pi/3,-pi/4,pi/2]')
set(gca,'FontName','Arial','FontSize',16)

%% Q4: Triangle ABC with A = [0;0;0], B = [2;0;0], C = [0;1;0]

clear; clc;

% a) Draw triangle
A = [0;0;0];
B = [2;0;0];
C = [0;1;0];

% Define triangle
tri = [A,B,C];

% Plot original triangle
figure;
subplot(2,3,1)
hold on;
patch(tri(1,:),tri(2,:),tri(3,:),'blue');

% Plot settings
xlabel('x')
ylabel('y')
zlabel('z')
view(3)
axis('equal')
grid on
title('Original triangle ABC')
set(gca,'FontName','Arial','FontSize',16)

% Add vertices as text
labels = {'A','B','C'};
plot3(tri(1,:),tri(2,:),tri(3,:),'.k')
text(tri(1,:),tri(2,:),tri(3,:),labels,'VerticalAlignment','top')

% b) Apply rotation pi/6 about X0 axis and plot.
rot_b = eul2rotm([0 0 pi/6]);    % default is ZYX

% New vector = R * original vector
A2 = rot_b*A;
B2 = rot_b*B;
C2 = rot_b*C;

% Define new triangle with rotated vertices
tri_b = [A2,B2,C2];

% Plot new triangle
subplot(2,3,2)
hold on;
patch(tri_b(1,:),tri_b(2,:),tri_b(3,:),'blue');

% Plot settings
xlabel('x')
ylabel('y')
zlabel('z')
view(3)
axis('equal')
grid on
title('Rotated pi/6 about X_0')
set(gca,'FontName','Arial','FontSize',16)

% Add vertices as text
plot3(tri_b(1,:),tri_b(2,:),tri_b(3,:),'.k')
text(tri_b(1,:),tri_b(2,:),tri_b(3,:),labels,'VerticalAlignment','top')

% c) Apply rotation -pi/4 about Y0 to current triangle and plot.
rot_c = eul2rotm([0 -pi/4 0]);   % default is ZYX

% H02 = H*H01 where H transforms {1}->{2} wrt {0}
H02 = rot_c*rot_b;
% New vector = H * original vector
A3 = H02*A;
B3 = H02*B;
C3 = H02*C;

% Define new triangle with rotated vertices
tri_c = [A3,B3,C3];

% Plot new triangle
subplot(2,3,3)
hold on;
patch(tri_c(1,:),tri_c(2,:),tri_c(3,:),'blue');

% Plot settings
xlabel('x')
ylabel('y')
zlabel('z')
view(3)
axis('equal')
grid on
title('Rotated -pi/4 about Y_0')
set(gca,'FontName','Arial','FontSize',16)

% Add vertices as text
plot3(tri_c(1,:),tri_c(2,:),tri_c(3,:),'.k')
text(tri_c(1,:),tri_c(2,:),tri_c(3,:),labels,'VerticalAlignment','top')

% d) Apply rotation of 2pi/3 about Z0 to current triangle and plot.
rot_d = eul2rotm([2*pi/3 0 0]);  % default is ZYX

% H03 = H*H02 where H transforms {2}->{3} wrt {0}
H03 = rot_d*H02;
% New vector = H * original vector
A4 = H03*A;
B4 = H03*B;
C4 = H03*C;

% Define new triangle with rotated vertices
tri_d = [A4,B4,C4];

% Plot new triangle
subplot(2,3,4)
hold on;
patch(tri_d(1,:),tri_d(2,:),tri_d(3,:),'blue');

% Plot settings
xlabel('x')
ylabel('y')
zlabel('z')
view(3)
axis('equal')
grid on
title('Rotated 2pi/3 about Z_0')
set(gca,'FontName','Arial','FontSize',16)

% Add vertices as text
plot3(tri_d(1,:),tri_d(2,:),tri_d(3,:),'.k')
text(tri_d(1,:),tri_d(2,:),tri_d(3,:),labels,'VerticalAlignment','top')

% e) Apply rotation R to current triangle so result will be the same as
% original triangle ABC. Find R and plot.

% Inverse of the current rotation matrix will return to original
R = inv(H03);
% Original vector = R * new vector where R = inverse(H)
A5 = H03\A4;
B5 = H03\B4;
C5 = H03\C4;

% Define new triangle with rotated vertices
tri_e = [A5,B5,C5];

% Plot new AKA original triangle
subplot(2,3,6)
hold on;
patch(tri_e(1,:),tri_e(2,:),tri_e(3,:),'blue');

% Plot settings
xlabel('x')
ylabel('y')
zlabel('z')
view(3)
axis('equal')
grid on
title('Rotated by R about to get original')
set(gca,'FontName','Arial','FontSize',16)

% Add vertices as text
plot3(tri_e(1,:),tri_e(2,:),tri_e(3,:),'.k')
text(tri_e(1,:),tri_e(2,:),tri_e(3,:),labels,'VerticalAlignment','top')

%% Q6: 3-DOF robot arm with a1 = 0.1m, a2 = 0.2m, a3 = 0.2m
% a) Forward kinematics

clear; clc;

% Set symbolic values
syms theta1 theta2 theta3 a1 a2 a3

% Define link lengths
% a1 = 0.1;
% a2 = 0.2;
% a3 = 0.2;

% FROM O0 to O1
    % Rotation about Z0
    R1 = [cos(theta1)   -sin(theta1)   0   0;
          sin(theta1)   cos(theta1)    0   0;
          0                    0       1   0;
          0                    0       0   1];

    % Translation up Z
    T1 = [1 0 0 0;
          0 1 0 0;
          0 0 1 a1;
          0 0 0 1];
    
    % Homogeneous transformation matrix H = R*T
    H01 = simplify(R1*T1);

% FROM O1 to O2
    % Rotation about Y1
    R2 = [cos(theta2)    0    sin(theta2)    0;
          0              1       0           0;
          -sin(theta2)   0    cos(theta2)    0;
          0              0       0           1];
    
    % Translation up X
    T2 = [1 0 0 a2;
          0 1 0 0;
          0 0 1 0;
          0 0 0 1];
    
    % Homogeneous transformation matrix H = R*T
    H12 = simplify(R2*T2);

% FROM O2 to O3
    % Rotation about Y2
    R3 = [cos(theta3)    0    sin(theta3)    0;
          0              1       0           0
          -sin(theta3)   0    cos(theta3)    0;
          0              0       0           1];
    
    % Translation up X
    T3 = [1 0 0 a3;
          0 1 0 0;
          0 0 1 0;
          0 0 0 1];

    % Homogeneous transformation matrix H = R*T
    H23 = simplify(R3*T3);

%% b) Draw robot configuration for following cases:
    % (1) theta = [0;0;0]
    % (2) theta = [0;pi/4;pi/4]
    % (3) theta = [pi/6;pi/4;-pi/2]
    % (4) theta = [pi;pi/2;pi/2]

    % Define O0 as fixed
    O0 = [0;0;0];
    
    % Set up vectors of 3 theta values according to 4 cases
    theta1_vector = [0 0 pi/6 pi];
    theta2_vector = [0 pi/4 pi/4 pi/2];
    theta3_vector = [0 pi/4 -pi/2 pi/2];

    % Loop through 4 cases
    for ii = 1:4
        clearvars theta1 theta2 theta3 O1 O2 O3 H02 H03 H01_sub H12_sub H23_sub
        
        % Set current condition
        theta1 = theta1_vector(ii);
        theta2 = theta2_vector(ii);
        theta3 = theta3_vector(ii);
        H01_sub = subs(H01);
        H12_sub = subs(H12);
        H23_sub = subs(H23);
    
        % Pull endpoint positions from each homogeneous transf. matrix wrt
        % world frame {0}
        O1 = H01_sub(1:3,end);
        H02 = simplify(H01_sub*H12_sub);    % Get in world frame {0}
        O2 = H02(1:3,end);
        H03 = simplify(H02*H23_sub);        % Get in world frame {0}
        O3 = H03(1:3,end);

        % Plot each link based on endpoint positions (and origin O0)
        subplot(2,2,ii)
        hold on;
        plot3([O0(1) O1(1)],[O0(2) O1(2)],[O0(3) O1(3)],'-o','LineWidth',1.5)
        plot3([O1(1) O2(1)],[O1(2) O2(2)],[O1(3) O2(3)],'-o','LineWidth',1.5)
        plot3([O2(1) O3(1)],[O2(2) O3(2)],[O2(3) O3(3)],'-o','LineWidth',1.5)

        % Plot settings
        xlabel('x')
        ylabel('y')
        zlabel('z')
        axis('equal')
        view(3)
        grid on
        set(gca,'FontName','Arial','FontSize',16)
        if ii == 1
            title('\Theta = [0; 0; 0]')
        elseif ii == 2
            legend({'O0O1','O1O2','O2O3'})
            title('\Theta = [0; pi/4; pi/4]')
        elseif ii == 3
            title('\Theta = [pi/6; pi/4; -pi/2]')
        elseif ii == 4
            title('\Theta = [pi; pi/2; pi/2]')
        end
    end

%% c) Now assume the arm is attached to a moving body (not fixed).

    % Make homogeneous transf. matrix T with translation
    T = [1 0 0 0.5;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];

    % Make homogeneous transf. matrix R with rotation 
    R = eul2rotm([pi/4,0,pi/2]); % 3x3 matrix
    % To get 4x4 homogeneous transf. matrix, set last row is 3 zeroes and 1 in bottom right position
    R(4,4) = 1;

    % (1) Translate forward 0.5m in x-axis of body frame, then
    %     rotate by [R,P,Y] = [pi/2, 0, pi/4]

        % H02 = H01*H12 wrt current frame
        H1 = T*R;

    % (2) Rotate by [R,P,Y] = [pi/2, 0, pi/4] and then translate forward
    %     0.5m in x-axis of body frame.

        % H02 = H01*H12 wrt current frame
        H2 = R*T;

    % (3) Rotate by [R,P,Y] = [pi/2, 0, pi/4] and then translate forward
    %     0.5m in x-axis of world frame.

        % H02 = H*H01 where H transforms {1}->{2} wrt {0}
        H3 = T*R;   % Same as (1) because order of mult. changes wrt current vs. world frame and R is always wrt world frame here

    % Draw robot configuration for each case with theta = [0;pi/4;pi/4]:
    % Set theta values
    theta1 = 0;
    theta2 = pi/4;
    theta3 = pi/4;

    % Substitute theta values into H matrices
    H01_sub = subs(H01);
    H12_sub = subs(H12);
    H23_sub = subs(H23);

    % Get original configuration without transformations
    % In order to allow multiplication by H matrices later, set 1 in 4th
    % row of vectors, but endpoint positions will just be 1:3 values
    O0 = [0;0;0;1];
    O1 = H01_sub(:,end);
    H02 = simplify(H01_sub*H12_sub);    % Get in world frame
    O2 = H02(:,end);
    H03 = simplify(H02*H23_sub);        % Get in world frame
    O3 = H03(:,end);

    % Plot original configuration without transformations
    figure;
    subplot(2,2,1)
    hold on;
    plot3([O0(1) O1(1)],[O0(2) O1(2)],[O0(3) O1(3)],'-o','LineWidth',1.5)
    plot3([O1(1) O2(1)],[O1(2) O2(2)],[O1(3) O2(3)],'-o','LineWidth',1.5)
    plot3([O2(1) O3(1)],[O2(2) O3(2)],[O2(3) O3(3)],'-o','LineWidth',1.5)

    % Plot settings
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis('equal')
    view(3)
    grid on
    title('Original configuration')
    set(gca,'FontName','Arial','FontSize',16)

    % Loop through 3 cases (defined above with H1, H2, and H3)
    for ii = 1:3
        clearvars H O0_new O1_new O2_new O3_new

        % Select homogeneous transf. matrix based on current case
        if ii == 1
            H = H1;
        elseif ii == 2
            H = H2;
        else
            H = H3;
        end

        % Get new endpoint positions by multiplying H by original
        O0_new = double(H*O0);  % set to double from sym
        O1_new = double(H*O1);
        O2_new = double(H*O2);
        O3_new = double(H*O3);

        % Plot current configuration
        subplot(2,2,ii+1)
        hold on;
        plot3([O0_new(1) O1_new(1)],[O0_new(2) O1_new(2)],[O0_new(3) O1_new(3)],'-o','LineWidth',1.5)
        plot3([O1_new(1) O2_new(1)],[O1_new(2) O2_new(2)],[O1_new(3) O2_new(3)],'-o','LineWidth',1.5)
        plot3([O2_new(1) O3_new(1)],[O2_new(2) O3_new(2)],[O2_new(3) O3_new(3)],'-o','LineWidth',1.5)
        
        % Plot settings
        xlabel('x')
        ylabel('y')
        zlabel('z')
        axis('equal')
        view(3)
        grid on
        set(gca,'FontName','Arial','FontSize',16)
        if ii == 1
            title('(1) Translate in current x-axis, then rotate')
        elseif ii == 2
            title('(2) Rotate, then translate in current x-axis')
        elseif ii == 3
            legend({'O0O1','O1O2','O2O3'})
            title('(3) Rotate, then translate in world x-axis')
        end
    end
    sgtitle('\Theta = [0; pi/4; pi/4]','FontName','Arial','FontSize',20)
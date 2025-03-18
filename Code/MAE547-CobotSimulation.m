%SIMULATING THE USE OF FORWARD KINEMATICS, INVERSE KINEMATICS & TRAJECTORY PLANNING IN A 6 DOF COBOT ROBOT 
% Defining the robot parameters for a 6 DOF COBOT Robot
L1 = 1; % Link length 1
L2 = 1; % Link length 2
L3 = 1; % Link length 3
L4 = 1; % Link length 4
L5 = 1; % Link length 5
L6 = 1; % Link length 6

% Creating the robot model
robot = SerialLink([
    Revolute('d', 0, 'a', 0, 'alpha', pi/2, 'offset', 0)
    Revolute('d', 0, 'a', L2, 'alpha', 0, 'offset', 0)
    Revolute('d', 0, 'a', L3, 'alpha', 0, 'offset', 0)
    Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0)
    Revolute('d', 0, 'a', 0, 'alpha', pi/2, 'offset', 0)
    Revolute('d', 0, 'a', 0, 'alpha', 0, 'offset', 0)
], 'name','6 DOF COBOT UNIVERSAL ROBOTICS MANIPULATOR');

% User Inputs the co-ordinates of the points A, B, and C (Anything within reach)
%  Like [0.5, 0.5, 0.5],[-0.5, -0.5, 0.5],[0.5, -0.5, -0.5])
pointA = input('Enter coordinates for point A [x, y, z]: ');
pointB = input('Enter coordinates for point B [x, y, z]: ');
pointC = input('Enter coordinates for point C [x, y, z]: ');

% Perform inverse kinematics and find joint angles for A, B and C
% respectively
qA = robot.ikine(transl(pointA), 'mask', [1 1 1 1 1 1], 'q0', [0 0 0 0 0 0]);
qB = robot.ikine(transl(pointB), 'mask', [1 1 1 1 1 1], 'q0', [0 0 0 0 0 0]);
qC = robot.ikine(transl(pointC), 'mask', [1 1 1 1 1 1], 'q0', [0 0 0 0 0 0]);

% Check if points are reachable w.r.t. the robot arm
if any(isnan(qA)) || any(isnan(qB)) || any(isnan(qC))
    error('One or more points are not reachable by the robot.');
end

% The "jtraj" function in this code is used to generate joint space trajectories 
% from point A to point B and from point B to point C taking the initial and final
% joint configurations (qA and qB & qB and qC),
% The number of steps in the trajectory (25 in this case). 
% "jtraj" returns a matrix where each row represents a point in the trajectory in joint space.
% Then we combine the trajectories A to B and B to C
trajectoryAB = jtraj(qA, qB, 25);
trajectoryBC = jtraj(qB, qC, 25);
combinedTrajectory = [trajectoryAB; trajectoryBC];

% 'fkine' performs forward kinematics on the combined joint space trajectory,resulting 
% in a series of transformation matrices that represent the pose of the end-effector.
% 'transl' extracts the translational (x,y,z) components from the transformation matrices.
% The position of the end-effector in Cartesian space and is stored in 'ee_trajectory'.
ee_trajectory = transl(robot.fkine(combinedTrajectory));

% Visualize the robot and the trajectory in the initial position
figure;
subplot(1, 1, 1);
robot.plot(qA, 'noarrow');
hold on;

% Mark points A, B, and C with small spheres
scatter3(pointA(1), pointA(2), pointA(3), 100, 'go', 'filled', 'MarkerEdgeColor', 'g');
scatter3(pointB(1), pointB(2), pointB(3), 100, 'bo', 'filled', 'MarkerEdgeColor', 'b');
scatter3(pointC(1), pointC(2), pointC(3), 100, 'ro', 'filled', 'MarkerEdgeColor', 'r');

% Mark the three points on the robot plot with labels
text(pointA(1), pointA(2), pointA(3), 'A', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'g');
text(pointB(1), pointB(2), pointB(3), 'B', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'b');
text(pointC(1), pointC(2), pointC(3), 'C', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r');

% Plot the combined trajectory with circles in the same figure
for i = 1:size(combinedTrajectory, 1)
    robot.plot(combinedTrajectory(i, :), 'noarrow');
    % Mark the trajectory points with small circles
    scatter3(ee_trajectory(i, 1), ee_trajectory(i, 2), ee_trajectory(i, 3), 'ko', 'filled');
    pause(0.1);  % Adjust the pause duration as needed
end

% Mark the three points on the final robot position
hold on;
scatter3(pointA(1), pointA(2), pointA(3), 'go', 'LineWidth', 2, 'MarkerFaceColor', 'g');
scatter3(pointB(1), pointB(2), pointB(3), 'bo', 'LineWidth', 2, 'MarkerFaceColor', 'b');
scatter3(pointC(1), pointC(2), pointC(3), 'ro', 'LineWidth', 2, 'MarkerFaceColor', 'r');

% Set axis properties
axis equal;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('UR5 Robot Moving along the Combined Trajectory with User-Defined Points and Trajectory');

%%%%%%%%%%%%%%%%%%%%%%%%PLOTTING THE WORKSPACE/OPERATIONAL SPACE%%%%%%%%%%%%%%%%%%%%%%%

% Defines the number of points for each joint angle
% 'num' determines the number of points sampled for each joint angle. 
% To get approximately 3000 points in the 6D joint space, we solve num^6 = 3000, 
% which gives num = (points required)^(1/6). Rounding to the ceiling nearest integer, we get num = 7 to 9.
num = 7;

% Initializes a counter c
c = 0;

% Defines the size of the reachability set 
n = 5^7;

% Initialize the reachability set
RS = zeros(n, 3);

% Define the range of each joint angle
q1 = linspace(-pi, pi, num);
q2 = linspace(-pi, pi, num);
q3 = linspace(-pi/2, pi/2, num);
q4 = linspace(-pi, pi, num);
q5 = linspace(-pi, pi, num);
q6 = linspace(-pi/2, pi/2, num);

% Loop over all combinations of joint angles
for i = 1:num
    for j = 1:num
        for k = 1:num
            for l = 1:num
                for m = 1:num
                    for n = 1:num
                        % Increment the counter
                        c = c + 1;
                        
                        % Define the joint configuration
                        q = [q1(i) q2(j) q3(k) q4(l) q5(m) q6(n)];
                        
                        % Calculate the forward kinematics
                        TM = robot.fkine(q);
                        
                        % Extract the end-effector position
                        pe = transl(TM);
                        
                        % Store the end-effector position in the reachability set
                        RS(c, :) = pe;
                    end
                end
            end
        end
    end
end

% Extract the X, Y, and Z coordinates of the end-effector positions
X_RS = RS(:, 1);
Y_RS = RS(:, 2);
Z_RS = RS(:, 3);

% Plot the operational workspace in 3D and 2D projections
subplot(1, 3, 1);
scatter3(X_RS, Y_RS, Z_RS, '*');
title('Operational Workspace 3D');
xlabel('X'); ylabel('Y'); zlabel('Z');

subplot(1, 3, 2);
scatter(X_RS, Y_RS, '*');
title('Operational Workspace X-Y');
xlabel('X'); ylabel('Y');

subplot(1, 3, 3);
scatter(Y_RS, Z_RS, '*');
title('Operational Workspace Y-Z');
xlabel('Y'); ylabel('Z');

%%%%%%%%%%%%%%%%%%%%%%%ANALYTICAL AND LINEAR JACOBIAN%%%%%%%%%%%%%%%%%%%%%%%%
% Define the robot
L1 = 1; % Link length 1
L2 = 1; % Link length 2
L3 = 1; % Link length 3
L4 = 1; % Link length 4
L5 = 1; % Link length 5
L6 = 1; % Link length 6

robot = SerialLink([
    Revolute('d', 0, 'a', 0, 'alpha', pi/2, 'offset', 0)
    Revolute('d', 0, 'a', L2, 'alpha', 0, 'offset', 0)
    Revolute('d', 0, 'a', L3, 'alpha', 0, 'offset', 0)
    Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0)
    Revolute('d', 0, 'a', 0, 'alpha', pi/2, 'offset', 0)
    Revolute('d', 0, 'a', 0, 'alpha', 0, 'offset', 0)
]);

% Calculate the Jacobian
q = [0 0 0 0 0 0]; % Joint angles
J = robot.jacob0(q); % Jacobian matrix at the given joint configuration

% Display the Jacobian
disp('Jacobian:');
disp(J);
% Calculate the analytical Jacobian
analyticalJ = robot.jacob0(q, 'rpy'); % Analytical Jacobian matrix at the given joint configuration

% Calculate the linear Jacobian
linearJ = robot.jacob0(q, 'trans'); % Linear Jacobian matrix at the given joint configuration


% Display the linear Jacobian
disp('Linear Jacobian:');
disp(linearJ);
%%%%%%%%%%%%%%%%%%%JOINT VELOCITIES and ACCELERATIONS%%%%%%%%%%%%%%%%%%%%
% Generate trajectory from A to B to C
[trajectoryAB, velocityAB, accelerationAB] = jtraj(qA, qB, 25);
[trajectoryBC, velocityBC, accelerationBC] = jtraj(qB, qC, 25);

% Combine trajectories A to B and B to C
combinedTrajectory = [trajectoryAB; trajectoryBC];
combinedVelocity = [velocityAB; velocityBC];
combinedAcceleration = [accelerationAB; accelerationBC];

% Display the joint velocities and accelerations
figure;
subplot(2, 1, 1);
plot(combinedVelocity);
title('Joint Velocities');
xlabel('Time');
ylabel('Velocity');

subplot(2, 1, 2);
plot(combinedAcceleration);
title('Joint Accelerations');
xlabel('Time');
ylabel('Acceleration');

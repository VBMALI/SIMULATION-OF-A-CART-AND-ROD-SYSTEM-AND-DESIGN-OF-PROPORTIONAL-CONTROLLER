%% Clean Up
%clearvars;  % Clear the workspace
clc;  % Clear the command window
close('all');  % Close the open figures
Simulink.sdi.clear;  % Clear the simulink data inspector

%% Physical Constancts
g  = 10;   % grav acceleration in m/s^2

%% Problem Statement Parameters
% Motor
Motor.Kt =  7.68*10^-3; % Torque Constant(Nm/A)
Motor.Ke = Motor.Kt;  % Back emf constant
Motor.Ra =  2.60;   % Armature resistance(ohm)
Motor.La =  1.8*10^-4; % Inductance (H)
Motor.Jm = 3.90*10^-7;  % rotational moment of Inertia(Kg-m2)
Motor.Bm = 1.0e-6; % Viscous friction factor of motor

% Gear box parameters
Gear.R =  6.35*10^-3; % Driving gear radius (m)
Gear.N =  3.71; % Gear Ratio
Gear.eta = 0.9;  % Gear box efficiency
Gear.J = 0;  % Gear box inertia

% Cart parameters
Cart.Mc = 0.521;  % Mass of Cart in Kg
Cart.Mw = 0.0; % Mass of wheel in Kg
Cart.Mt = Cart.Mc + Cart.Mw ;  % Total mass of cart
Cart.Jw = Motor.Jm + Gear.J; % Moment of inertia of wheel
Cart.Rw = 0.01483; % Radius of encoder wheel in m

% Rod
Rod.Mr = 0.23; % Mass of rod (Kg)
Rod.Lc = 0.64; % Stiffness of suspension spring (N/m)
Rod.Jr = 7.88e-3;  % Moment of inertia of rod in Kg-m2

%% Lumped Parameters
Calc.a1 = Cart.Mt + Rod.Mr + (Cart.Jw/Gear.R^2);
Calc.b1 = Rod.Mr*Rod.Lc/2;
Calc.c1 = (Gear.N^2/Gear.R^2)*((Gear.eta*Motor.Kt*Motor.Ke/Motor.Ra)+Motor.Bm);
Calc.b2 = Rod.Jr + (Rod.Mr*(0.5*Rod.Lc)^2);
Calc.d = Gear.N*Gear.eta*Motor.Kt/(Gear.R*Motor.Ra);
Calc.e = 1 - (Calc.b1^2/(Calc.a1*Calc.b2));

Mat.A32 = g*Calc.b1^2/(Calc.a1*Calc.b2*Calc.e);
Mat.A33 = -Calc.c1/(Calc.a1*Calc.e);
Mat.A42 = -g*Calc.b1/(Calc.b2*Calc.e);
Mat.A43 = Calc.b1*Calc.c1/(Calc.a1*Calc.b2*Calc.e);
Mat.B31 = Calc.d/(Calc.a1*Calc.e);
Mat.B41 = -Calc.b1*Calc.d/(Calc.a1*Calc.b2*Calc.e);

%% State Space Matrices
      
A = [ 0    0        1     0
      0    0        0     1
      0 Mat.A32  Mat.A33  0
      0 Mat.A42  Mat.A43  0 ]; % A Matrix
     
B = [0;0;Mat.B31;Mat.B41]; % B Matrix
 
C = [1 0 0 0
     0 1 0 0
     0 0 1 0
     0 0 0 1]; % C Matrix

D = [0;0;0;0]; % D Matrix

%% Input Parameters

V = 1;
Input.ts = 1;
Input.td = 4;
Input.tc = 2;

%% Simulation parameters
simPrm.solTyp = 'Fixed-step';  % Solver type
simPrm.sol    = 'ode3';  % Solver type 2
simPrm.dt  = 0.001; % Integration step size
simPrm.tEnd = 20; % Simulation end time
simPrm.t = 0:simPrm.dt:simPrm.tEnd;  %simulation time goes from 0 to 20

%% Simulate the math model
set_param('simTrue','SolverType',simPrm.solTyp);  % set this solver type in simulink parameters 
set_param('simTrue','Solver',simPrm.sol);     % set this integration method in simulink solver
SimOut = sim('simTrue','SignalLoggingName','sdata'); % Simulate the math model and save the data

%% Extract Data for plotting
Position = 1;  % Variable for extracting x1 state of linear model
Velocity = 2;  % Variable for extracting x3 state of linear model
rod_A = 3;  % Variable for extracting x2 state of linear model
rod_Ang_Vel = 4; % Variable for extracting x4 state of linear model
Voltage = 5;  % Variable for extracting Voltage data

%Results 
Results.X = SimOut.sdata{Position}.Values.Data(:,1); % Cart position of Lin model
Results.R = SimOut.sdata{rod_A}.Values.Data(:,1); % Rod angle of Lin model
Results.V = SimOut.sdata{Voltage}.Values.Data(:,1); % Voltage of Lin model
Results.Xd = SimOut.sdata{Velocity}.Values.Data(:,1); % Cart velocity of Lin model
Results.Rd = SimOut.sdata{rod_Ang_Vel}.Values.Data(:,1); % Rod angular velocity of Lin model

%% Plot the results
figure(1);
plot(simPrm.t,Results.V,'r','LineWidth',1);
hold on
xticks(0:1:20);  % Give the x axis ticks
yticks(0:0.1:1); % Give the y axis ticks
title("Voltage Input (V)"); % Give the title
xlabel('Time (s)');  % Give X label
ylabel('Voltage(V)');  % Give Y label

figure(2);
plot(simPrm.t,Results.X,'b','LineWidth',1);
hold on
xticks(0:1:20);  % Give the x axis ticks
yticks(0:0.5:5); % Give the y axis ticks
title("Cart position vs Time for True model"); % Give the title
xlabel('Time (s)');  % Give X label
ylabel('Cart position(m)');  % Give Y label

figure(3);
plot(simPrm.t,Results.R,'b','LineWidth',1);
hold on
xticks(0:1:20);  % Give the x axis ticks
title("Rod Angle vs Time for True model"); % Give the title
xlabel('Time (s)');  % Give X label
ylabel('Rod Angle(rad)');  % Give Y label

figure(4);
plot(simPrm.t,Results.Xd,'b','LineWidth',1);
hold on
xticks(0:1:20);  % Give the x axis ticks
title("Cart Velocity vs Time for True model"); % Give the title
xlabel('Time (s)');  % Give X label
ylabel('Cart velocity (m/s)');  % Give Y label

figure(5);
plot(simPrm.t,Results.Rd,'b','LineWidth',1);
hold on
xticks(0:1:20);  % Give the x axis ticks
title("Rod Angular Velocity vs Time for True model"); % Give the title
xlabel('Time (s)');  % Give X label
ylabel('Rod Angular Velocity (rad/s)');  % Give Y label
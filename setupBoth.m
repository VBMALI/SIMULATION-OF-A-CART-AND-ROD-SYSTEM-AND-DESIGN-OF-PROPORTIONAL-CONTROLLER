%% Clean Up
clearvars;  % Clear the workspace
clc;  % Clear the command window
close('all');  % Close the open figures
Simulink.sdi.clear;  % Clear the simulink data inspector

%% Physical Constants
g  = 10;   % grav acceleration in m/s^2

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

%% Matrix Coefficients
Mat.A32 = g*Calc.b1^2/(Calc.a1*Calc.b2*Calc.e);
Mat.A33 = -Calc.c1/(Calc.a1*Calc.e);
Mat.A42 = -g*Calc.b1/(Calc.b2*Calc.e);
Mat.A43 = Calc.b1*Calc.c1/(Calc.a1*Calc.b2*Calc.e);
Mat.B31 = Calc.d/(Calc.a1*Calc.e);
Mat.B41 = -Calc.b1*Calc.d/(Calc.a1*Calc.b2*Calc.e);

Mat.Ad33 = -Calc.c1/Calc.a1;
Mat.Ad42 = -g*Calc.b1/Calc.b2;
Mat.Bd31 = Calc.d/Calc.a1;

%% State Space Matrices
% Coupled System Matrices      
Ac = [ 0    0        1     0
      0    0         0     1
      0 Mat.A32  Mat.A33   0
      0 Mat.A42  Mat.A43   0 ]; % A Matrix
     
Bc = [0;0;Mat.B31;Mat.B41]; % B Matrix
 
Cc = [1 0 0 0
      0 1 0 0
      0 0 1 0
      0 0 0 1]; % C Matrix

Dc = [0;0;0;0]; % D Matrix

% Decoupled System Matrices      
Ad = [0    0           1     0
      0    0           0     1
      0    0       Mat.Ad33  0
      0  Mat.Ad42      0     0 ]; % A Matrix
     
Bd = [0;0;Mat.Bd31;0]; % B Matrix
 
Cd = [1 0 0 0
      0 1 0 0
      0 0 1 0
      0 0 0 1]; % C Matrix

Dd = [0;0;0;0]; % D Matrix

%% Input Parameters
Input.Ref = 0.3;  % Reference position in meters
Input.ts = 1;  % Start time of pulse
Input.freq = 4.9319; % Frequency of swinging rod

%% Calculate Kp
Zeta = 0.456;
Omega_n = (Calc.c1/Calc.a1)/(2*Zeta);
Kp = Omega_n^2/(Calc.d/Calc.a1);

%% Simulation parameters
simPrm.solTyp = 'Fixed-step';  % Solver type
simPrm.sol    = 'ode3';  % Solver type 2
simPrm.dt  = 0.001; % Integration step size
simPrm.tEnd = 20; % Simulation end time
simPrm.t = 0:simPrm.dt:simPrm.tEnd;  %simulation time goes from 0 to 20

%% Simulate the math model
set_param('simBoth','SolverType',simPrm.solTyp);  % set this solver type in simulink parameters 
set_param('simBoth','Solver',simPrm.sol);     % set this integration method in simulink solver
SimOut = sim('simBoth','SignalLoggingName','sdata'); % Simulate the math model and save the data

% Extract Data for plotting
Reference_Position = 1;  % Variable for extracting reference input
Position_D = 2;  % Variable for extracting x3 state of linear model
Position_C = 3;  % Variable for extracting x2 state of linear model
Rod_A_C = 4; % Variable for extracting x4 state of linear model
Rod_A_D = 5;  % Variable for extracting Voltage data

%Results 
Results.Xd = SimOut.sdata{Position_D}.Values.Data(:,1); % Cart position of Lin model
Results.Rd = SimOut.sdata{Rod_A_D}.Values.Data(:,1); % Rod angle of Lin model
Results.Xc = SimOut.sdata{Position_C}.Values.Data(:,1); % Voltage of Lin model
Results.Rc = SimOut.sdata{Rod_A_C}.Values.Data(:,1); % Cart velocity of Lin model
Results.Ref = SimOut.sdata{Reference_Position}.Values.Data(:,1); % Rod angular velocity of Lin model

%% Plot the results
figure(1);
plot(simPrm.t,Results.Xd,'r','LineWidth',1);
hold on
plot(simPrm.t,Results.Ref,'b','LineWidth',1);
xticks(0:1:20);  % Give the x axis ticks
title("Controlled Position of cart for Decoupled Model"); % Give the title
xlabel('Time (s)');  % Give X label
ylabel('Position (cm)');  % Give Y label
legend('Controlled Position (cm)','Reference Position (cm)'); % legend

figure(2);
plot(simPrm.t,Results.Xc,'r','LineWidth',1);
hold on
plot(simPrm.t,Results.Ref,'b','LineWidth',1);
xticks(0:1:20);  % Give the x axis ticks
title("Controlled Position of cart for coupled Model"); % Give the title
xlabel('Time (s)');  % Give X label
ylabel('Position (cm)');  % Give Y label
legend('Controlled Position (cm)','Reference Position (cm)'); % legend
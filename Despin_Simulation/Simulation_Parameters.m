% This simulation models the despin process of the space emulator. Nothing
% should be changed here. Also in order for the visual simulation to function 
% properly you need to replace the directory in the inertia blocks of the 
% Simulink Model with the corresponding STEP files that are in this folder
% in order for Matlab to be able to locate them.

% Startup & Cleanup

close all;
clear all;
clc;

% Inertia Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Body's Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Active System without RW

m_b = 16.9927;  % Body's Mass [kg]

COM_b = [0/10^3 -0.81/10^3 0.51/10^3]; % Body's COM [m] 

I_b = [151612935.5/10^9	       -12035.62/10^9        -9941.66/10^9; % Body's Inertia [kg*m^2]
         -12035.62/10^9	     235208657.4/10^9	    540868.82/10^9;
          -9941.66/10^9	       540868.82/10^9	    286720213/10^9];

% Stand's Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Passive System

m_st = 9.48989;  % Stand's Mass [kg]

COM_st = [0.19/10^3 2.69/10^3 75.2/10^3]; % Stand's COM [m]

I_st = [118601552.74/10^9	       -491524.85/10^9         95420.21/10^9; % Stand's Inertia [kg*m^2]
          -491524.85/10^9	     146920027.49/10^9	    -2298412.62/10^9;
            95420.21/10^9	      -2298412.62/10^9	   164134840.77/10^9];

% Reaction Wheel's Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

m_rw = 0.24806; % Reaction Wheel's Mass [kg]

COM_rw = [0/10^3 0/10^3 0.73/10^3];  % Reaction Wheel's COM [m]
       
I_single_rw = [220358.07/10^9         0               0;         % Single Reaction Wheel's Inertia [kg*m^2]
                    0            220358.07/10^9       0;
                    0                 0          418129.72/10^9];         

I_rw = [418129.72/10^9        0               0;         % Three Reaction Wheel's Inertia [kg*m^2]
             0          418129.72/10^9        0;
             0                0         418129.72/10^9];
         
I_rw_z = 418129.72/10^9; % Reaction Wheel's Polar Inertia [kg*m^2]
         
% System's Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Active System with RW

m_sys = 17.73689;    % System's Mass [kg]
           
COM_sys = [0/10^3 0/10^3 -0.31/10^3];  % System's COM [m]         

I_sys = [177776253.6/10^9         -12078.47/10^9	        -9914.94/10^9; % System's Inertia [kg*m^2]
           -12078.47/10^9	    252995396.8/10^9	      1316589.63/10^9;
            -9914.94/10^9        1316589.63/10^9	     329251886.4/10^9];
     
% Model's Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        
R_0_to_1 = [sqrt(2)/2 0.5 0.5; -sqrt(2)/2 0.5 0.5; 0 -sqrt(2)/2 sqrt(2)/2];
R_1_to_0 = inv(R_0_to_1);

a1 = [1;0;0];
a2 = [0;1;0];
a3 = [0;0;1];

a1_0_to_R1 = a1' * R_0_to_1;
a2_0_to_R2 = a2' * R_0_to_1;
a3_0_to_R3 = a3' * R_0_to_1;

a1_R1_to_0 = R_1_to_0 * a1;
a2_R2_to_0 = R_1_to_0 * a2;
a3_R3_to_0 = R_1_to_0 * a3;

I_new = I_sys - I_rw_z * (a1_R1_to_0*a1_0_to_R1+a2_R2_to_0*a2_0_to_R2+a3_R3_to_0*a3_0_to_R3); % Combined Inertia of the system for control [kg*m^2]

m_tot = m_sys + m_st; % Total Mass of the system [m]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Simulation and Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Motor Restrictions

x1 = -1.5:1:1.5;
y1a = 10000 + 0 * x1;
y1b = -10000 + 0 * x1;

y2 = -10000:1:10000;
x2a = 0.0911 + 0 * y2;
x2b = -0.0911 + 0 * y2;

x3 = 0:0.01:(265*24/6920);
y3 = 265*24 - 6920*x3;

x4 = -(265*24/6920):0.01:0;
y4 = -265*24 - 6920*x4;

x5 = -1.5:1:1.5;
y5 = 0*x5;

y6 = -10000:10000:10000;
x6 = 0*y6;

x7 = 0.05:0.01:1.5;
y7 = (50./x7) * 60 / (2*pi);

x8 = -1.5:0.01:-0.05;
y8 = (50./x8) * 60 / (2*pi);

% Model Estimator Control Model

sim 'Despin_Simulink_Model.slx'

w_rw = w_rw * 60 / (2*pi);

% Results

% Reaction Wheel Plots

figure('Name','Reaction Wheel No.3 w-T plot Model Estimation Control');
plot(T(:,3),w_rw(:,3),x1,y1a,'r',x1,y1b,'r',x2a,y2,'--g',x2b,y2,'--g',3*x2a,y2,'r',3*x2b,y2,'r',x3,y3,'r',x4,y4,'r',x5,y5,'k',x6,y6,'k',x7,y7,'r',x8,y8,'r');
title('Speed-Torque Plot');
xlabel('Torque [Nm]');
ylabel('Speed [rpm]');

figure('Name','Reaction Wheel No.2 w-T plot');
plot(T(:,2),w_rw(:,2),x1,y1a,'r',x1,y1b,'r',x2a,y2,'--g',x2b,y2,'--g',3*x2a,y2,'r',3*x2b,y2,'r',x3,y3,'r',x4,y4,'r',x5,y5,'k',x6,y6,'k',x7,y7,'r',x8,y8,'r');
title('RW No.2 Speed-Torque Plot');
xlabel('Torque [Nm]');
ylabel('Speed [rpm]');

figure('Name','Reaction Wheel No.1 w-T plot');
plot(T(:,1),w_rw(:,1),x1,y1a,'r',x1,y1b,'r',x2a,y2,'--g',x2b,y2,'--g',3*x2a,y2,'r',3*x2b,y2,'r',x3,y3,'r',x4,y4,'r',x5,y5,'k',x6,y6,'k',x7,y7,'r',x8,y8,'r');
title('RW No.1 Speed-Torque Plot');
xlabel('Torque [Nm]');
ylabel('Speed [rpm]');

figure('Name','All Reaction Wheel w-T plot');
plot(T(:,1),w_rw(:,1),T(:,2),w_rw(:,2),T(:,3),w_rw(:,3),x1,y1a,'r',x1,y1b,'r',x2a,y2,'--g',x2b,y2,'--g',3*x2a,y2,'r',3*x2b,y2,'r',x3,y3,'r',x4,y4,'r',x5,y5,'k',x6,y6,'k',x7,y7,'r',x8,y8,'r');
title('All Reaction Wheel Speed-Torque Plot');
xlabel('Torque [Nm]');
ylabel('Speed [rpm]');

% Thruster Force Plots

figure('Name','Thruster 1 PWM Force');
plot(F_thruster(:,1));
title('Force - Time Plot');
ylabel('Force [N]');
xlabel('time [sec]');

figure('Name','Thruster 2 PWM Force');
plot(F_thruster(:,2));
title('Force - Time Plot');
ylabel('Force [N]');
xlabel('time [sec]');

figure('Name','Thruster 3 PWM Force');
plot(F_thruster(:,3));
title('Force - Time Plot');
ylabel('Force [N]');
xlabel('time [sec]');

figure('Name','Thruster 4 PWM Force');
plot(F_thruster(:,4));
title('Force - Time Plot');
ylabel('Force [N]');
xlabel('time [sec]');

figure('Name','Thruster 5 PWM Force');
plot(F_thruster(:,5));
title('Force - Time Plot');
ylabel('Force [N]');
xlabel('time [sec]');

figure('Name','Thruster 6 PWM Force');
plot(F_thruster(:,6));
title('Force - Time Plot');
ylabel('Force [N]');
xlabel('time [sec]');

figure('Name','Thruster 7 PWM Force');
plot(F_thruster(:,7));
title('Force - Time Plot');
ylabel('Force [N]');
xlabel('time [sec]');

figure('Name','Thruster 8 PWM Force');
plot(F_thruster(:,8));
title('Force - Time Plot');
ylabel('Force [N]');
xlabel('time [sec]');

figure('Name','Thruster 9 PWM Force');
plot(F_thruster(:,9));
title('Force - Time Plot');
ylabel('Force [N]');
xlabel('time [sec]');

figure('Name','Thruster 10 PWM Force');
plot(F_thruster(:,10));
title('Force - Time Plot');
ylabel('Force [N]');
xlabel('time [sec]');

figure('Name','Thruster 11 PWM Force');
plot(F_thruster(:,11));
title('Force - Time Plot');
ylabel('Force [N]');
xlabel('time [sec]');

figure('Name','Thruster 12 PWM Force');
plot(F_thruster(:,12));
title('Force - Time Plot');
ylabel('Force [N]');
xlabel('time [sec]');

figure('Name','Thruster 12 PWM Force');
plot(F_thruster(:,:));
title('Force - Time Plot');
ylabel('Force [N]');
xlabel('time [sec]');

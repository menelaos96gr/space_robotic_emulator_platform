% This simulation models the translational and rotational motion of the
% space emulator. Only the first parameters of motion need to be changed
% acordingly. Also in order for the visual simulation to function properly
% you need to replace the directory in the inertia blocks of the Simulink
% Model with the corresponding STEP files that are in this folder in order
% for Matlab to be able to locate them.

% Startup & Cleanup

close all;
clear all;
clc;

% Basic Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n = 4; % Number of point in position and rotation sequence (Minimum of 2)

mf = 1; % 1 for motors full function (they can give more than the max continuous torque) or 0 for conservative function (they cannot give more than the max continuous torque)

pos_s = [0 0]; % [m] Starting Position (x,y)

% Position Points to achieve [m]

pos_d(1,:) = [1.5 1];
pos_d(2,:) = [1.5 1];
pos_d(3,:) = [0 0];
pos_d(4,:) = [0 0];

euler_s = [0 0 0];        % [deg] Starting Euler Angles in 3-2-1 (phi,theta,psi) Sequence

% Rotation Points to achieve [deg]

euler_d(1,:) = [0 0 0];
euler_d(2,:) = [0 0 0];
euler_d(3,:) = [0 0 0];
euler_d(4,:) = [0 0 0];

T_s = 0; % [sec] Starting time of simulation

% Desired times to reach desired position and rotation points [sec]

T_d(1) = 20;
T_d(2) = 25;
T_d(3) = 45;
T_d(4) = 50;

% Position, Rotation and Time Vectors

for i = 1:n+1
    if i == 1
        pos_v(i,:) = pos_s;
        euler_v(i,:) = euler_s;
        T_v(i) = T_s;
    else
        pos_v(i,:) = pos_d(i-1,:);
        euler_v(i,:) = euler_d(i-1,:);
        T_v(i,:) = T_d(i-1);
    end
end

euler_v = deg2rad(euler_v);

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

% Trajectory Planning

    %Calculating basic motion parameters

        %Translational Motion Parameters

        for i = 1:n
            for j = 1:2
                v_mean(i,j) = (pos_v(i+1,j) - pos_v(i,j)) / (T_v(i+1) - T_v(i));
                
                v_k(i,j) = 1.5 * v_mean(i,j);
                
                if pos_v(i+1,j) == pos_v(i,j)
                    t_t_no_acc(i,j) = T_v(i+1) - T_v(i);
                else
                    t_t_no_acc(i,j) = 2*(pos_v(i+1,j) - pos_v(i,j))/v_k(i,j)-(T_v(i+1) - T_v(i));
                end
                
                t_t_acc(i,j) = 0.5*((T_v(i+1) - T_v(i)) - t_t_no_acc(i,j));
                
                if pos_v(i+1,j) == pos_v(i,j)
                    acc_t(i,j) = 0;
                else
                    acc_t(i,j) = v_k(i,j) / t_t_acc(i,j);
                end
            end
        end

        %Rotational Motion Parameters

        for i = 1:n
            for j = 1:3
                w_mean(i,j) = (euler_v(i+1,j) - euler_v(i,j)) / (T_v(i+1) - T_v(i));
                
                w_k(i,j) = 1.5 * w_mean(i,j);
                
                if euler_v(i+1,j) == euler_v(i,j)
                    t_no_acc(i,j) = T_v(i+1) - T_v(i);
                else
                    t_no_acc(i,j) = 2*(euler_v(i+1,j) - euler_v(i,j))/w_k(i,j)-(T_v(i+1) - T_v(i));
                end
                
                t_acc(i,j) = 0.5*((T_v(i+1) - T_v(i)) - t_no_acc(i,j));
                
                if euler_v(i+1,j) == euler_v(i,j)
                    acc(i,j) = 0;
                else
                    acc(i,j) = w_k(i,j) / t_acc(i,j);
                end
            end
        end
        
    %Creating the Equations of Motion

    for i = 1:n
        t_incr(i) = (T_v(i+1) - T_v(i))/1000000;
    end
    
    time = zeros(1,n*1000000);
    
    x_d = zeros(1,n*1000000);
    v_x_d = zeros(1,n*1000000);
    a_x_d = zeros(1,n*1000000);
    
    y_d = zeros(1,n*1000000);
    v_y_d = zeros(1,n*1000000);
    a_y_d = zeros(1,n*1000000);
    
    for i = 1:n
        t_t_x_0(i) = T_v(i);
        t_t_x_1(i) = T_v(i) + t_t_acc(i,1);
        t_t_x_2(i) = T_v(i) + t_t_acc(i,1) + t_t_no_acc(i,1);
        t_t_x_3(i) = T_v(i+1);
        
        t_t_y_0(i) = T_v(i);
        t_t_y_1(i) = T_v(i) + t_t_acc(i,2);
        t_t_y_2(i) = T_v(i) + t_t_acc(i,2) + t_t_no_acc(i,2);
        t_t_y_3(i) = T_v(i+1);
    end

    psi_d = zeros(1,n*1000000);
    w_psi = zeros(1,n*1000000);
    a_psi = zeros(1,n*1000000);
    
    phi_d = zeros(1,n*1000000);
    w_phi = zeros(1,n*1000000);
    a_phi = zeros(1,n*1000000);
    
    theta_d = zeros(1,n*1000000);
    w_theta = zeros(1,n*1000000);
    a_theta = zeros(1,n*1000000);
    
    for i = 1:n
        t_t_phi_0(i) = T_v(i);
        t_t_phi_1(i) = T_v(i) + t_acc(i,1);
        t_t_phi_2(i) = T_v(i) + t_acc(i,1) + t_no_acc(i,1);
        t_t_phi_3(i) = T_v(i+1);
        
        t_t_theta_0(i) = T_v(i);
        t_t_theta_1(i) = T_v(i) + t_acc(i,2);
        t_t_theta_2(i) = T_v(i) + t_acc(i,2) + t_no_acc(i,2);
        t_t_theta_3(i) = T_v(i+1);
        
        t_t_psi_0(i) = T_v(i);
        t_t_psi_1(i) = T_v(i) + t_acc(i,3);
        t_t_psi_2(i) = T_v(i) + t_acc(i,3) + t_no_acc(i,3);
        t_t_psi_3(i) = T_v(i+1);
    end

% x Calculations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for j = 1:n
    t_x_0 = t_t_x_0(j);
    t_x_1 = t_t_x_1(j);
    t_x_2 = t_t_x_2(j);
    t_x_3 = t_t_x_3(j);
    
    p_x_1(1) = pos_v(j,1);
    p_x_1(2) = pos_v(j,1) + 0.5*acc_t(j,1)*t_t_acc(j,1)^2;
    p_x_1(3) = 0;
    p_x_1(4) = v_k(j,1);
    p_x_1(5) = 0;
    p_x_1(6) = 0;
    p_x_1(7) = 0;
    p_x_1(8) = 0;
    if j == 1
        p_x_1 = p_x_1';
    end
    
    T = [    t_x_0^7     t_x_0^6    t_x_0^5    t_x_0^4   t_x_0^3   t_x_0^2 t_x_0^1 1;
             t_x_1^7     t_x_1^6    t_x_1^5    t_x_1^4   t_x_1^3   t_x_1^2 t_x_1^1 1;
           7*t_x_0^6   6*t_x_0^5  5*t_x_0^4  4*t_x_0^3 3*t_x_0^2 2*t_x_0^1     1   0;
           7*t_x_1^6   6*t_x_1^5  5*t_x_1^4  4*t_x_1^3 3*t_x_1^2 2*t_x_1^1     1   0;
          42*t_x_0^5  30*t_x_0^4 20*t_x_0^3 12*t_x_0^2 6*t_x_0^1      2        0   0;
          42*t_x_1^5  30*t_x_1^4 20*t_x_1^3 12*t_x_1^2 6*t_x_1^1      2        0   0;
         210*t_x_0^4 120*t_x_0^3 60*t_x_0^2 24*t_x_0^1    6           0        0   0;
         210*t_x_1^4 120*t_x_1^3 60*t_x_1^2 24*t_x_1^1    6           0        0   0];
    
    k_x_1(j,:) = inv(T)*p_x_1;
    
    p_x_2(1) = pos_v(j,1) + 0.5*acc_t(j,1)*t_t_acc(j,1)^2;
    p_x_2(2) = pos_v(j,1) + 0.5*acc_t(j,1)*t_t_acc(j,1)^2 + v_k(j,1) * t_t_no_acc(j,1);
    p_x_2(3) = v_k(j,1);
    p_x_2(4) = v_k(j,1);
    p_x_2(5) = 0;
    p_x_2(6) = 0;
    p_x_2(7) = 0;
    p_x_2(8) = 0;
    if j == 1
        p_x_2 = p_x_2';
    end
    
    T = [    t_x_1^7     t_x_1^6    t_x_1^5    t_x_1^4   t_x_1^3   t_x_1^2 t_x_1^1 1;
             t_x_2^7     t_x_2^6    t_x_2^5    t_x_2^4   t_x_2^3   t_x_2^2 t_x_2^1 1;
           7*t_x_1^6   6*t_x_1^5  5*t_x_1^4  4*t_x_1^3 3*t_x_1^2 2*t_x_1^1     1   0;
           7*t_x_2^6   6*t_x_2^5  5*t_x_2^4  4*t_x_2^3 3*t_x_2^2 2*t_x_2^1     1   0;
          42*t_x_1^5  30*t_x_1^4 20*t_x_1^3 12*t_x_1^2 6*t_x_1^1      2        0   0;
          42*t_x_2^5  30*t_x_2^4 20*t_x_2^3 12*t_x_2^2 6*t_x_2^1      2        0   0;
         210*t_x_1^4 120*t_x_1^3 60*t_x_1^2 24*t_x_1^1    6           0        0   0;
         210*t_x_2^4 120*t_x_2^3 60*t_x_2^2 24*t_x_2^1    6           0        0   0];
    
    k_x_2(j,:) = inv(T)*p_x_2;
    
    p_x_3(1) = pos_v(j,1) + 0.5*acc_t(j,1)*t_t_acc(j,1)^2 + v_k(j,1) * t_t_no_acc(j,1);
    p_x_3(2) = pos_v(j+1,1);
    p_x_3(3) = v_k(j,1);
    p_x_3(4) = 0;
    p_x_3(5) = 0;
    p_x_3(6) = 0;
    p_x_3(7) = 0;
    p_x_3(8) = 0;
    if j == 1
        p_x_3 = p_x_3';
    end
    
    T = [    t_x_2^7     t_x_2^6    t_x_2^5    t_x_2^4   t_x_2^3   t_x_2^2 t_x_2^1 1;
             t_x_3^7     t_x_3^6    t_x_3^5    t_x_3^4   t_x_3^3   t_x_3^2 t_x_3^1 1;
           7*t_x_2^6   6*t_x_2^5  5*t_x_2^4  4*t_x_2^3 3*t_x_2^2 2*t_x_2^1     1   0;
           7*t_x_3^6   6*t_x_3^5  5*t_x_3^4  4*t_x_3^3 3*t_x_3^2 2*t_x_3^1     1   0;
          42*t_x_2^5  30*t_x_2^4 20*t_x_2^3 12*t_x_2^2 6*t_x_2^1      2        0   0;
          42*t_x_3^5  30*t_x_3^4 20*t_x_3^3 12*t_x_3^2 6*t_x_3^1      2        0   0;
         210*t_x_2^4 120*t_x_2^3 60*t_x_2^2 24*t_x_2^1    6           0        0   0;
         210*t_x_3^4 120*t_x_3^3 60*t_x_3^2 24*t_x_3^1    6           0        0   0];
    
    k_x_3(j,:) = inv(T)*p_x_3;
    
    for i = (j-1)*1000000+1:j*1000000
        if i == 1
            time(i)=0;
        else
            time(i) = time(i-1)+t_incr(j);
        end
        
        if time(i) <= T_v(j) + t_t_acc(j,1)
            x_d(i)   =    k_x_1(j,1)*time(i)^7 +    k_x_1(j,2)*time(i)^6 +    k_x_1(j,3)*time(i)^5 +    k_x_1(j,4)*time(i)^4 +   k_x_1(j,5)*time(i)^3 +   k_x_1(j,6)*time(i)^2 + k_x_1(j,7)*time(i)^1 + k_x_1(j,8);
            v_x_d(i) =  7*k_x_1(j,1)*time(i)^6 +  6*k_x_1(j,2)*time(i)^5 +  5*k_x_1(j,3)*time(i)^4 +  4*k_x_1(j,4)*time(i)^3 + 3*k_x_1(j,5)*time(i)^2 + 2*k_x_1(j,6)*time(i)^1 + k_x_1(j,7);
            a_x_d(i) = 42*k_x_1(j,1)*time(i)^5 + 30*k_x_1(j,2)*time(i)^4 + 20*k_x_1(j,3)*time(i)^3 + 12*k_x_1(j,4)*time(i)^2 + 6*k_x_1(j,5)*time(i)^1 + 2*k_x_1(j,6);
        elseif time(i) <= T_v(j) + t_t_acc(j,1)+t_t_no_acc(j,1)
            x_d(i)   =    k_x_2(j,1)*time(i)^7 +    k_x_2(j,2)*time(i)^6 +    k_x_2(j,3)*time(i)^5 +    k_x_2(j,4)*time(i)^4 +   k_x_2(j,5)*time(i)^3 +   k_x_2(j,6)*time(i)^2 + k_x_2(j,7)*time(i)^1 + k_x_2(j,8);
            v_x_d(i) =  7*k_x_2(j,1)*time(i)^6 +  6*k_x_2(j,2)*time(i)^5 +  5*k_x_2(j,3)*time(i)^4 +  4*k_x_2(j,4)*time(i)^3 + 3*k_x_2(j,5)*time(i)^2 + 2*k_x_2(j,6)*time(i)^1 + k_x_2(j,7);
            a_x_d(i) = 42*k_x_2(j,1)*time(i)^5 + 30*k_x_2(j,2)*time(i)^4 + 20*k_x_2(j,3)*time(i)^3 + 12*k_x_2(j,4)*time(i)^2 + 6*k_x_2(j,5)*time(i)^1 + 2*k_x_2(j,6);
        else
            x_d(i)   =    k_x_3(j,1)*time(i)^7 +    k_x_3(j,2)*time(i)^6 +    k_x_3(j,3)*time(i)^5 +    k_x_3(j,4)*time(i)^4 +   k_x_3(j,5)*time(i)^3 +   k_x_3(j,6)*time(i)^2 + k_x_3(j,7)*time(i)^1 + k_x_3(j,8);
            v_x_d(i) =  7*k_x_3(j,1)*time(i)^6 +  6*k_x_3(j,2)*time(i)^5 +  5*k_x_3(j,3)*time(i)^4 +  4*k_x_3(j,4)*time(i)^3 + 3*k_x_3(j,5)*time(i)^2 + 2*k_x_3(j,6)*time(i)^1 + k_x_3(j,7);
            a_x_d(i) = 42*k_x_3(j,1)*time(i)^5 + 30*k_x_3(j,2)*time(i)^4 + 20*k_x_3(j,3)*time(i)^3 + 12*k_x_3(j,4)*time(i)^2 + 6*k_x_3(j,5)*time(i)^1 + 2*k_x_3(j,6);
        end
    end
end

figure('Name','x Axis Equations of Motion');
subplot(3,1,1);
plot(time,x_d);
title('Position');
xlabel('t [sec]');
ylabel('x [m]');
subplot(3,1,2);
plot(time,v_x_d);
title('Velocity');
xlabel('t [sec]');
ylabel('v [m/sec]');
subplot(3,1,3);
plot(time,a_x_d);
title('Acceleration');
xlabel('t [sec]');
ylabel('a [m/sec^2]');

% y Calculations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for j = 1:n
    t_y_0 = t_t_y_0(j);
    t_y_1 = t_t_y_1(j);
    t_y_2 = t_t_y_2(j);
    t_y_3 = t_t_y_3(j);
    
    p_y_1(1) = pos_v(j,2);
    p_y_1(2) = pos_v(j,2) + 0.5*acc_t(j,2)*t_t_acc(j,2)^2;
    p_y_1(3) = 0;
    p_y_1(4) = v_k(j,2);
    p_y_1(5) = 0;
    p_y_1(6) = 0;
    p_y_1(7) = 0;
    p_y_1(8) = 0;
    if j == 1
        p_y_1 = p_y_1';
    end
    
    T = [    t_y_0^7     t_y_0^6    t_y_0^5    t_y_0^4   t_y_0^3   t_y_0^2 t_y_0^1 1;
             t_y_1^7     t_y_1^6    t_y_1^5    t_y_1^4   t_y_1^3   t_y_1^2 t_y_1^1 1;
           7*t_y_0^6   6*t_y_0^5  5*t_y_0^4  4*t_y_0^3 3*t_y_0^2 2*t_y_0^1     1   0;
           7*t_y_1^6   6*t_y_1^5  5*t_y_1^4  4*t_y_1^3 3*t_y_1^2 2*t_y_1^1     1   0;
          42*t_y_0^5  30*t_y_0^4 20*t_y_0^3 12*t_y_0^2 6*t_y_0^1      2        0   0;
          42*t_y_1^5  30*t_y_1^4 20*t_y_1^3 12*t_y_1^2 6*t_y_1^1      2        0   0;
         210*t_y_0^4 120*t_y_0^3 60*t_y_0^2 24*t_y_0^1    6           0        0   0;
         210*t_y_1^4 120*t_y_1^3 60*t_y_1^2 24*t_y_1^1    6           0        0   0];
    
    k_y_1(j,:) = inv(T)*p_y_1;
    
    p_y_2(1) = pos_v(j,2) + 0.5*acc_t(j,2)*t_t_acc(j,2)^2;
    p_y_2(2) = pos_v(j,2) + 0.5*acc_t(j,2)*t_t_acc(j,2)^2 + v_k(j,2) * t_t_no_acc(j,2);
    p_y_2(3) = v_k(j,2);
    p_y_2(4) = v_k(j,2);
    p_y_2(5) = 0;
    p_y_2(6) = 0;
    p_y_2(7) = 0;
    p_y_2(8) = 0;
    if j == 1
        p_y_2 = p_y_2';
    end
    
    T = [    t_y_1^7     t_y_1^6    t_y_1^5    t_y_1^4   t_y_1^3   t_y_1^2 t_y_1^1 1;
             t_y_2^7     t_y_2^6    t_y_2^5    t_y_2^4   t_y_2^3   t_y_2^2 t_y_2^1 1;
           7*t_y_1^6   6*t_y_1^5  5*t_y_1^4  4*t_y_1^3 3*t_y_1^2 2*t_y_1^1     1   0;
           7*t_y_2^6   6*t_y_2^5  5*t_y_2^4  4*t_y_2^3 3*t_y_2^2 2*t_y_2^1     1   0;
          42*t_y_1^5  30*t_y_1^4 20*t_y_1^3 12*t_y_1^2 6*t_y_1^1      2        0   0;
          42*t_y_2^5  30*t_y_2^4 20*t_y_2^3 12*t_y_2^2 6*t_y_2^1      2        0   0;
         210*t_y_1^4 120*t_y_1^3 60*t_y_1^2 24*t_y_1^1    6           0        0   0;
         210*t_y_2^4 120*t_y_2^3 60*t_y_2^2 24*t_y_2^1    6           0        0   0];
    
    k_y_2(j,:) = inv(T)*p_y_2;
    
    p_y_3(1) = pos_v(j,2) + 0.5*acc_t(j,2)*t_t_acc(j,2)^2 + v_k(j,2) * t_t_no_acc(j,2);
    p_y_3(2) = pos_v(j+1,2);
    p_y_3(3) = v_k(j,2);
    p_y_3(4) = 0;
    p_y_3(5) = 0;
    p_y_3(6) = 0;
    p_y_3(7) = 0;
    p_y_3(8) = 0;
    if j == 1
        p_y_3 = p_y_3';
    end
    
    T = [    t_y_2^7     t_y_2^6    t_y_2^5    t_y_2^4   t_y_2^3   t_y_2^2 t_y_2^1 1;
             t_y_3^7     t_y_3^6    t_y_3^5    t_y_3^4   t_y_3^3   t_y_3^2 t_y_3^1 1;
           7*t_y_2^6   6*t_y_2^5  5*t_y_2^4  4*t_y_2^3 3*t_y_2^2 2*t_y_2^1     1   0;
           7*t_y_3^6   6*t_y_3^5  5*t_y_3^4  4*t_y_3^3 3*t_y_3^2 2*t_y_3^1     1   0;
          42*t_y_2^5  30*t_y_2^4 20*t_y_2^3 12*t_y_2^2 6*t_y_2^1      2        0   0;
          42*t_y_3^5  30*t_y_3^4 20*t_y_3^3 12*t_y_3^2 6*t_y_3^1      2        0   0;
         210*t_y_2^4 120*t_y_2^3 60*t_y_2^2 24*t_y_2^1    6           0        0   0;
         210*t_y_3^4 120*t_y_3^3 60*t_y_3^2 24*t_y_3^1    6           0        0   0];
    
    k_y_3(j,:) = inv(T)*p_y_3;
    
    for i = (j-1)*1000000+1:j*1000000
        if i == 1
            time(i)=0;
        else
            time(i) = time(i-1) + t_incr(j);
        end
        
        if time(i) <= T_v(j) + t_t_acc(j,2)
            y_d(i) =      k_y_1(j,1)*time(i)^7 +    k_y_1(j,2)*time(i)^6 +    k_y_1(j,3)*time(i)^5 +    k_y_1(j,4)*time(i)^4 +   k_y_1(j,5)*time(i)^3 +   k_y_1(j,6)*time(i)^2 + k_y_1(j,7)*time(i)^1 + k_y_1(j,8);
            v_y_d(i) =  7*k_y_1(j,1)*time(i)^6 +  6*k_y_1(j,2)*time(i)^5 +  5*k_y_1(j,3)*time(i)^4 +  4*k_y_1(j,4)*time(i)^3 + 3*k_y_1(j,5)*time(i)^2 + 2*k_y_1(j,6)*time(i)^1 + k_y_1(j,7);
            a_y_d(i) = 42*k_y_1(j,1)*time(i)^5 + 30*k_y_1(j,2)*time(i)^4 + 20*k_y_1(j,3)*time(i)^3 + 12*k_y_1(j,4)*time(i)^2 + 6*k_y_1(j,5)*time(i)^1 + 2*k_y_1(j,6);
        elseif time(i) <= T_v(j) + t_t_acc(j,2)+t_t_no_acc(j,2)
            y_d(i) =      k_y_2(j,1)*time(i)^7 +    k_y_2(j,2)*time(i)^6 +    k_y_2(j,3)*time(i)^5 +    k_y_2(j,4)*time(i)^4 +   k_y_2(j,5)*time(i)^3 +   k_y_2(j,6)*time(i)^2 + k_y_2(j,7)*time(i)^1 + k_y_2(j,8);
            v_y_d(i) =  7*k_y_2(j,1)*time(i)^6 +  6*k_y_2(j,2)*time(i)^5 +  5*k_y_2(j,3)*time(i)^4 +  4*k_y_2(j,4)*time(i)^3 + 3*k_y_2(j,5)*time(i)^2 + 2*k_y_2(j,6)*time(i)^1 + k_y_2(j,7);
            a_y_d(i) = 42*k_y_2(j,1)*time(i)^5 + 30*k_y_2(j,2)*time(i)^4 + 20*k_y_2(j,3)*time(i)^3 + 12*k_y_2(j,4)*time(i)^2 + 6*k_y_2(j,5)*time(i)^1 + 2*k_y_2(j,6);
        else
            y_d(i) =      k_y_3(j,1)*time(i)^7 +    k_y_3(j,2)*time(i)^6 +    k_y_3(j,3)*time(i)^5 +    k_y_3(j,4)*time(i)^4 +   k_y_3(j,5)*time(i)^3 +   k_y_3(j,6)*time(i)^2 + k_y_3(j,7)*time(i)^1 + k_y_3(j,8);
            v_y_d(i) =  7*k_y_3(j,1)*time(i)^6 +  6*k_y_3(j,2)*time(i)^5 +  5*k_y_3(j,3)*time(i)^4 +  4*k_y_3(j,4)*time(i)^3 + 3*k_y_3(j,5)*time(i)^2 + 2*k_y_3(j,6)*time(i)^1 + k_y_3(j,7);
            a_y_d(i) = 42*k_y_3(j,1)*time(i)^5 + 30*k_y_3(j,2)*time(i)^4 + 20*k_y_3(j,3)*time(i)^3 + 12*k_y_3(j,4)*time(i)^2 + 6*k_y_3(j,5)*time(i)^1 + 2*k_y_3(j,6);
        end
    end
end

figure('Name','y Axis Equations of Motion');
subplot(3,1,1);
plot(time,y_d);
title('Position');
xlabel('t [sec]');
ylabel('x [m]');
subplot(3,1,2);
plot(time,v_y_d);
title('Velocity');
xlabel('t [sec]');
ylabel('v [m/sec]');
subplot(3,1,3);
plot(time,a_y_d);
title('Acceleration');
xlabel('t [sec]');
ylabel('a [m/sec^2]');

% phi Calculations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for j = 1:n
    t_phi_0 = t_t_phi_0(j);
    t_phi_1 = t_t_phi_1(j);
    t_phi_2 = t_t_phi_2(j);
    t_phi_3 = t_t_phi_3(j);
    
    p_phi_1(1) = euler_v(j,1);
    p_phi_1(2) = euler_v(j,1) + 0.5*acc(j,1)*t_acc(j,1)^2;
    p_phi_1(3) = 0;
    p_phi_1(4) = w_k(j,1);
    p_phi_1(5) = 0;
    p_phi_1(6) = 0;
    p_phi_1(7) = 0;
    p_phi_1(8) = 0;
    if j == 1
        p_phi_1 = p_phi_1';
    end
    
    T = [    t_phi_0^7     t_phi_0^6    t_phi_0^5    t_phi_0^4   t_phi_0^3   t_phi_0^2 t_phi_0^1 1;
             t_phi_1^7     t_phi_1^6    t_phi_1^5    t_phi_1^4   t_phi_1^3   t_phi_1^2 t_phi_1^1 1;
           7*t_phi_0^6   6*t_phi_0^5  5*t_phi_0^4  4*t_phi_0^3 3*t_phi_0^2 2*t_phi_0^1     1     0;
           7*t_phi_1^6   6*t_phi_1^5  5*t_phi_1^4  4*t_phi_1^3 3*t_phi_1^2 2*t_phi_1^1     1     0;
          42*t_phi_0^5  30*t_phi_0^4 20*t_phi_0^3 12*t_phi_0^2 6*t_phi_0^1      2          0     0;
          42*t_phi_1^5  30*t_phi_1^4 20*t_phi_1^3 12*t_phi_1^2 6*t_phi_1^1      2          0     0;
         210*t_phi_0^4 120*t_phi_0^3 60*t_phi_0^2 24*t_phi_0^1      6           0          0     0;
         210*t_phi_1^4 120*t_phi_1^3 60*t_phi_1^2 24*t_phi_1^1      6           0          0     0];
    
    k_phi_1(j,:) = inv(T)*p_phi_1;
    
    p_phi_2(1) = euler_v(j,1) + 0.5*acc(j,1)*t_acc(j,1)^2;
    p_phi_2(2) = euler_v(j,1) + 0.5*acc(j,1)*t_acc(j,1)^2 + w_k(j,1) * t_no_acc(j,1);
    p_phi_2(3) = w_k(j,1);
    p_phi_2(4) = w_k(j,1);
    p_phi_2(5) = 0;
    p_phi_2(6) = 0;
    p_phi_2(7) = 0;
    p_phi_2(8) = 0;
    if j == 1
        p_phi_2 = p_phi_2';
    end
    
    T = [    t_phi_1^7     t_phi_1^6    t_phi_1^5    t_phi_1^4   t_phi_1^3   t_phi_1^2 t_phi_1^1 1;
             t_phi_2^7     t_phi_2^6    t_phi_2^5    t_phi_2^4   t_phi_2^3   t_phi_2^2 t_phi_2^1 1;
           7*t_phi_1^6   6*t_phi_1^5  5*t_phi_1^4  4*t_phi_1^3 3*t_phi_1^2 2*t_phi_1^1     1     0;
           7*t_phi_2^6   6*t_phi_2^5  5*t_phi_2^4  4*t_phi_2^3 3*t_phi_2^2 2*t_phi_2^1     1     0;
          42*t_phi_1^5  30*t_phi_1^4 20*t_phi_1^3 12*t_phi_1^2 6*t_phi_1^1      2          0     0;
          42*t_phi_2^5  30*t_phi_2^4 20*t_phi_2^3 12*t_phi_2^2 6*t_phi_2^1      2          0     0;
         210*t_phi_1^4 120*t_phi_1^3 60*t_phi_1^2 24*t_phi_1^1      6           0          0     0;
         210*t_phi_2^4 120*t_phi_2^3 60*t_phi_2^2 24*t_phi_2^1      6           0          0     0];
     
    k_phi_2(j,:) = inv(T)*p_phi_2;
    
    p_phi_3(1) = euler_v(j,1) + 0.5*acc(j,1)*t_acc(j,1)^2 + w_k(j,1) * t_no_acc(j,1);
    p_phi_3(2) = euler_v(j+1,1);
    p_phi_3(3) = w_k(j,1);
    p_phi_3(4) = 0;
    p_phi_3(5) = 0;
    p_phi_3(6) = 0;
    p_phi_3(7) = 0;
    p_phi_3(8) = 0;
    if j == 1
        p_phi_3 = p_phi_3';
    end
    
    T = [    t_phi_2^7     t_phi_2^6    t_phi_2^5    t_phi_2^4   t_phi_2^3   t_phi_2^2 t_phi_2^1 1;
             t_phi_3^7     t_phi_3^6    t_phi_3^5    t_phi_3^4   t_phi_3^3   t_phi_3^2 t_phi_3^1 1;
           7*t_phi_2^6   6*t_phi_2^5  5*t_phi_2^4  4*t_phi_2^3 3*t_phi_2^2 2*t_phi_2^1     1     0;
           7*t_phi_3^6   6*t_phi_3^5  5*t_phi_3^4  4*t_phi_3^3 3*t_phi_3^2 2*t_phi_3^1     1     0;
          42*t_phi_2^5  30*t_phi_2^4 20*t_phi_2^3 12*t_phi_2^2 6*t_phi_2^1      2          0     0;
          42*t_phi_3^5  30*t_phi_3^4 20*t_phi_3^3 12*t_phi_3^2 6*t_phi_3^1      2          0     0;
         210*t_phi_2^4 120*t_phi_2^3 60*t_phi_2^2 24*t_phi_2^1      6           0          0     0;
         210*t_phi_3^4 120*t_phi_3^3 60*t_phi_3^2 24*t_phi_3^1      6           0          0     0];
    
    k_phi_3(j,:) = inv(T)*p_phi_3;
    
    for i = (j-1)*1000000+1:j*1000000
        if i==1
            time(i) = 0;
        else
            time(i) = time(i-1) + t_incr(j);
        end
        
        if time(i) <= T_v(j) + t_acc(j,1)
            phi_d(i) =    k_phi_1(j,1)*time(i)^7 +    k_phi_1(j,2)*time(i)^6 +    k_phi_1(j,3)*time(i)^5 +    k_phi_1(j,4)*time(i)^4 +   k_phi_1(j,5)*time(i)^3 +   k_phi_1(j,6)*time(i)^2 + k_phi_1(j,7)*time(i)^1 + k_phi_1(j,8);
            w_phi(i) =  7*k_phi_1(j,1)*time(i)^6 +  6*k_phi_1(j,2)*time(i)^5 +  5*k_phi_1(j,3)*time(i)^4 +  4*k_phi_1(j,4)*time(i)^3 + 3*k_phi_1(j,5)*time(i)^2 + 2*k_phi_1(j,6)*time(i)^1 + k_phi_1(j,7);
            a_phi(i) = 42*k_phi_1(j,1)*time(i)^5 + 30*k_phi_1(j,2)*time(i)^4 + 20*k_phi_1(j,3)*time(i)^3 + 12*k_phi_1(j,4)*time(i)^2 + 6*k_phi_1(j,5)*time(i)^1 + 2*k_phi_1(j,6);
        elseif time(i) <= T_v(j) + t_acc(j,1)+t_no_acc(j,1)
            phi_d(i) =    k_phi_2(j,1)*time(i)^7 +    k_phi_2(j,2)*time(i)^6 +    k_phi_2(j,3)*time(i)^5 +    k_phi_2(j,4)*time(i)^4 +   k_phi_2(j,5)*time(i)^3 +   k_phi_2(j,6)*time(i)^2 + k_phi_2(j,7)*time(i)^1 + k_phi_2(j,8);
            w_phi(i) =  7*k_phi_2(j,1)*time(i)^6 +  6*k_phi_2(j,2)*time(i)^5 +  5*k_phi_2(j,3)*time(i)^4 +  4*k_phi_2(j,4)*time(i)^3 + 3*k_phi_2(j,5)*time(i)^2 + 2*k_phi_2(j,6)*time(i)^1 + k_phi_2(j,7);
            a_phi(i) = 42*k_phi_2(j,1)*time(i)^5 + 30*k_phi_2(j,2)*time(i)^4 + 20*k_phi_2(j,3)*time(i)^3 + 12*k_phi_2(j,4)*time(i)^2 + 6*k_phi_2(j,5)*time(i)^1 + 2*k_phi_2(j,6);
        else
            phi_d(i) =    k_phi_3(j,1)*time(i)^7 +    k_phi_3(j,2)*time(i)^6 +    k_phi_3(j,3)*time(i)^5 +    k_phi_3(j,4)*time(i)^4 +   k_phi_3(j,5)*time(i)^3 +   k_phi_3(j,6)*time(i)^2 + k_phi_3(j,7)*time(i)^1 + k_phi_3(j,8);
            w_phi(i) =  7*k_phi_3(j,1)*time(i)^6 +  6*k_phi_3(j,2)*time(i)^5 +  5*k_phi_3(j,3)*time(i)^4 +  4*k_phi_3(j,4)*time(i)^3 + 3*k_phi_3(j,5)*time(i)^2 + 2*k_phi_3(j,6)*time(i)^1 + k_phi_3(j,7);
            a_phi(i) = 42*k_phi_3(j,1)*time(i)^5 + 30*k_phi_3(j,2)*time(i)^4 + 20*k_phi_3(j,3)*time(i)^3 + 12*k_phi_3(j,4)*time(i)^2 + 6*k_phi_3(j,5)*time(i)^1 + 2*k_phi_3(j,6);
        end
    end
end

figure('Name','phi Euler Angle Equations of Motion');
subplot(3,1,1);
plot(time,phi_d);
title('Angle');
xlabel('t [sec]');
ylabel('Angle [rad]');
subplot(3,1,2);
plot(time,w_phi);
title('Angular Velocity');
xlabel('t [sec]');
ylabel('w [rad/sec]');
subplot(3,1,3);
plot(time,a_phi);
title('Angular Acceleration');
xlabel('t [sec]');
ylabel('a [rad/sec^2]');

% theta Calculations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for j = 1:n
    
    t_theta_0 = t_t_theta_0(j);
    t_theta_1 = t_t_theta_1(j);
    t_theta_2 = t_t_theta_2(j);
    t_theta_3 = t_t_theta_3(j);
    
    p_theta_1(1) = euler_v(j,2);
    p_theta_1(2) = euler_v(j,2) + 0.5*acc(j,2)*t_acc(j,2)^2;
    p_theta_1(3) = 0;
    p_theta_1(4) = w_k(j,2);
    p_theta_1(5) = 0;
    p_theta_1(6) = 0;
    p_theta_1(7) = 0;
    p_theta_1(8) = 0;
    if j == 1
        p_theta_1 = p_theta_1';
    end
    
    T = [    t_theta_0^7     t_theta_0^6    t_theta_0^5    t_theta_0^4   t_theta_0^3   t_theta_0^2 t_theta_0^1 1;
             t_theta_1^7     t_theta_1^6    t_theta_1^5    t_theta_1^4   t_theta_1^3   t_theta_1^2 t_theta_1^1 1;
           7*t_theta_0^6   6*t_theta_0^5  5*t_theta_0^4  4*t_theta_0^3 3*t_theta_0^2 2*t_theta_0^1     1     0;
           7*t_theta_1^6   6*t_theta_1^5  5*t_theta_1^4  4*t_theta_1^3 3*t_theta_1^2 2*t_theta_1^1     1     0;
          42*t_theta_0^5  30*t_theta_0^4 20*t_theta_0^3 12*t_theta_0^2 6*t_theta_0^1       2           0     0;
          42*t_theta_1^5  30*t_theta_1^4 20*t_theta_1^3 12*t_theta_1^2 6*t_theta_1^1       2           0     0;
         210*t_theta_0^4 120*t_theta_0^3 60*t_theta_0^2 24*t_theta_0^1        6            0           0     0;
         210*t_theta_1^4 120*t_theta_1^3 60*t_theta_1^2 24*t_theta_1^1        6            0           0     0];
    
    k_theta_1(j,:) = inv(T)*p_theta_1;
    
    p_theta_2(1) = euler_v(j,2) + 0.5*acc(j,2)*t_acc(j,2)^2;
    p_theta_2(2) = euler_v(j,2) + 0.5*acc(j,2)*t_acc(j,2)^2 + w_k(j,2) * t_no_acc(j,2);
    p_theta_2(3) = w_k(j,2);
    p_theta_2(4) = w_k(j,2);
    p_theta_2(5) = 0;
    p_theta_2(6) = 0;
    p_theta_2(7) = 0;
    p_theta_2(8) = 0;
    if j == 1
        p_theta_2 = p_theta_2';
    end
    
    T = [    t_theta_1^7     t_theta_1^6    t_theta_1^5    t_theta_1^4   t_theta_1^3   t_theta_1^2 t_theta_1^1 1;
             t_theta_2^7     t_theta_2^6    t_theta_2^5    t_theta_2^4   t_theta_2^3   t_theta_2^2 t_theta_2^1 1;
           7*t_theta_1^6   6*t_theta_1^5  5*t_theta_1^4  4*t_theta_1^3 3*t_theta_1^2 2*t_theta_1^1     1     0;
           7*t_theta_2^6   6*t_theta_2^5  5*t_theta_2^4  4*t_theta_2^3 3*t_theta_2^2 2*t_theta_2^1     1     0;
          42*t_theta_1^5  30*t_theta_1^4 20*t_theta_1^3 12*t_theta_1^2 6*t_theta_1^1       2           0     0;
          42*t_theta_2^5  30*t_theta_2^4 20*t_theta_2^3 12*t_theta_2^2 6*t_theta_2^1       2           0     0;
         210*t_theta_1^4 120*t_theta_1^3 60*t_theta_1^2 24*t_theta_1^1        6            0           0     0;
         210*t_theta_2^4 120*t_theta_2^3 60*t_theta_2^2 24*t_theta_2^1        6            0           0     0];
    
    k_theta_2(j,:) = inv(T)*p_theta_2;
    
    p_theta_3(1) = euler_v(j,2) + 0.5*acc(j,2)*t_acc(j,2)^2 + w_k(j,2) * t_no_acc(j,2);
    p_theta_3(2) = euler_v(j+1,2);
    p_theta_3(3) = w_k(j,2);
    p_theta_3(4) = 0;
    p_theta_3(5) = 0;
    p_theta_3(6) = 0;
    p_theta_3(7) = 0;
    p_theta_3(8) = 0;
    if j == 1
        p_theta_3 = p_theta_3';
    end
    
    T = [    t_theta_2^7     t_theta_2^6    t_theta_2^5    t_theta_2^4   t_theta_2^3   t_theta_2^2 t_theta_2^1 1;
             t_theta_3^7     t_theta_3^6    t_theta_3^5    t_theta_3^4   t_theta_3^3   t_theta_3^2 t_theta_3^1 1;
           7*t_theta_2^6   6*t_theta_2^5  5*t_theta_2^4  4*t_theta_2^3 3*t_theta_2^2 2*t_theta_2^1     1     0;
           7*t_theta_3^6   6*t_theta_3^5  5*t_theta_3^4  4*t_theta_3^3 3*t_theta_3^2 2*t_theta_3^1     1     0;
          42*t_theta_2^5  30*t_theta_2^4 20*t_theta_2^3 12*t_theta_2^2 6*t_theta_2^1       2           0     0;
          42*t_theta_3^5  30*t_theta_3^4 20*t_theta_3^3 12*t_theta_3^2 6*t_theta_3^1       2           0     0;
         210*t_theta_2^4 120*t_theta_2^3 60*t_theta_2^2 24*t_theta_2^1        6            0           0     0;
         210*t_theta_3^4 120*t_theta_3^3 60*t_theta_3^2 24*t_theta_3^1        6            0           0     0];
    
    k_theta_3(j,:) = inv(T)*p_theta_3;
    
    for i = (j-1)*1000000+1:j*1000000
        if i==1
            time(i)=0;
        else
            time(i) = time(i-1)+t_incr(j);
        end
        
        if time(i) <= T_v(j) + t_acc(j,2)
            theta_d(i) =    k_theta_1(j,1)*time(i)^7 +    k_theta_1(j,2)*time(i)^6 +    k_theta_1(j,3)*time(i)^5 +    k_theta_1(j,4)*time(i)^4 +   k_theta_1(j,5)*time(i)^3 +   k_theta_1(j,6)*time(i)^2 + k_theta_1(j,7)*time(i)^1 + k_theta_1(j,8);
            w_theta(i) =  7*k_theta_1(j,1)*time(i)^6 +  6*k_theta_1(j,2)*time(i)^5 +  5*k_theta_1(j,3)*time(i)^4 +  4*k_theta_1(j,4)*time(i)^3 + 3*k_theta_1(j,5)*time(i)^2 + 2*k_theta_1(j,6)*time(i)^1 + k_theta_1(j,7);
            a_theta(i) = 42*k_theta_1(j,1)*time(i)^5 + 30*k_theta_1(j,2)*time(i)^4 + 20*k_theta_1(j,3)*time(i)^3 + 12*k_theta_1(j,4)*time(i)^2 + 6*k_theta_1(j,5)*time(i)^1 + 2*k_theta_1(j,6);
        elseif time(i) <= T_v(j) + t_acc(j,2)+t_no_acc(j,2)
            theta_d(i) =    k_theta_2(j,1)*time(i)^7 +    k_theta_2(j,2)*time(i)^6 +    k_theta_2(j,3)*time(i)^5 +    k_theta_2(j,4)*time(i)^4 +   k_theta_2(j,5)*time(i)^3 +   k_theta_2(j,6)*time(i)^2 + k_theta_2(j,7)*time(i)^1 + k_theta_2(j,8);
            w_theta(i) =  7*k_theta_2(j,1)*time(i)^6 +  6*k_theta_2(j,2)*time(i)^5 +  5*k_theta_2(j,3)*time(i)^4 +  4*k_theta_2(j,4)*time(i)^3 + 3*k_theta_2(j,5)*time(i)^2 + 2*k_theta_2(j,6)*time(i)^1 + k_theta_2(j,7);
            a_theta(i) = 42*k_theta_2(j,1)*time(i)^5 + 30*k_theta_2(j,2)*time(i)^4 + 20*k_theta_2(j,3)*time(i)^3 + 12*k_theta_2(j,4)*time(i)^2 + 6*k_theta_2(j,5)*time(i)^1 + 2*k_theta_2(j,6);
        else
            theta_d(i) =    k_theta_3(j,1)*time(i)^7 +    k_theta_3(j,2)*time(i)^6 +    k_theta_3(j,3)*time(i)^5 +    k_theta_3(j,4)*time(i)^4 +   k_theta_3(j,5)*time(i)^3 +   k_theta_3(j,6)*time(i)^2 + k_theta_3(j,7)*time(i)^1 + k_theta_3(j,8);
            w_theta(i) =  7*k_theta_3(j,1)*time(i)^6 +  6*k_theta_3(j,2)*time(i)^5 +  5*k_theta_3(j,3)*time(i)^4 +  4*k_theta_3(j,4)*time(i)^3 + 3*k_theta_3(j,5)*time(i)^2 + 2*k_theta_3(j,6)*time(i)^1 + k_theta_3(j,7);
            a_theta(i) = 42*k_theta_3(j,1)*time(i)^5 + 30*k_theta_3(j,2)*time(i)^4 + 20*k_theta_3(j,3)*time(i)^3 + 12*k_theta_3(j,4)*time(i)^2 + 6*k_theta_3(j,5)*time(i)^1 + 2*k_theta_3(j,6);
        end
    end
end
    
figure('Name','theta Euler Angle Equations of Motion');
subplot(3,1,1);
plot(time,theta_d);
title('Angle');
xlabel('t [sec]');
ylabel('Angle [rad]');
subplot(3,1,2);
plot(time,w_theta);
title('Angular Velocity');
xlabel('t [sec]');
ylabel('w [rad/sec]');
subplot(3,1,3);
plot(time,a_theta);
title('Angular Acceleration');
xlabel('t [sec]');
ylabel('a [rad/sec^2]');


% psi Calculations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for j = 1:n
    
    t_psi_0 = t_t_psi_0(j);
    t_psi_1 = t_t_psi_1(j);
    t_psi_2 = t_t_psi_2(j);
    t_psi_3 = t_t_psi_3(j);
    
    p_psi_1(1) = euler_v(j,3);
    p_psi_1(2) = euler_v(j,3) + 0.5*acc(j,3)*t_acc(j,3)^2;
    p_psi_1(3) = 0;
    p_psi_1(4) = w_k(j,3);
    p_psi_1(5) = 0;
    p_psi_1(6) = 0;
    p_psi_1(7) = 0;
    p_psi_1(8) = 0;
    if j == 1
        p_psi_1 = p_psi_1';
    end
    
    T = [    t_psi_0^7     t_psi_0^6    t_psi_0^5    t_psi_0^4   t_psi_0^3   t_psi_0^2 t_psi_0^1 1;
             t_psi_1^7     t_psi_1^6    t_psi_1^5    t_psi_1^4   t_psi_1^3   t_psi_1^2 t_psi_1^1 1;
           7*t_psi_0^6   6*t_psi_0^5  5*t_psi_0^4  4*t_psi_0^3 3*t_psi_0^2 2*t_psi_0^1     1     0;
           7*t_psi_1^6   6*t_psi_1^5  5*t_psi_1^4  4*t_psi_1^3 3*t_psi_1^2 2*t_psi_1^1     1     0;
          42*t_psi_0^5  30*t_psi_0^4 20*t_psi_0^3 12*t_psi_0^2 6*t_psi_0^1      2          0     0;
          42*t_psi_1^5  30*t_psi_1^4 20*t_psi_1^3 12*t_psi_1^2 6*t_psi_1^1      2          0     0;
         210*t_psi_0^4 120*t_psi_0^3 60*t_psi_0^2 24*t_psi_0^1      6           0          0     0;
         210*t_psi_1^4 120*t_psi_1^3 60*t_psi_1^2 24*t_psi_1^1      6           0          0     0];
    
    k_psi_1(j,:) = inv(T)*p_psi_1;
    
    p_psi_2(1) = euler_v(j,3) + 0.5*acc(j,3)*t_acc(j,3)^2;
    p_psi_2(2) = euler_v(j,3) + 0.5*acc(j,3)*t_acc(j,3)^2 + w_k(j,3) * t_no_acc(j,3);
    p_psi_2(3) = w_k(j,3);
    p_psi_2(4) = w_k(j,3);
    p_psi_2(5) = 0;
    p_psi_2(6) = 0;
    p_psi_2(7) = 0;
    p_psi_2(8) = 0;
    if j == 1
        p_psi_2 = p_psi_2';
    end
    
    T = [    t_psi_1^7     t_psi_1^6    t_psi_1^5    t_psi_1^4   t_psi_1^3   t_psi_1^2 t_psi_1^1 1;
             t_psi_2^7     t_psi_2^6    t_psi_2^5    t_psi_2^4   t_psi_2^3   t_psi_2^2 t_psi_2^1 1;
           7*t_psi_1^6   6*t_psi_1^5  5*t_psi_1^4  4*t_psi_1^3 3*t_psi_1^2 2*t_psi_1^1     1     0;
           7*t_psi_2^6   6*t_psi_2^5  5*t_psi_2^4  4*t_psi_2^3 3*t_psi_2^2 2*t_psi_2^1     1     0;
          42*t_psi_1^5  30*t_psi_1^4 20*t_psi_1^3 12*t_psi_1^2 6*t_psi_1^1      2          0     0;
          42*t_psi_2^5  30*t_psi_2^4 20*t_psi_2^3 12*t_psi_2^2 6*t_psi_2^1      2          0     0;
         210*t_psi_1^4 120*t_psi_1^3 60*t_psi_1^2 24*t_psi_1^1      6           0          0     0;
         210*t_psi_2^4 120*t_psi_2^3 60*t_psi_2^2 24*t_psi_2^1      6           0          0     0];
    
    k_psi_2(j,:) = inv(T)*p_psi_2;
    
    p_psi_3(1) = euler_v(j,3) + 0.5*acc(j,3)*t_acc(j,3)^2 + w_k(j,3) * t_no_acc(j,3);
    p_psi_3(2) = euler_v(j+1,3);
    p_psi_3(3) = w_k(j,3);
    p_psi_3(4) = 0;
    p_psi_3(5) = 0;
    p_psi_3(6) = 0;
    p_psi_3(7) = 0;
    p_psi_3(8) = 0;
    if j == 1
        p_psi_3 = p_psi_3';
    end
    
    T = [    t_psi_2^7     t_psi_2^6    t_psi_2^5    t_psi_2^4   t_psi_2^3   t_psi_2^2 t_psi_2^1 1;
             t_psi_3^7     t_psi_3^6    t_psi_3^5    t_psi_3^4   t_psi_3^3   t_psi_3^2 t_psi_3^1 1;
           7*t_psi_2^6   6*t_psi_2^5  5*t_psi_2^4  4*t_psi_2^3 3*t_psi_2^2 2*t_psi_2^1     1     0;
           7*t_psi_3^6   6*t_psi_3^5  5*t_psi_3^4  4*t_psi_3^3 3*t_psi_3^2 2*t_psi_3^1     1     0;
          42*t_psi_2^5  30*t_psi_2^4 20*t_psi_2^3 12*t_psi_2^2 6*t_psi_2^1      2          0     0;
          42*t_psi_3^5  30*t_psi_3^4 20*t_psi_3^3 12*t_psi_3^2 6*t_psi_3^1      2          0     0;
         210*t_psi_2^4 120*t_psi_2^3 60*t_psi_2^2 24*t_psi_2^1      6           0          0     0;
         210*t_psi_3^4 120*t_psi_3^3 60*t_psi_3^2 24*t_psi_3^1      6           0          0     0];
    
    k_psi_3(j,:) = inv(T)*p_psi_3;
    
    for i = (j-1)*1000000+1:j*1000000
        if i==1
            time(i)=0;
        else
            time(i) = time(i-1)+t_incr(j);
        end
        
        if time(i) <= T_v(j) + t_acc(j,3)
            psi_d(i) =    k_psi_1(j,1)*time(i)^7 +    k_psi_1(j,2)*time(i)^6 +    k_psi_1(j,3)*time(i)^5 +    k_psi_1(j,4)*time(i)^4 +   k_psi_1(j,5)*time(i)^3 +   k_psi_1(j,6)*time(i)^2 + k_psi_1(j,7)*time(i)^1 + k_psi_1(j,8);
            w_psi(i) =  7*k_psi_1(j,1)*time(i)^6 +  6*k_psi_1(j,2)*time(i)^5 +  5*k_psi_1(j,3)*time(i)^4 +  4*k_psi_1(j,4)*time(i)^3 + 3*k_psi_1(j,5)*time(i)^2 + 2*k_psi_1(j,6)*time(i)^1 + k_psi_1(j,7);
            a_psi(i) = 42*k_psi_1(j,1)*time(i)^5 + 30*k_psi_1(j,2)*time(i)^4 + 20*k_psi_1(j,3)*time(i)^3 + 12*k_psi_1(j,4)*time(i)^2 + 6*k_psi_1(j,5)*time(i)^1 + 2*k_psi_1(j,6);
        elseif time(i) <= T_v(j) + t_acc(j,3)+t_no_acc(j,3)
            psi_d(i) =    k_psi_2(j,1)*time(i)^7 +    k_psi_2(j,2)*time(i)^6 +    k_psi_2(j,3)*time(i)^5 +    k_psi_2(j,4)*time(i)^4 +   k_psi_2(j,5)*time(i)^3 +   k_psi_2(j,6)*time(i)^2 + k_psi_2(j,7)*time(i)^1 + k_psi_2(j,8);
            w_psi(i) =  7*k_psi_2(j,1)*time(i)^6 +  6*k_psi_2(j,2)*time(i)^5 +  5*k_psi_2(j,3)*time(i)^4 +  4*k_psi_2(j,4)*time(i)^3 + 3*k_psi_2(j,5)*time(i)^2 + 2*k_psi_2(j,6)*time(i)^1 + k_psi_2(j,7);
            a_psi(i) = 42*k_psi_2(j,1)*time(i)^5 + 30*k_psi_2(j,2)*time(i)^4 + 20*k_psi_2(j,3)*time(i)^3 + 12*k_psi_2(j,4)*time(i)^2 + 6*k_psi_2(j,5)*time(i)^1 + 2*k_psi_2(j,6);
        else
            psi_d(i) =    k_psi_3(j,1)*time(i)^7 +    k_psi_3(j,2)*time(i)^6 +    k_psi_3(j,3)*time(i)^5 +    k_psi_3(j,4)*time(i)^4 +   k_psi_3(j,5)*time(i)^3 +   k_psi_3(j,6)*time(i)^2 + k_psi_3(j,7)*time(i)^1 + k_psi_3(j,8);
            w_psi(i) =  7*k_psi_3(j,1)*time(i)^6 +  6*k_psi_3(j,2)*time(i)^5 +  5*k_psi_3(j,3)*time(i)^4 +  4*k_psi_3(j,4)*time(i)^3 + 3*k_psi_3(j,5)*time(i)^2 + 2*k_psi_3(j,6)*time(i)^1 + k_psi_3(j,7);
            a_psi(i) = 42*k_psi_3(j,1)*time(i)^5 + 30*k_psi_3(j,2)*time(i)^4 + 20*k_psi_3(j,3)*time(i)^3 + 12*k_psi_3(j,4)*time(i)^2 + 6*k_psi_3(j,5)*time(i)^1 + 2*k_psi_3(j,6);
        end
    end
end

figure('Name','psi Euler Angle Equations of Motion');
subplot(3,1,1);
plot(time,psi_d);
title('Angle');
xlabel('t [sec]');
ylabel('Angle [rad]');
subplot(3,1,2);
plot(time,w_psi);
title('Angular Velocity');
xlabel('t [sec]');
ylabel('w [rad/sec]');
subplot(3,1,3);
plot(time,a_psi);
title('Angular Acceleration');
xlabel('t [sec]');
ylabel('a [rad/sec^2]');

% Simulation and Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Gait Selection

T_s = 1; % Desired settling time for error [sec]
z = 0.8; % Desired depreciation for error

w = 4.6/(T_s*z);

p = w^2;
d = 2*z*w;

% Translational Gaits 

Kp_t = [p;p];
Kd_t = [d;d];

% Rotational Gaits 

Kp_r = [p;p;p];
Kd_r = [d;d;d];

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

sim 'Simulink_Model.slx'

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

% Desired vs Actual Position and Rotation Plots

figure('Name','Euler Angles Desired and Calculated');
subplot(3,1,1);
plot(time,phi_d);
hold on;
plot(phi_actual,'--r');
title('Phi Angle');
xlabel('t [sec]');
ylabel('Angle [rad]');
subplot(3,1,2);
plot(time,theta_d);
hold on;
plot(theta_actual,'--r');
title('Theta Angle');
xlabel('t [sec]');
ylabel('Angle [rad]');
subplot(3,1,3);
plot(time,psi_d);
hold on;
plot(psi_actual,'--r');
title('Psi Angle');
xlabel('t [sec]');
ylabel('Angle [rad]');

figure('Name','Position Desired and Calculated');
subplot(2,1,1);
plot(time,x_d,'-b');
hold on;
plot(x_actual,'--r');
title('x Position');
xlabel('t [sec]');
ylabel('Position [m]');
subplot(2,1,2);
plot(time,y_d,'-b');
hold on;
plot(y_actual,'--r');
title('y Position');
xlabel('t [sec]');
ylabel('Position [m]');

% Position and Rotation Plots

figure('Name','Euler Angle Desired - Actual Error');
subplot(3,1,1);
plot(error(:,1));
title('Phi Error - Time Plot');
ylabel('Error [rad]');
xlabel('time [sec]');
subplot(3,1,2);
plot(error(:,2));
title('Theta Error - Time Plot');
ylabel('Error [rad]');
xlabel('time [sec]');
subplot(3,1,3);
plot(error(:,3));
title('Psi Error - Time Plot');
ylabel('Error [rad]');
xlabel('time [sec]');

figure('Name','X and Y Position Desired - Actual Error');
subplot(2,1,1);
plot(error_pos(:,1));
title('x Error - Time Plot');
ylabel('Error [m]');
xlabel('time [sec]');
subplot(2,1,2);
plot(error_pos(:,2));
title('y Error - Time Plot');
ylabel('Error [m]');
xlabel('time [sec]');

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

max(max(J)); % [Watt] Maximum Spontenious RW Power
max(max(abs(w_rw))); % [RPM] Maximum Spontenious RW Speed


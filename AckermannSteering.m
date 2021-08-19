%% Header

% ENAE 788X - Planetary Surface Robotics
% Fall 2020
% Biobot Project

%% Description

% Ackermann Steeering Geometry calculations

%% Init

clear all; close all; clc;

m_t = 175+170+170+80 %[kg]
g_moon = 1.62 % [m/s]
Fweight = m_t*g_moon % newtons
num_wheels = 4;
Ww = Fweight/num_wheels

b = 0.25; % %width of wheels [m]

mu = 0.8;

tao = mu*(Ww*b)/3 %Torque required to steer wheels

tao_two = tao*2;

F_actuator = 220; %N

l_min = tao_two/F_actuator %minimum moment arm required to steer wheel [m]

%% Computer Ackermann Geometries


c = 1.827; %width of wheelbase [m]
b = 1.345; %length of wheelbase [m]


alpha = atan((c/2)/b) %ackermann base angle radians
alpha_deg = rad2deg(alpha)
%set length of l based on necessary wheel toques

l = l_min;
y = l*tan(alpha) %ackermann offset

r = y/sin(alpha) %length of the steering link

%solve for length of total center link

d = c - sin(alpha)*r*r



ratio = c/b

Final = 0;

for theta = 0:.01:2*pi
    theta;
    phi = alpha - asin((2*y)/r - sin(alpha + theta));
    guess = cot(phi) - cot(theta);
    if round(guess, 2) == round(ratio, 2)
        theta_final= theta
        theta_final_deg = rad2deg(theta)
        phi
        Final = guess
    end
    
end



%% Ackermann Geometry Deriv
TW = c;
WB = b;



D = 0.35;
l_arm = .15;

l_rack = 1.1;

adj = D-l_arm;
oppo = (TW -l_rack)/2 
%l_rod = 0.54;
l_rod = sqrt(oppo*oppo + adj*adj)

%% Set deltaP to determine the steering angle beta
deltaP = 0:-.001:-.08; %change in rack location

%solve for the following internal angles
l_1_i(:) = (TW -l_rack)/2 - deltaP;

l_1_i_min = min(l_1_i)

l_2_i(:) = sqrt(l_1_i.^2 + D^2);
%acos((l_arm^2 + l_2_i.^2 - l_rod^2)./(2*l_arm*l_2_i))

Beta_i= rad2deg(pi/2 - atan(D./l_1_i) - acos((l_arm^2 + l_2_i.^2 - l_rod^2)./(2*l_arm*l_2_i)));
Alpha_i_Steer(:) = real(Beta_i(:));

for i = 2:(length(Beta_i) - 1)
    slope_i(i-1) = (Alpha_i_Steer(i) - Alpha_i_Steer(i-1))/(deltaP(i) - deltaP(i-1));
end
slope_i;

%% solve for the following outer angles
l_1_o(:) = (TW -l_rack)/2 + deltaP;

l_2_o(:) = sqrt(l_1_o.^2 + D^2);
%acos((l_arm^2 + l_2_o.^2 - l_rod^2)./(2*l_arm*l_2_o))

Beta_o= rad2deg(-atan(l_1_o/D) + acos((l_arm^2 + l_2_o.^2 - l_rod^2)./(2*l_arm*l_2_o)));
Alpha_o_Steer(:) = real(Beta_o(:));


for i = 2:(length(Beta_o) - 1)
    slope_o(i-1) = (Alpha_o_Steer(i) - Alpha_o_Steer(i-1))/(deltaP(i) - deltaP(i-1));
end
slope_o;



%% Compute Steering Radius

r_turn_i = b*tan(deg2rad(51))

r_turn_o = b*tan(deg2rad(90-27))




%% Ideal Ackerman geometries
r_turn = -1000:.1:500;




alpha_i_idea(:) = rad2deg(atan(b./(r_turn - (c/2))));
alpha_o_ideal(:) = rad2deg(atan(b./(r_turn + (c/2))));
figure 
hold on
plot(0:50,0:50,alpha_i_idea,alpha_o_ideal, Alpha_i_Steer, Alpha_o_Steer)
axis([ 0 50 0 30])
xlabel('Alpha_i{\circ}')
ylabel('Alpha_o{\circ}')
title('Internal Vs. External Wheel Steering Angles')
legend('Simple Steering', 'Ideal Ackermann Steering','Actual Ackermann','Location','Best')
grid on
hold off

figure 
hold on
plot(-deltaP, Alpha_i_Steer, -deltaP, Alpha_o_Steer)
xlabel('{\Delta}P [m]')
ylabel('Steering Angle{\circ}')
legend('Internal Steering Angle', 'External Steering Angle','Location','Best')
grid on
axis([0 0.8 0 50])
hold off


figure 
hold on
plot(-deltaP(2:80), -slope_i, -deltaP(2:80), -slope_o)
xlabel('{\Delta}P [m]')
ylabel('d{\Theta}/{\Delta}P')
legend('Internal d{\Theta}/{\Delta}P', 'External d{\Theta}/{\Delta}P','Location','Best')
grid on
hold off



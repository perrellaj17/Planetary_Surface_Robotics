%% Header

% ENAE 788X - Planetary Surface Robotics
% Fall 2020
% Biobot Project

%% Description

% Calculates rover performance using given parameters
% Terramechanics I calculates general resistances
% Terramechanics II calculates resistances for height of grouser  tandem wheels, wheels with ...
%   grousers, and final drawbar pull
% Output compares MATLAB calculations to Dr. Akin's slides for LRV

%% Init
clc, clear, close all

%% Inputs

%%%%%% Parameters %%%%%%%
% Lunar gravity
g = 1.62; %  < m/s^2 > : Lunar Gravity

% Lunar Rover Parameters
n_crew = 1; % < Number > : Number of Crew on Vehicle
m_v = 250.0; % < kg > : Rover Gross Mass 
m_crew = 170; % < kg > : Individual Crew Gross Mass 
m_payload = 80; % < kg > : Individual Crew Gross Mass

if n_crew > 1
    m_t = m_v + n_crew*m_crew; % < kg > : Total Loaded Vehicle Mass
else
    m_t = m_v + m_crew + m_payload; % < kg > : Total Loaded Vehicle Mass
end

% Lunar Rover Parameters
W_v = m_t*g; % < N > : Vehicle Weight
len = 3; % < m > : Length Between Front and Rear Axel
height = 0.508; % < m > : Vehicles Center of Mass Height Measured from Axel
a_cg = 1.5; % < m > : Vehicles Center of Mass X Position Measured to Front Wheel
y_cg = 0.755; % < m > : Vehicles Center of Mass Y Position Measured from Vehicles CG to Pivot Wheel

num_front_wheels = 2; %  < m > : Number of Front Wheels
num_wheels_4 = 4; %  < m > : Number of Total Wheels
num_wheels_tandem_4 = 2; %  < m > : Number of Total Wheels
num_wheels_6 = 6; %  < m > : Number of Total Wheels
num_wheels_tandem_6 = 4; %  < m > : Number of Total Wheels

% Grousers
h = .03; % < m > : Height of Grouser
N_g_t = 30; % < count > : Number of Grousers

% Climb Angle
theta = deg2rad(20); % < Radians > : Slope Angle 

% Lunar Rover Wheel Parameters
W_w_4 = W_v/num_wheels_4; % < N > : Weight on Each Wheel
W_w_6 = W_v/num_wheels_6; % < N > : Weight on Each Wheel

D_vec = [.1:.01:.9]; % < m >.7 : Wheel Diameter 
r_vec = D_vec./2; % < m > : Wheel Radi
b_vec = [.25]; % < m > : Wheel Width

% Lunar Soil Parameters
c_f = 0.05; % coefficient of friction
gamma = 2470; %  weight density of soil [N/m^3]
n = 1; % exponential heuristic soil parameter
k_c = 1400; % cohesive modulus of soil deformation [N/m^2]
k_phi = 83e4; % frictional modulus of soil deformaiton [N/m^2]
phi = 0.576; % angle of internal resistance of soil [rad]
c = 170; % cohesion [N/m^2]
K = 0.0178; % shear deformation modulus [m]
s = .5; % wheel slip typ. 0.2 - 0.5

% TERZHAGI PAREMETRS for Lunar Soil
N_q = exp((3*pi/2 - phi)*tan(phi))/(2*cos(pi/4 + phi/2)^2);
N_c = (N_q - 1)/tan(phi);
N_gamma = 2*(N_q + 1)*tan(phi)/(1 + 0.4*sin(4*phi));
K_c = (N_c - tan(phi))*cos(phi)^2; % cohesive modulus of soil deformation
K_phi = 72.77; % frictional modulus of soil deformation
K_gamma = (2*N_gamma/tan(phi) + 1)*cos(phi)^2;


%% Calculated parameters [Terramechanics I]

for i = 1:1:length(b_vec)

b = b_vec(i);

for j = 1:1:length(D_vec)

D = D_vec(j); 

% Wheel Soil Interaction Parameters
k = k_c/b + k_phi;

%%%% Flexible Wheel Interaction Parameters %%%%
max_deflection = (4.45)*1e-2;
deflected_length = (D/2 - max_deflection);
deflection_angle = acos(deflected_length/(D/2));
l_0_f = tan(deflection_angle)*deflected_length*2; % length of wheel's circumference in contact with the soil [m]
A_f = b*l_0_f; % area of contact with soil [m^2]

P_s_f_4 = W_w_4/A_f; % safe soil pressure Ridgid [Pa]
P_s_f_6 = W_w_6/A_f; % safe soil pressure Ridgid [Pa]

z_f_4 = ((P_s_f_4)/k)^(1/n); % compression depth [m]
z_f_6 = ((P_s_f_6)/k)^(1/n); % compression depth [m]

alpha_f_4 = acos(1 - 2*z_f_4/D); % angle of approach of wheel to soil [rad]
alpha_f_6 = acos(1 - 2*z_f_6/D); % angle of approach of wheel to soil [rad]

l_f = l_0_f; % length of soil ruptured by compression [m]

N_g_f = l_f/(2*pi*(D/2)/(N_g_t)); % < count > : Number of Grousers in Contact With Ground

%%%% Compression Resistance %%%%
% Flexible
R_c_f_4(i,j) = (1/(n+1))*(k_c+b*k_phi)^(-1/(2*n+1))*(3*W_w_4/((3-n)*sqrt(D)))^(2*(n+1)/(2*n+1)); % compression resistance [N]
R_c_f_6(i,j) = (1/(n+1))*(k_c+b*k_phi)^(-1/(2*n+1))*(3*W_w_6/((3-n)*sqrt(D)))^(2*(n+1)/(2*n+1)); % compression resistance [N]

%%%% Bulldozing  Resistance %%%%
% Flexible
R_b_f_4(i,j) = b*sin(alpha_f_4+phi)/(2*sin(alpha_f_4)*cos(phi))*(2*z_f_4*c*K_c + gamma*z_f_4^2*K_gamma) + l_0_f^3*gamma/3*(pi/2 - phi) + c*l_0_f^2*(1 + tan(pi/4 + phi/2)); % bulldozing resistance [N]
R_b_f_6(i,j) = b*sin(alpha_f_6+phi)/(2*sin(alpha_f_6)*cos(phi))*(2*z_f_6*c*K_c + gamma*z_f_6^2*K_gamma) + l_0_f^3*gamma/3*(pi/2 - phi) + c*l_0_f^2*(1 + tan(pi/4 + phi/2)); % bulldozing resistance [N]

%%%% Gravitational Resistance %%%%
R_g(i,j) = W_v*sin(theta); % gravitation resistance [N]

%%%% Rolling Resistance %%%%
R_r_4(i,j) = (2/3)*(1/sqrt(b*k))* (2*W_w_4/(pi*sqrt(D)) + sqrt(4*W_w_4^2/(pi^2*D) + W_w_4^2/D))^(3/2);
R_r_6(i,j) = (2/3)*(1/sqrt(b*k))* (2*W_w_6/(pi*sqrt(D)) + sqrt(4*W_w_6^2/(pi^2*D) + W_w_6^2/D))^(3/2);

%% Calculated parameters [Terramechanics 2]

%%%% Tractive Force with Grousers %%%%
% Flexible 4 Wheel
H_grousers_f_4(i,j) = (b*l_f*c*(l_f + 2*h/b)*N_g_f + W_w_4*tan(phi)*(1+ 0.64*(h/b)*atan(b/h)))*(1-(K/(s*l_f))*(1-exp(-s*l_f/K))); % tractive force per wheel with grousers [N]

% Flexible 6 Wheel
H_grousers_f_6(i,j) = (b*l_f*c*(l_f + 2*h/b)*N_g_f + W_w_6*tan(phi)*(1+ 0.64*(h/b)*atan(b/h)))*(1-(K/(s*l_f))*(1-exp(-s*l_f/K))); % tractive force per wheel with grousers [N]

%%%% Drawbar Pull %%%%
% Drawbar pull must be positive to move rover

% Flexible 4 Wheel
DP_flat_grousers_f_4(i,j) = num_wheels_4*H_grousers_f_4(i,j) - (num_wheels_4*R_c_f_4(i,j) + num_front_wheels*R_b_f_4(i,j) + num_wheels_tandem_4*R_r_4(i,j)); % drawbar pull on flat ground [N]
DP_slope_grousers_f_4(i,j) = num_wheels_4*H_grousers_f_4(i,j) - (num_wheels_4*R_c_f_4(i,j) + num_front_wheels*R_b_f_4(i,j) + num_wheels_tandem_4*R_r_4(i,j) + R_g(i,j)); % drawbar pull on 15 deg slope [N]

% Flexible 6 Wheel
DP_flat_grousers_f_6(i,j) = num_wheels_6*H_grousers_f_6(i,j) - (num_wheels_6*R_c_f_6(i,j) + num_front_wheels*R_b_f_6(i,j) + num_wheels_tandem_6*R_r_6(i,j)); % drawbar pull on flat ground [N]
DP_slope_grousers_f_6(i,j) = num_wheels_6*H_grousers_f_6(i,j) - (num_wheels_6*R_c_f_6(i,j) + num_front_wheels*R_b_f_6(i,j) + num_wheels_tandem_6*R_r_6(i,j) + R_g(i,j)); % drawbar pull on 15 deg slope [N]
end

figure(1)
hold on
plot(D_vec,DP_flat_grousers_f_4(i,:))
plot(D_vec,DP_flat_grousers_f_6(i,:))
hold off

figure(2)
hold on
plot(D_vec,DP_slope_grousers_f_4(i,:))
plot(D_vec,DP_slope_grousers_f_6(i,:))
hold off

end

figure(1)
hold on
title('Drawbar Pull for Flexible Wheels on a Flat Surface')
xlabel('Wheel Diameter (m)')
ylabel('Drawbar Pull (N)')
plot([.1 .9],[100 100],'--k')
plot([.1 .9],[0 0],'--r')
plot([.6 .6],[-400 1000],'-r')
legend('Wheel Width (4 Wheel) = .2 m', 'Wheel Width (6 Wheel) = .2 m', '100 N of Drawbar Pull','0 N of Drawbar Pull','Min Wheel Diameter')
hold off

figure(2)
hold on
title('Drawbar Pull for Flexible Wheels at a 20^o Slope')
xlabel('Wheel Diameter (m)')
ylabel('Drawbar Pull (N)')
plot([.1 .9],[100 100],'--k')
plot([.1 .9],[0 0],'--r')
plot([.6 .6],[-400 1000],'-r')
legend('Wheel Width (4 Wheel) = .2 m', 'Wheel Width (6 Wheel) = .2 m', '100 N of Drawbar Pull','0 N of Drawbar Pull','Min Wheel Diameter')
hold off


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
clc, clear, clear all

%% Inputs

% Lunar Soil
gamma = 2472; %  weight density of soil [N/m^3]
n = 1; % exponential heuristic soil parameter
k_c = 1400; % cohesive modulus of soil deformation [N/m^2]
k_phi = 82e4; % frictional modulus of soil deformaiton [N/m^2]
phi = 0.576; % angle of internal resistance of soil [rad]
c = 170; % cohesion [N/m^2]
K_shear = 0.0178; % shear deformation modulus [m]

% TERZHAGI PAREMETRS for lunar soil
N_q = 32.23;
N_c = 48.09;
N_gamma = 33.27;
K_c = 33.37; % cohesive modulus of soil deformation
K_phi = 72.77; % frictional modulus of soil deformation
K_gamma = (2*N_gamma/tan(phi) + 1)*cos(phi)^2;

% Lunar gravity
g = 1.62; % [m/s^2]

% Lunar Rover
b = 0.229; % wheel width [m]
D = 0.813; % wheel diameter [m]
W_w = 259; % weight on each wheel [N]
W_v = 1004; % vehicle weight [N]
h = 0; % height of grouser [m]
N_g = 0; % number of grousers in contact with ground
theta_slope = deg2rad(15); % slope angle [rad]
c_f = 0.05; % coefficient of friction
num_front_wheels = 2;
num_wheels = 4;

% 2 parallel 36Vdc batteries (Ag-Zn non rechargable)
% Independent motors on each wheel
% front and rear independent steering
% W_w = W_v/num_wheels;

% Motion
% omega = 0; % wheel angular acceleration [rad/s^2]
% V = 0; % vehicle speed [m/s]

% Grousers
h = 0; % height of grouser [m]
N_g = 0; % number of grousers in contact with ground

%% Calculated parameters [Terramechanics I]

% PARAMETERS
k = k_c/b + k_phi;
z = (3*W_w/((3-n)*(k_c+b*k_phi)*sqrt(D)))^(2/(2*n+1)); % compression depth [m]
% z_LRV = 0.0151 [m] from slides, but i believe 0.0172 is actually correct
alpha = acos(1 - 2*z/D); % angle of approach of wheel to soil [rad]
% alpha_LRV = 0.2999 [rad] from slides
l_o = z*tan(pi/4 - phi/2)^2; % length of soil ruptured by compression [m]
% l_o_LRV = 0.005341 [m] from slides
l = (D/2)*acos(1 - 2*z/D); % length of wheel's circumference in contact with the soil [m]
% l_LRV = 0.1218 [m] from slides
l_min = l/2; % minimum contact length [m]
% l_min_LRV = 0.058 [m]
A = b*l; % area of contact with soil [m^2]
W_s = A*(c*N_c + gamma*z*N_q + 0.5*gamma*b*N_gamma); % safe weight on soil [N]
P_s = W_s/A; % safe soil pressure [Pa]
% P_s_LRV = 18,790 [Pa] from slides

% RESISTANCES PER WHEEL
R_c = (1/(n+1))*(k_c+b*k_phi)^(-1/(2*n+1))*(3*W_w/((3-n)*sqrt(D)))^(2*(n+1)/(2*n+1)); % compression resistance [N]
% R_c_LRV = 28.3 [N] from slides
R_b = b*sin(alpha+phi)/(2*sin(alpha)*cos(phi))*(2*z*c*K_c + gamma*z^2*K_gamma) + l_o^3*gamma/3*(pi/2 - phi) + c*l_o^2*(1 + tan(pi/4 + phi/2)); % bulldozing resistance [N]
% R_b_LRV_per_leading_wheel = 95 [N] from slides

% TOTAL RESISTANCES
R_c_total = R_c*num_wheels; % total compression resistance from all wheels [N]
R_r = W_v*c_f; % rolling resistance [N]
% R_r_LRV = 50 [N] from slides
R_b_total = R_b*num_front_wheels; % total bulldozing resistance from all front wheels [N]
% R_b_LRV_total = 190 [N] from slides
R_g = W_v*sin(theta_slope); % gravitation resistance [N]
% R_g_LRV_15deg_slope = 260 [N] from slides
R_t = R_c_total + R_r + R_b_total; % total resistance for flat [N]
R_t_slope = R_c_total + R_r + R_b_total + R_g; % total resistance for 15 deg slope [N]


%% Calculated parameters [Terramechanics 2]

% TANDEM WHEELS
% assume n = 1/2
% assume small sinkage, small angle assumption cos(theta) = 1
r_1 = D/2; % radius of front wheel [m]
r_2 = D/2; % radius of back wheel [m]
D_1 = D; % diameter of front wheel [m]
D_2 = D; % diameter of back wheel [m]
% z_1 = ; % sinkage from front wheel [m]
% z_2 = ; % sinkage from back wheel [m]
% z_0 = z_1 + z_2; % total sinkage [m]

% (unsure how to calculate sinkage and weights. Calculating one ...
%   requires the other)

% W_1 = pi*b*k*z_1*sqrt(r_1)/(2*sqrt(2)); % weight on front wheel [N]
% W_2 = b*k*sqrt(2*r_2)*sqrt(z_o*z_2); % weight on back wheel [N]
% z_o = 1/(b*k)* (sqrt(2)*W_1/(pi*sqrt(r_1)) + sqrt(2*W_1*W_1/(pi*pi*r_1) + W_2*W_2/(2*r_2)));
% for n = 0.5 and R = 2/3 * b*k* (z_o^3/2)
% Rolling resistance tandem wheels
% R_roll_t = (2/3)*(1/sqrt(b*k))* (2*W_1/(pi*sqrt(D_1)) + sqrt(4*W_1*W_1/(pi*pi*D_1) + W_2*W_2/(D_2)))^3/2

% WHEEL SLIP
% s = 1 - V/(omega*(D/2)); % wheel slip
s = 1; % wheel slip typ. 0.02 - 0.05

% TRACTIVE FORCE PER SMOOTH WHEEL
W = W_w; % weight on each wheel [N]
H_smooth = (A*c + W*tan(phi))* (1 - (K_shear/(s*l)) * (1 - exp(-s*l/K_shear))); % tractive force per smooth wheel [N]

% TRACTIVE FORCE PER WHEEL WITH GROUSERS
H_grousers = (b*l*c*(1 + 2*h/b)*N_g + W*tan(phi)*(1+ 0.64*(h/b)*atan(b/h)))*(1-(K_shear/(s*l))*(1-exp(-s*l/K_shear))); % tractive force per wheel with grousers [N]

% Draw Bar Pull
H = H_smooth*4; % choose either H_smooth or H_grousers
DP_flat = H - (R_c + R_b + R_r); % drawbar pull on flat ground [N]
DP_slope = H - (R_c + R_b + R_r + R_g); % drawbar pull on 15 deg slope [N]
% drawbar pull must be positive to move rover

%% LRV Slide Calculations

% Final calculations from Dr. Akin slides for LRV

R_c_LRV = 113; % N; 4 wheels
R_b_LRV = 190; % N; 2 front wheels
R_g_LRV = 260; % N; gravitational resistance on 15 deg slope
R_r_LRV = 50; % N
R_t_LRV = 352; % N; flat ground
R_t_LRV_slope = 613; % N; 15deg slope

%% Output

fprintf("\nBIOBOT CALCULATOR\n\n")
fprintf("Compare MATLAB calculations to Dr. Akin slides for LRV\n\n")
fprintf("============================================================\n")
fprintf("Parameter Description                  MATLAB       Slides\n")
fprintf("------------------------------------------------------------\n")
fprintf("R_c       Compression Resistance       %.2f         %.2f\n\n", R_c, R_c_LRV)
fprintf("R_b       Total Bulldozing Resistance  %.2f         %.2f\n\n", R_b_total, R_b_LRV)
fprintf("R_g       Gravitation Resistance       %.2f         %.2f\n\n", R_g, R_g_LRV)
fprintf("R_r       Rolling Resistance           %.2f         %.2f\n\n", R_r, R_r_LRV)
fprintf("R_t       Total Resistance [flat]      %.2f         %.2f\n\n", R_t, R_t_LRV)
fprintf("R_t_slope Total Resistance [15 deg]    %.2f         %.2f\n\n", R_t_slope, R_t_LRV_slope)
fprintf("============================================================\n")
fprintf("\n")
fprintf("Final Drawbar Pull (must be positive for rover to move forward)\n\n")
fprintf("============================================================\n")
fprintf("Parameter Description                  MATLAB\n")
fprintf("------------------------------------------------------------\n")
fprintf("DP_flat   Drawbar pull on flat ground  %.2f\n", DP_flat)
fprintf("DP_slope  Drawbar pull on 15deg slope  %.2f\n", DP_slope)
fprintf("============================================================\n")
fprintf("\n")





%% Mass breakdown

% Suspension & Surface contact interface - 
%     Rims
%     Arms
%     Tires
%     Coils
%     Shafts
% Drivetrain - 
%     Motors
%     Gears
%     Breaks and Steering  
% Chassis -
%     Chassis structure
%     Driver Control Interface (pedals and steering wheel)
%     Fenders
% Sensors -
%     LIDAR
%     Optical cameras
%     potentiometers
%     synchro-resolver
% Electronics System-
%     Energy Storage
%     power supply and regulation
%     Cables & protection systems
%     Lights
% Life support HArdware-
%     LS monitoring
%     water
%     oxygen
% Communications-
%     Do we want diret to earth and direct from earth? Is there an uplink/
%     download speed we need? How about an operational RF bandwidth?
%     Comms processing CCAs
%     cables
%     transcievers
% Thermal Control - 
%    MLI
%    Radiation sheilding
%    Thermal Straps
%    Radiators
%    absorptive/reflective coatings
















% % REQUIREMENTS 
% 
% 1.	Performance
% i.	Rover shall have a maximum operating speed of at least 4 m/sec on level, flat terrain.
% ii.	Rover shall be designed to accommodate a 0.3 meter obstacle at minimal velocity.
% iii.	Rover shall be designed to accommodate a 0.1 m obstacle at a velocity of 2.5 m/sec.
% iv.	Rover shall be designed to safely accommodate a 20? slope in any direction at a speed of at least 1 m/sec and including the ability to start and stop.
% v.	The rover shall have a nominal sortie range of 54 km at an average speed of 2.5 m/sec.
% 
% 2.	Payload
% i.	Rover shall be capable of carrying one 170 kg EVA crew and 80 kg of assorted payload in nominal conditions.
% ii.	Payload may be modeled as a 0.25 m box
% iii.	Rover shall be capable of also carrying a second 170 kg EVA crew in a contingency situation. Payload may be jettisoned if design permits.
% iv.	Rover design shall incorporate roll-over protection for the crew and all required ingress/egress aids and crew restraints.
% 
% 3.	Operations
% i.	A nominal sortie shall be at least eight hours long.
% ii.	Two rovers must be launched on a single CLPS lander.
% iii.	A single rover shall mass ?250 kg.
% iv.	Rovers shall be developed in time to be used on the first Artemis landing mission.
% v.	Rover shall be capable of operating indefinitely without crew present.
% 
% 4.	Guidance Navigation & Control
% i.	Rover shall be capable of being controlled directly, remotely, or automated.
% ii.	Rover shall be capable of following an astronaut, following an astronaut?s path, or autonomous path planning between waypoints.
% iii.	Rover shall be capable of operating during any portion of the lunar day/night cycle and at any latitude.
% 




%%% COME UP WITH A 6 WHEEL DESIGN
% SKETCH
% ASTRONAUT ORIENTATION
% 

clear all
%%%%%%% Parameters %%%%%%%%

mu_s = 1; % Sideways Friction Coefficient
mu_r = 0.2; % Rolling Friction Coefficient
c = 1.5; % Distance Between Left and Right Tires [m]
l = 3; % Distance Between Front and Back Tires [m]
%r = linspace(1,10); % Range of Turn Radii
r_min = 5;
r_max = 50;
alpha = asin(mu_r); % Turn Angle of Wheel From Forward
r = linspace(r_min,r_max); % Range of Turn Radii

%% In-place Rotation Analysis
% Compares skid-steer and 4 wheel steer in-place
% turns (ackerman doesn't have this capability)

r_turn = sqrt((c/2)^2 + (l/2)^2); % Distance From CG to Wheel

% Turn In-Place Power Requirements
P_ip_skid = 4*(1/2*(c*mu_r + l*mu_s));
P_ip_steer = 4*(r_turn*mu_r);

% Ratio of power turning in place skid/steer
P_ip_ratio = P_ip_skid/P_ip_steer;

fprintf("\nBiobot Steering Analysis\n\n")
fprintf("============================================================\n")

fprintf("Turning In-Place Power Ratio              \n")
fprintf("Ratio Between Skid Steer and 4 Wheel Steer: %.2f \n", P_ip_ratio)
%fprintf("Steering Angle: %.2f \n", rad2deg(alpha))
fprintf("============================================================\n")

%% Turning Along a Path
% Compares the relative total power required
% for skid-steer, 4 wheel steer, and ackerman
% steering along curves of specified radii

% Turn along curve relative total power

%4 wheel skid steering
P_skid_tot = 2*(l*mu_r + 2*r*mu_s);
% P_inner_tot = (V/r)*N*(mu_r*(l/2) + mu_s*(r - c/2)); %page 8
% P_outer_tot = (V/r)*N*(mu_r*(l/2) + mu_s*(r + c/2)); %page 8
%P_skid_tot = P_innter_tot + P_outer_tot;


%Front axis ackerman steering (ri and ro are create different steer angles
%on the front inner and outer wheels. 
P_ackerman_tot = mu_r*(2*r + sqrt((r - c/2).^2 + l^2) + sqrt((r + c/2).^2 + l^2)); %single front axis steering, total power
%P_outer_front = V/r * (sqrt((r + c/2).^2 + l^2); %page 9
%P_inner_front = V/r * (sqrt((r - c/2).^2 + l^2); %page 9
%P_outer_rear = V/r * (r + c/2); %page 9
%P_inner_rear = V/r * (r - c/2); %page 9


%"four wheel steer" page 6
P_steer_tot    = 2*mu_r*(sqrt((r + c/2).^2 + (l/2)^2) + sqrt((r - c/2).^2 + (l/2)^2));
% P_w_inner = V/r * (sqrt((r - c/2).^2 + (l/2)^2) * mu_r*N; %page 6
% P_w_outer = V/r * (sqrt((r + c/2).^2 + (l/2)^2)* mu_r*N; %page 6
% P_skid_tot = 2*P_w_inner + 2*P_w_outer; %made up?


%%combonation steering? front with ackerman and back with skid??

figure(1)
hold on
plot(r,P_steer_tot, '--',r,P_ackerman_tot)
title('Relative Power vs. Turn Radius')
xlabel('Turning Radius (m)')
ylabel('Relative Power')
legend('Four Wheel Steer','Front Axis Ackermann')
grid on
hold off


figure(2)
hold on
plot(r,P_skid_tot, '-o',r,P_steer_tot, '--',r,P_ackerman_tot)
title('Relative Power vs. Turn Radius')
xlabel('Turning Radius (m)')
ylabel('Relative Power')
legend('Skid Steering','Four Wheel Steer','Front Axis Ackermann')
grid on
hold off


% r = 10
% r_i = sqrt((r - c/2)^2 + (l/2)^2)
% 
% r_o = sqrt((r + c/2)^2 + (l/2)^2)
% 
% alpha_i = rad2deg(acos((l/2)/r_i))
% alpha_o = rad2deg(acos((l/2)/r_o))

% alpha_i = asin((r-(c/2))/r_i)
% alpha_o = asin((r+(c/2))/r_o)

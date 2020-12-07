function [costVector]=parallelHybrid(t_vec,SOC_start,SOC_final);
% Function for calculating the all arc-costs from one node to all other possible nodes.
%
% This function is called from dynProg1D.m
%
% Inputs:
% t_vec - 1x2 matrix, with the start and stop time for the interval.
% SOC_start - A single start value for SO during the interval.
% SOC_final - Vector (from the dicretization) with all possible final values for the
% interval.
%
% Output:
% costVector - Vector with the costs for all arcs from SOC_start.
% Version 1.0, 2008-06-30 Lars Eriksson
% Implement your parallel hybrid model and calculate the arc costs below.

% Global Variable
 global V_z;
 global G_z;

% Vehicle Data 
H_l= 44.6e6; % Lower Heating Value [J/kg] 

roh_l=732.2; % Fuel Density [Kg/m3]

roh_a=1.18; % Air Density [kg/m3]

Je=0.2; % Engine Inertia [kgm2]

T_engine_max= 115; % Engine Maximum Torque [Nm]

V_disp=1.497e-3; % Engine Displacment [m3]

e=0.4; % Willans approximation of engine efficiency

p_me0=0.1e6; % Willans approximation of engine pressure [MPa]

Q_o=6.5; % Battery charging capacity [Ah]

Uoc=300; % Open circuit voltage [V]

Imax= 200; % Maximum dis-/charging current [A]

Imin=-200; % Maximum dis-/charging current [A]

Ri= 0.65; % Inner resistance [ohm]

n_electricmachine=0.9; % Efficiency of electrical machine

g=9.81; % Gravity

cD=0.32; % Drag coefficient

cR=0.015; % Rolling resistance coefficient

Af=2.31; % Frontal area [m2]

mv= 1500; % Vehicle mass [kg]

rw=0.3; % Wheel radius [m]

Jw=0.6; % Inertia of the wheels [kgm2]

mwheel=Jw/(rw^2); % Rotating Mass [kg]

eta_gearbox=0.98; % Efficiency of Transmission

T_em_max=400; % Maximum Electric machine Torque [Nm]

P_em_max=50; % Power of Electric Machine [kW]

m_em=1.5; % Electric motor weight [kg/Kw]

P_pt_max=90.8; % Maximum powertrain power [kW]

%-----------------------Cost Vector Calculations--------------------------%

% Average Velocity at Wheel
Average_speed_wheel= mean(V_z(t_vec)); 

% Average Acceleration at Wheel
Average_accl_wheel = V_z(t_vec(2))-V_z(t_vec(1)); 

% Angular Velocity at Wheel
Angular_vspeed_wheel=Average_speed_wheel/rw; 

% Angular Acceleration at Wheel
Angular_acc_wheel=Average_accl_wheel/rw; 

% Gear Ratio according to gear number from drive cycle
Gearratio(G_z(t_vec(1)) == 0) =0; 
Gearratio(G_z(t_vec(1)) == 1) =13.0529; % 1st Gear ratio
Gearratio(G_z(t_vec(1)) == 2) =8.1595; % 2nd Gear ratio
Gearratio(G_z(t_vec(1)) == 3) =5.6651; % 3rd Gear ratio
Gearratio(G_z(t_vec(1)) == 4) =4.2555; % 4th Gear ratio
Gearratio(G_z(t_vec(1)) == 5) =3.2623; % 5th Gear ratio

% Total Required or traction force at wheel
Forcetraction = (0.5*roh_a*Af*cD.*(Average_speed_wheel.^2)) + (mv*g*cR) + ((mv+mwheel).*Average_accl_wheel); 

% Torque of Wheel
Torquewheel=Forcetraction*rw; 

% Torque of Gearbox
Torquegearbox=Torquewheel./(Gearratio*eta_gearbox.^sign(Torquewheel)); 

% Angular Valocity of Engine
Angular_speed_engine=Angular_vspeed_wheel.*Gearratio; 

% Angular accleration of engine
Angular_accl_engine=Angular_acc_wheel.*Gearratio; 

% Charging/ dischargning Current
I_batt=-((SOC_final-SOC_start)*(Q_o*3600)); 

% Power of Battery
Powerbattery=(Uoc*I_batt)-(Ri*I_batt.^2); 

% Torque of Electric Machine
Torqueelectric_machine=(Powerbattery.*n_electricmachine.^sign(I_batt))./Angular_speed_engine;

% Torque of Engine
Torqueengine=Torquegearbox-Torqueelectric_machine; 

% Fuel Consumption Cost Vector for given t_vec
x=((p_me0*V_disp)/(4*pi)); 
costVector=(Angular_speed_engine/(e*H_l))*(Torqueengine+x+Je*(Angular_accl_engine)); 


% Additional Constraints
costVector(Torqueengine>T_engine_max)=inf;
costVector((Gearratio==0) & ((I_batt)==0))=0;
costVector(Torqueelectric_machine<-T_em_max | Torqueelectric_machine>T_em_max)=Inf;
costVector((abs(I_batt)>0) & (Gearratio==0))=inf;
costVector(I_batt<Imin | I_batt>Imax)=Inf;
costVector(costVector<0)=0;

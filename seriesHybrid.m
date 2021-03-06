function costVector=seriesHybrid(t_vec,SOC_start,SOC_final,Ne_start,Ne_final);

% costVector=seriesHybrid(t_vec,SOC_start,SOC_final,Ne_start,Ne_final);
% 
% Function for calculating all the arc-costs during one time interval, from one node to
% all other possible nodes.
% 
% This function is called from dynProg2D.m with the following argument list
%    funName([tVec(t) tVec(t+1)],discX(ii),discX,discY(jj),discY)
%  
% Inputs:
%  t_vec      - 1x2 matrix, with the start and stop time for the interval.
%  SOC_start  - A single start value for SOC during the interval.
%  SOC_final  - Vector (from the dicretization) with all possible final values for the
%               interval.
%  Ne_start   - A single start value for engine speed during the interval.
%  Ne_final   - Vector (from the dicretization) with all possible final values for the
%               interval.
%
% Output:
%  costMatrix - Matrix with the costs for all arcs from SOC_start. 
%               The size of the returned cost-matrix should have the following
%               size:  length(SOC_final) x length(Ne_final)

%% Matrix Creation 3D
SOC_final=SOC_final';
SOC_final=repmat(SOC_final,1,length(Ne_final));
[b,demo]=size(SOC_final);
Ne_final=repmat(Ne_final,b,1);

%% Global Variables
global V_z; % Velocity Gird
global T_z; % Time Grid


%----------------------- Vehicle Data --------------------------%

%% Fuel and Air Parameters
H_l=44.6*1000000; % Lower Heating Value [J/kg] 
roha_a=1.18; % Fuel Density [Kg/m3]
J_e=0.2; % Air Density [kg/m3]

%% Engine Configuration
T_Engine_max= 115; % Engine Maximum Torque [Nm]
V_d=1.497/1000; % Engine Displacment [m3]
e=0.4; % Willans approximation of engine efficiency
p_me0=0.1; % Willans approximation of engine pressure [MPa]

%% Battery Configuration
Q_o=6.5; % Battery charging capacity [Ah]
U_oc=300; % Open circuit voltage [V]
Imax= 200; % Maximum dis-/charging current [A]
Imin=-200; % Maximum dis-/charging current [A]
Ri= 0.65; % Inner resistance [ohm]

%% Motor and Generator Configuration
eta_motor=0.9; % Efficiency of Electrical Machine
T_mech_max=400; % Maximum Torque of Electric Machine [Nm]
Pmech_max=50000; % Maximum Power of Electric Machine [W]
Pmech_min=-50000; % Minimum Power of Electric Machine [W]
  
%% Logitudinal Vehicle Model Parameters
cD=0.32; % Drag coefficient
cR=0.015; % Rolling resistance coefficient
Af=2.31; % Frontal area [m2]
m= 1500; % Vehicle mass [kg]
rw=0.3; % Wheel radius [m]
Jw=0.6; % Inertia of the wheels [kgm2]
mv_wheel=Jw/(rw)^2; % Rotating Mass [kg]
g=9.81; % Gravity

%% Velocity and Aceelration Engien Range
N_e= 0:800:5000; % Engine RPM range
w_dot_e= 300; % [rad/s2] % Maximum accelration of the engine


%-----------------------Cost Vector Calculations--------------------------%

%% Longitudinal Vehicle Dynamics

% Angular Velocity of Engine
Average_angular_velocity_engine=((Ne_start+Ne_final)/2)*((2*pi)/60);

% Angular Acceleration of Engine
Average_angular_acceleration_engine=((Ne_final-Ne_start)*((2*pi)/60));

% Average Velocity at Wheel
Average_speed_wheel= (V_z(t_vec(2))+V_z(t_vec(1)))/2; 

% Average Acceleration at Wheel
Average_acce_wheel = (V_z(t_vec(2))-V_z((t_vec(1))));    

% Angular Velocity at Wheel
Angular_speed_wheel=Average_speed_wheel/rw;  

% Total Required or traction force at wheel
Forcetractive = (0.5*roha_a*Af*cD.*(Average_speed_wheel.^2)) + (m*g*cR) + ((m+mv_wheel).*Average_acce_wheel); 

% Torque of Wheel
Torquewheel=Forcetractive*rw; 

%% Electric Machine Model

% Torque of Electric Motor through motor efficiecny
Torque_electricmotor=Torquewheel/eta_motor^sign(Torquewheel); 

% Power of Electric Motor
Power_electricmotor=Torque_electricmotor*Angular_speed_wheel;  

%% Battery Model

% Charging/ dischargning Current
I_batt=-((SOC_final-SOC_start)*(Q_o*3600));  

% Power of Battery
Pbattery=(U_oc*I_batt)-(Ri*I_batt.^2); 

% Power of Generator
Power_gen=Power_electricmotor-Pbattery;  

% Torque of Generator
Torque_gen=Power_gen./Average_angular_velocity_engine;

% Additional Constraints
Power_gen(Power_gen<0)=0;

%% Engine Model

% Torque of Engine
Torque_engine=Torque_gen/eta_motor;   

% Additional Constraints
Torque_engine(Power_gen==0)=0;

% Fuel Consumption Cost Vector for given t_vec
x=((p_me0*1000000*V_d)/(4*pi));  
costVector=(Average_angular_velocity_engine./(e*H_l)).*(Torque_engine+x+(J_e*Average_angular_acceleration_engine));        % Fuel Consumption for given t_vec
 
%% Additional Constraints
costVector((Ne_start==0 & Ne_final>800))=inf;
costVector(Power_gen>Pmech_max)=inf;
costVector(Ne_start==0 & Torque_engine>0)=inf;
costVector(Average_angular_acceleration_engine>w_dot_e)=inf;
costVector(costVector<0)=0;
costVector(I_batt>Imax | I_batt<Imin)=inf;
costVector(Torque_engine>T_Engine_max)=inf;%
costVector(Torque_gen>T_mech_max)=inf;
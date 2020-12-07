% Uncomment the lines as applicable
%CaseToRun='parallel';
%CaseToRun='series';
CaseToRun='testParallel';
%CaseToRun='testSeries';

% Define Global Variable
global V_z;
global G_z;
% global T_z;
% global D_z;

% Load Drive Cycle
load('City_MAN_DDP.mat');
%load('EUDC_MAN_DDP');
%load('FTP_75.mat');


switch CaseToRun,
 case 'parallel',
     
  % Set up the problem:
  
  % Time Grid
  T_grid    = T_z;
  
  % Select% discretization for the SOC.
   SOC_grid  = 0.4:0.001:0.6;
 
   % Assign cost for the final state.
  finalCost=zeros(1,length(SOC_grid));
  finalCost(SOC_grid<0.5)=Inf;

  % Solve the problem
  [value,SOC_path]=dynProg1D(@parallelHybrid,T_grid,SOC_grid,finalCost);

  % Postprocess and analyze the result
  
 case 'series',
  % Set up the problem:
  
  % Time Grid
    T_grid    = T_z;
    
  % Select discretization for the SOC and engine speed
    SOC_grid  = 0.4:0.05:0.6;
   
  % Engine Speed Grid
    Ne_grid=20:0.02:25;  
    
  % Assign cost for the final states.1
    finalCost=zeros(length(SOC_grid),length(Ne_grid));
    finalCost(SOC_grid<0.5,Ne_grid<22)=Inf;
    
  % Solve the problem
  [value,SOC_path,Ne_path]=dynProg2D(@seriesHybrid,T_grid,SOC_grid,Ne_grid,finalCost);
  
  
  
  
  
  % Postprocess and analyze the result
  
 case 'testParallel',
   % Test the arc costs of the parallel hybrid to see that they are reasonable
   % Add the code for the tests that you want to make
   o=find(SOC_grid == 0.5);
   a=zeros();
   x=zeros();
%   SOC_path=zeros();
   Fuelcon_path=zeros();
   for i=1:length(T_z)-1
      if(SOC_path(i,o)==o)
      a(i)=SOC_path(i,o);
      x(i)=value(i,o);
      else
      o=SOC_path(i,o);
      x(i)=value(i,o);
      a(i)=o;
      end
       SOC(i)=SOC_grid(a(i));
       Fuelcon_path(i)=x(i);
   end
   totallength=trapz(T_z,V_z);
   Fuel_consumption=((value(1,(length(SOC_grid)/2)+0.5)/0.7372)*100)/(totallength/1000);  
   SOC((T_z(end,1)))=0.5;
   Fuelcon_path((T_z(end,1)))=0;
%    subplot(2,2,1);
%    plot(T_z,V_z);
%    xlabel('Time(s)');
%    ylabel('Velocity(m/s)');
%     subplot(2,2,2);
%    plot(T_z,S_path);
%    xlabel('Time(s)');
%    ylabel('State of Charge');
%    subplot(2,2,[3 4]);
%    %subplot(2,1,2);
%    plot(T_z,Fuel_path);
%    xlabel('Time(s)');
%    ylabel('Fuel Consumption(litre)');
       
   subplot(2,2,1);
       plot(T_z,V_z);
       xlabel('Time in seconds');
       ylabel('Velocity in m/s');
       
   subplot(2,2,2);
       plot(T_z,SOC*100);
       xlabel('Time in seconds');
       ylabel('SOC');
   
   subplot(2,1,2);
       plot(T_z,Fuelcon_path);
       xlabel('Time in seconds');
       ylabel('Fuel Consumption in KG');
  
   
 
   
 case 'testSeries',
   % Test the arc costs of the series hybrid and
   % Add the code for the tests that you want to make
   o=3;
   z=101;
   a=zeros();
   c=zeros();
   x=zeros();
   SOC_new=zeros();
   Ne_new=zeros();
   fuelcon=zeros();
   for i=1:length(time)-1
      if(SOC_path(i,o,z)==o && Ne_path(i,o,z)==z)
      a(i)=SOC_path(i,o,z);
      c(i)=Ne_path(i,o,z);
      x(i)=value(i,o,z);
      else
      a(i)=SOC_path(i,o,z);
      c(i)=Ne_path(i,o,z);
      x(i)=value(i,o,z);
      z=c(i);
      o=a(i);
      end
       SOC_new(i)=SOC_grid(a(i));
       Ne_new(i)=Ne_grid(c(i));
       fuelcon(i)=x(i);
   end
      
   total_length=trapz(time,V_z);
   Fuel_consumption=((value(1,o,1)/0.7372)*100)/(total_length/1000); 
   SOC_new((time(end,1)))=0.5;
   Ne_new((T_z(end,1)))=0;
   fuelcon((T_z(end,1)))=0;
   %subplot(2,2,1);
   figure(1)
   plot(T_z,V_z);
   xlabel('Time in second');
   ylabel('Velocity in m/s');
   %subplot(2,2,2);
   figure(2)
   plot(T_z,SOC_new);
   xlabel('Time in second');
   ylabel('SOC');
   %subplot(2,2,3);
   figure(3)
   plot(T_z,Ne_new);
   xlabel('Time in seconds');
   ylabel('Engine RPM');
   %subplot(2,2,4);
   figure(4)
   plot(T_z,fuelcon);
   xlabel('Time(s)');
   ylabel('Fuel Consumption in Kg');      
   
 otherwise,
  error('Unknown case')
end
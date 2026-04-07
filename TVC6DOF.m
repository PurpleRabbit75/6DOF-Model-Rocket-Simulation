%% Nominal Vehicle Parameters
%Moments of Inertia (in kg m^2)
Ix_n=0.1;
Iy_n=0.1;
Iz_n=0.1;

%Mass (in kg)
m_dry_n=0.94;  %Mass of everything but fuel
g=9.81;

%Distance between CG and TVC Mount (in m)
l_n=0.3;

%% Control Parameters
%Control Cycle Time (in s)
dt=0.01;

%Servo Actuator Delay (in s)
servo_delay_n=0.08;

%Control Gains
kp=1; % Proportional
kd=0.1; % Integral
ki=1; % Derivative

%% Vehicle Parameter Error (variance from nominal values)
%Misalignment of TVC Mount on Y and Z axes (in m)
Ix_v=0.0001;
Iy_v=0.0001;
Iz_v=0.0001;
m_dry_v=0.0001;
servo_delay_v=0.01;
mis_y_v=0.001;
mis_z_v=0.001;
static_error_y_v=1;
static_error_z_v=1;
l_v=0.01;

%% Simulation Parameters
sim_time=16; % Simulation time in seconds
cpu_cores=4; % Number of CPU cores to use for parallel processing (if applicable)
runs=50; % Number of runs for Monte-Carlo simulation
filename = 'TVC6DOF_Results.xlsx';

%% Monte-Carlo


% 1. Open parallel pool if not already open
if isempty(gcp('nocreate'))
    parpool(cpu_cores);
end


% 2. Pre-allocate an array of SimulationInput objects
in(runs) = Simulink.SimulationInput('TVC6DOF_sim');
tic % Start timer
for k = 1:runs
    % Create a SimulationInput object for this run
    in(k) = Simulink.SimulationInput('TVC6DOF_sim');

    % Set simulation duration
    in(k) = in(k).setVariable('sim_time', sim_time);

    % Set perturbation parameters for this run
    in(k) = in(k).setVariable('Ix_mc', Ix_n + randn*Ix_v);
    in(k) = in(k).setVariable('Iy_mc', Iy_n + randn*Iy_v);
    in(k) = in(k).setVariable('Iz_mc', Iz_n + randn*Iz_v);
    in(k) = in(k).setVariable('m_dry_mc', m_dry_n + randn*m_dry_v);
    in(k) = in(k).setVariable('l_mc', l_n + randn*l_v);
    in(k) = in(k).setVariable('servo_delay_mc', servo_delay_n + randn*servo_delay_v);
    in(k) = in(k).setVariable('mis_y_mc', randn*mis_y_v);
    in(k) = in(k).setVariable('mis_z_mc', randn*mis_z_v);
    in(k) = in(k).setVariable('static_error_y_mc', randn*static_error_y_v);
    in(k) = in(k).setVariable('static_error_z_mc', randn*static_error_z_v);
    
end

% 3. Run with parsim in parallel
results = parsim(in, 'ShowProgress', 'on', 'UseFastRestart', 'on'); 
toc

%% Aggregate & Save Results
fprintf("Aggregating & Saving Results...\n");
tic

varNames = {'acceleration', 'angularVelocity', 'position', 'quaternion', 'velocity'};
aggregatedData = struct();

for i = 1:length(results)
    % Initialize table with the time vector ('tout')
    runTable = table(results(i).tout, 'VariableNames', {'Time'});
    
    % Add each struct's data to the table
    for j = 1:length(varNames)
        vName = varNames{j};
        runTable.(vName) = results(i).(vName).signals.values;
    end
    
    aggregatedData.(['run_' num2str(i)]) = runTable;
    writetable(runTable, sprintf('output\\run_%d.csv', i));
end

toc
fprintf("Data has been collected into struct aggregatedData and saved to \\output\n");




%% Plotting Simulated Trajectories
figure(1)
for i=1:runs
    plot3(results(i).position.signals.values(:,2),results(i).position.signals.values(:,3),results(i).position.signals.values(:,1))
    hold on
end

fprintf("Plotting...\n");
title(sprintf('%d Simulated Monte-Carlo Flight Trajectories',runs))
xlabel('Downrange (m)')
ylabel('Crossrange (m)')
zlabel('Altitude (m)')
axis equal
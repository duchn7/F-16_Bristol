clear all; close all; 
parpool(23) % Number of parallel workers. No more than the number of cores - 1

%% Generate the sweep parameters (CG position and stabilator)
sweep = [];
for i = 25:0.2:30 
    for j = -25:0.1:5
        sweep = [sweep; [i j]];
    end
end

%% Simulate
mdl = 'f16_4_sweep';
isModelOpen = bdIsLoaded(mdl);
open_system(mdl);

numSims = length(sweep);

for i = 1:numSims
    in(i) = Simulink.SimulationInput(mdl);
    in(i) = setBlockParameter(in(i), ...
                                    [mdl '/CG (%MAC)'], 'Value', num2str(sweep(i,1)),...
                                    [mdl '/de (deg)'], 'Value', num2str(sweep(i,2)));
end

tic; out = parsim(in, 'ShowProgress', 'on'); toc

%% Plot
data_plot = zeros(numSims,1);
for i = 1:numSims
        data_plot(i) = out(i).yout(end,1)*180/pi; % get the angle of attack and the end of the simulation
end

figure; scatter3(sweep(:,1), sweep(:,2) ,data_plot); 
xlabel('CG (%MAC)'); ylabel('\delta_s (deg)'); zlabel('\alpha (deg)');
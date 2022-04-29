clear
close all
clc


delete(gcp('nocreate'))


%% INPUTS

% Run Inputs
nRuns = 1;
mode = 'SERIAL';
printProgress = true;
outputFolder = [pwd, '\Output\'];
nThreads = 6;


% Add paths
addpath(genpath(pwd));


% Input File
inputFile = 'X20Taem.inp';
monteCarloFile = 'MonteCarloParams.inp';
seedFile = 'Seeds.txt';


% Sim options
saveData = false;
createPlots = true;
 

%% INITIALIZE THE MONTE CARLO RUNS

seeds = dlmread(seedFile);
seeds = seeds(1:nRuns);

% Read input file to determine the number of simulation points we need to
% preallocate random values for
inputData = readInputFile(inputFile);
nSimPoints = numel(inputData.simtStart : inputData.simdt : inputData.simtEnd);
simtEnd = inputData.simtEnd;
clear inputData;

if strcmp(mode, 'SERIAL_DETERMINISTIC')
    mcParams = zeroMcPerturbations(monteCarloFile, nSimPoints);
else
    mcParams = generateMCParams(monteCarloFile, nSimPoints, seeds, simtEnd);
end


%% RUN THE SIMULATION

% Run the simulation
if strcmp(mode, 'SERIAL') || strcmp(mode, 'SERIAL_DETERMINISTIC')
    
    for idx = 1:nRuns
        FlightSimulator(inputFile, mcParams(idx), saveData, createPlots, printProgress, idx, outputFolder);
        fprintf(['Run ', num2str(idx), ' of ', num2str(nRuns), ' Complete!\n']);
    end
    
elseif strcmp(mode, 'PARALLEL')
    
    fprintf('Starting Parallel Pool...\n');
    threadpool = parpool('local', nThreads);
    
    parfor (idx = 1:nRuns, nThreads)
        FlightSimulator(inputFile, mcParams(idx), saveData, createPlots, printProgress, idx, outputFolder);
        fprintf(['Run ', num2str(idx), ' of ', num2str(nRuns), ' Complete!\n']);
    end
    
    delete(threadpool);
    
end
    

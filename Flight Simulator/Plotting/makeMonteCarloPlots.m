function makeMonteCarloPlots(dataFolder, plotFolder, saveBool)

    
    % Load all data into a structure
    
    files = dir(dataFolder);
    files(1:2) = [];
    files = {files.name};
    files(contains(files, '.gitignore')) = [];
    files(contains(files, '.txt')) = [];
    
    files(2:2:end) = [];
    nRuns = numel(files);
    
    for idx = 1:nRuns
        files{idx}(end-3:end) = [];
        plotData(idx) = readMonteCarloData(dataFolder, files{idx});
    end
    
    rng('default')
    monteCarloColors = rand(nRuns, 3) * .75;
    
    monteCarloPlots(plotData, monteCarloColors, plotFolder, saveBool);
    
end
        
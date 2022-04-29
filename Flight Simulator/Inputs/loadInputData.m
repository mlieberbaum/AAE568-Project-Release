function inputData = loadInputData(inputFile)


    % First, read the input file
    inputData = readInputFile(inputFile);
    
    % Runway center conversion
    inputData.rwyCenter(1:2) = inputData.rwyCenter(1:2) .* pi/180;
    
    % Standard Atmosphere
    load('StandardAtmosphere.mat', 'StandardAtmosphereTable')
    inputData.standardAtmosphere = StandardAtmosphereTable;
    clear StandardAtmosphereTable;
    inputData.standardAtmosphere = inputData.standardAtmosphere(1 : inputData.standardAtmosphereInterpStep : end, :);
    
    
end
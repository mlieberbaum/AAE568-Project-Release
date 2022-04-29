function plotData = readMonteCarloData(outputFolder, files)

    headerFile = [outputFolder, files, '.hdr'];
    dataFile = [outputFolder, files, '.bin'];
    
    headers = fileread(headerFile);
    headers = split(headers, ',');
    nHeaders = numel(headers);
    
    fh = fopen(dataFile, 'r');
    data = fread(fh, 'double', 'l');
    fclose(fh);
    
    data = reshape(data, numel(data) / nHeaders, nHeaders);
    
    plotData = struct();
    
    for idx = 1:nHeaders
        if ~isempty(headers{idx})
            eval(['plotData.', (headers{idx}), ' = data(:, idx);']);
        end
    end
    
end
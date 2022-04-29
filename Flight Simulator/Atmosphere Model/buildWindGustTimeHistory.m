function windGustTimeHistory = buildWindGustTimeHistory(gustTimeTable, tVector)
    

    % No wind case
    if isempty(gustTimeTable)
        windGustTimeHistory = [];
        return;
    end
    
    
    % Build the wind gust time history
    windGustTimeHistory(:,1) = tVector.';
    windGustTimeHistory(:,2) = 0;
    nGusts = size(gustTimeTable, 1);
    
    for idx = 1 : nGusts-1
        
        idx1 = find(tVector >= gustTimeTable(idx,2), 1, 'first');
        idx2 = find(tVector <= gustTimeTable(idx,3), 1, 'last');
        
        gustTimeStart = tVector(idx1);
        gustTimeEnd = tVector(idx2);
        gustPeakMultiplier = gustTimeTable(idx, 4);
        
        T = gustTimeEnd - gustTimeStart;
        A = gustPeakMultiplier / 2;
        
        windGustTimeHistory(idx1:idx2, 2) = (A * (1 - cos(2*pi/T .* (tVector(idx1:idx2) - gustTimeStart))))';
        windGustTimeHistory(idx1:idx2, 3) = gustTimeTable(idx, 5) * (pi/180);
        
    end
    
    
    
end
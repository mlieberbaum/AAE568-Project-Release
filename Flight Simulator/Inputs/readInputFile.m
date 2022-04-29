function inputFileParams = readInputFile(inputFile)


    inputFileParams = struct();

    % Open the file
    fh = fopen(inputFile, 'r');
    
    
    if fh < 0
        error('Could not find input file')
    end
    
    
    ln = '';
    
    while ischar(ln)
        
        ln = fgetl(fh);
        
        if ~ischar(ln)
            break;
        end
        
        if isempty(ln)
            continue;
        end
        
        idxPound = strfind(ln, '#');
        
        if ~isempty(idxPound)
            idxPound = idxPound(1);
            ln(idxPound : end) = [];

            if all(isspace(ln(1:idxPound-1))) || idxPound == 1
                continue;
            end
        end
        
        eval(['inputFileParams.', ln, ';']);
        
    end
    
    
    fclose(fh);
    
    
end
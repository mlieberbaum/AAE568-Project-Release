function enumPhase = phase2enum(phase)

    switch phase
        case 'TAEM'
            enumPhase = 1;
        case 'HAC'
            enumPhase = 2;
        case 'LONG_FINAL'
            enumPhase = 3;
        case 'PRE_FLARE'
            enumPhase = 4;
        case 'SHORT_FINAL'
            enumPhase = 5;
        case 'FLARE'
            enumPhase = 6;
        otherwise
            error('Invalid flight phase');
    end
    
end
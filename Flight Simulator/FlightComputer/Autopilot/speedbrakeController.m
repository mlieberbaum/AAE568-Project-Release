function [speedbkCommand, curLevel] = speedbrakeController(aspd, aspdcmd, spdbkNom, curLevel)

    levelChangeMap = [ 2  -1000   8
                       1   10     3
                       0   5     -5
                      -1   -3    -10
                      -2   -8    1000];
    
    outputCommandMap = [ 2  -0.2
                         1  -0.1
                         0   0
                        -1  0.1
                        -2  0.2];
    
	signalError = aspdcmd - aspd;

	levelUpThreshold = levelChangeMap(levelChangeMap(:,1) == curLevel, 2);
	levelDownThreshold = levelChangeMap(levelChangeMap(:,1) == curLevel, 3);

    if (signalError > levelUpThreshold && curLevel < 2)
        curLevel = curLevel + 1;
    elseif (signalError < levelDownThreshold && curLevel > -2)
        curLevel = curLevel - 1;
    end
	
	speedbkCommand = outputCommandMap(outputCommandMap(:,1) == curLevel, 2) + spdbkNom;
    
    
end
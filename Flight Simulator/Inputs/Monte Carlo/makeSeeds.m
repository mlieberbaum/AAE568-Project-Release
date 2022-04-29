function seeds = makeSeeds(n)

    rng('default')
    seedArr = randi(9, n, 8);
    
    seeds = char(n, 8);
    
    for i = 1:n
        for j = 1:8
            seeds(i,j) = num2str(seedArr(i,j));
        end
    end
    
end
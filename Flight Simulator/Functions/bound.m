function n = bound(n, min, max)

    if n < min
        n = min;
    elseif n > max
        n = max;
    end
    
end
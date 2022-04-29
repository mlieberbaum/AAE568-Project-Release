function g = gravityModel(alt)

    g = 398600439968871 / ((alt + 6371010.0)^2);
    
end
function E = requiredVoltage(tau,dphi)
    I = 0;
    if tau > 0
        I = (1/Km) * ((tau/(eta*N))-tm);
    elseif tau < 0 
        I = (1/Km) * ((tau/(eta*N))+tm);
    elseif tau == 0
        return;
    end
    E = I * Ra + Km * dphi;

end